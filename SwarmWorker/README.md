# SwarmWorker

`SwarmWorker` is the outbound GPU compute agent for the RobotSwarm control
plane. It connects to the backend SignalR worker hub, pulls durable commands,
and owns isolated ROS/Gazebo Docker sessions. The browser and backend never
receive access to the Docker socket, ROS master, or Gazebo ports.

## Prerequisites

- Windows 11 with WSL2 and an Ubuntu distribution.
- Docker Desktop with **Use the WSL 2 based engine** and this distribution
  enabled under **Resources > WSL Integration**.
- A current NVIDIA Windows driver with WSL CUDA support.
- .NET 8 SDK inside WSL.
- A locally available, preferably digest-pinned ROS image built from
  `swarm_ws/Dockerfile`.

Validate the runtime before starting the worker:

```bash
docker version
docker run --rm --gpus device=0 nvidia/cuda:12.6.3-base-ubuntu22.04 nvidia-smi
docker build -t robotswarm/ros-noetic:local -f swarm_ws/Dockerfile swarm_ws
docker image inspect --format='{{.Id}}' robotswarm/ros-noetic:local
```

The ROS image is deliberately not pulled from a command payload. Image
selection is worker configuration so an authenticated backend command cannot
make the worker execute an arbitrary image.

## Enrollment and configuration

The backend enrolls a worker and displays a token once:

```text
<worker-guid>.<base64url-secret>
```

Split that value into `WorkerId` and `WorkerSecret`. Store the secret in the
WSL service environment or a root/user-only environment file, never in Git or
`appsettings.json`.

Required settings:

```bash
export Worker__BackendUrl="https://api.example.test"
export Worker__WorkerId="00000000-0000-0000-0000-000000000000"
export Worker__WorkerSecret="replace-with-enrollment-secret"
export Worker__Name="gpu-worker-01"
export Worker__MaxConcurrentSessions="4"
export Worker__SessionImage="robotswarm/ros-noetic@sha256:replace-with-digest"
export Worker__ImageVersion="2026.07.16"
```

Digest pinning is required by default. A mutable local tag can be used for
development only by setting `Worker__AllowMutableSessionImage=true` together
with a unique `Worker__ImageVersion` for every rebuild.

For a local HTTP development backend only:

```bash
export Worker__BackendUrl="http://127.0.0.1:5000"
export Worker__AllowInsecureTransport="true"
```

Run:

```bash
dotnet run --project SwarmWorker/SwarmWorker.csproj
```

Production should run this command from an auto-starting WSL systemd user
service or a Windows scheduled task that starts the WSL service after Docker
Desktop is ready. Protect the service environment with owner-only
permissions.

## Runtime guarantees

- SignalR uses the enrolled bearer token and automatic reconnect.
- `CommandAvailable` is only a wake-up notification; command payloads are
  always pulled from `PullPendingCommands`.
- Commands are acknowledged, marked running, and completed or failed through
  durable backend methods. Backend redelivery plus the command idempotency key
  makes restart recovery safe.
- The executor is bounded globally and serializes commands for the same
  session.
- Provisioning discovers existing containers by managed/session labels before
  creating anything.
- Task commands are published to `/swarm/commands` as JSON through a fixed
  `docker exec` script. Command JSON remains a separate process argument and
  is never interpolated into a shell command.
- The worker reports immediate task state changes when task commands complete,
  then polls `/swarm/status` every two seconds and reports only state changes
  or configured progress increments.
- Every session receives a dedicated internal Docker bridge network and no
  published ROS/Gazebo ports.
- Containers are non-privileged, capability-dropped, read-only, PID/CPU/RAM
  limited, and run as a configured non-root uid/gid.
- Worker shutdown stops pulling commands and drains the executor. Session
  containers remain available for restart recovery, but their ROS watchdog
  emergency-stops motion when worker heartbeats cease.
- If backend contact is lost for 30 seconds, the independent safety monitor
  emergency-stops every running ROS session. If ROS cannot acknowledge the
  stop, the worker stops that session container and reports the failure after
  reconnecting.
- Control-heartbeat pulses stop when backend contact is older than 15 seconds.
  ROS then latches its own watchdog stop after the configured 10-second
  timeout, covering a dead worker or a stale worker lease.

## Current scaffold boundaries

- `ProvisionSession`, `UpdateFleet`, `StartTask`, `PauseTask`, `ResumeTask`,
  `CancelTask`, `EmergencyStop`, `ResetEmergencyStop`, and `StopSession` are
  implemented. `SetViewerSource` remains unsupported.
- Backend task types map to ROS as `FollowLeader` -> `follow_leader`, `Figure`
  -> `formation`, and `CollaborativeTransport` -> `transport`. The backend
  task-run UUID is always sent as ROS `task_id`; caller-supplied `task_id` and
  `task_type` parameter fields cannot override those values.
- Task status polling recovers active tasks from `/swarm/status` after a worker
  restart, then resumes state and progress reporting with the task ID supplied
  by ROS.
- Fleet updates replace the complete robot roster. The current ROS topic API
  cannot safely place incremental additions relative to occupied positions;
  full replacement avoids spawning robots on top of one another.
- The container is headless and publishes no ports. WebRTC scene/robot-camera
  encoding, per-session VNC, TURN, and viewer leases remain separate work.
- Gazebo Classic GPU rendering under WSL/NVIDIA still needs an integration
  benchmark. `--gpus` exposes the device, but the ROS image does not yet
  include a verified EGL/VirtualGL/X server path.
- Managed containers launch `swarm_main.launch` with
  `start_legacy_bridge:=false`; the worker is the sole backend control-plane
  adapter for those sessions.
- Completion state is durable in the backend, not in a local worker database.
  If the worker is terminated before reporting completion, the backend
  redelivers the command and the Docker operation reconciles idempotently.
