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
  latches every running session before asking ROS to emergency-stop. The same
  latch cancels active work and rejects queued motion/fleet commands until a
  confirmed reset or stop. If ROS cannot acknowledge the stop, the worker
  stops that session container and reports the failure after reconnecting.
- Control-heartbeat pulses stop when backend contact is older than 15 seconds.
  ROS then latches its own watchdog stop after the configured 10-second
  timeout, covering a dead worker or a stale worker lease.

## Current scaffold boundaries

- `ProvisionSession`, `UpdateFleet`, `StartTask`, `PauseTask`, `ResumeTask`,
  `CancelTask`, `EmergencyStop`, `ResetEmergencyStop`, `StopSession`, and the
  `SetViewerSource` control path are implemented. Viewer capability is still
  advertised only after the configured publisher helper passes its runtime
  probe; the default configuration keeps it disabled.
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
- The simulation container remains isolated and publishes no ports. A viewer
  helper must create a private per-session display and a visible `gzclient`;
  attaching every session to the shared WSLg `:0` display is not supported.
  The shipped helper advertises `Scene` only after its host probe succeeds;
  the current TurtleBot image has no supported robot-camera sensor.
- The visible WSL/NVIDIA preflight has been exercised on the commissioning PC:
  the RTX 3080 rendered the Gazebo viewport at 58.6 average FPS while physics
  held a 2.996 real-time factor. Deployment should rerun the preflight after
  driver, Docker Desktop, WSL, or ROS-image changes.
- Managed containers launch `swarm_main.launch` with
  `start_legacy_bridge:=false`; the worker is the sole backend control-plane
  adapter for those sessions.
- Completion state is durable in the backend, not in a local worker database.
  If the worker is terminated before reporting completion, the backend
  redelivers the command and the Docker operation reconciles idempotently.

## Viewer publisher helper contract

Viewer publishing is an external, locally configured process. The executable
is never selected by a backend command. Keep `Worker__Viewer__Enabled=false`
until the shipped helper and full media path pass the host smoke test.

The worker probes the helper with:

```text
<publisher> probe --protocol-version 2
```

The probe must exit successfully and print one JSON object like:

```json
{"protocolVersion":2,"ready":true,"videoCodec":"H264","sources":["Scene"],"interactive":true}
```

Do not report `RobotCamera` until the TurtleBot model has a tested camera
sensor/topic. For a viewer command the worker launches the helper with
separate, non-shell arguments containing the enrolled worker ID, the
session/container identity, lease ID and expiry, canonical source/path, and
configured RTSP base URL. The
probe receives no backend or media credential. Each `SetViewerSource` command
instead carries its own short-lived, publish-only token. The worker validates
that token and writes it as the first bounded line of the publishing helper's
redirected standard input. Protocol 2 keeps that private pipe open for
validated JSONL mouse and keyboard events associated with the same active
session and lease; a replacement or stop closes it with the helper. The probe
receives closed, empty input. Neither the long-lived worker credential nor the
publish token is placed in an environment, process arguments, or logs. The helper's loopback
RTSP proxy path-checks FFmpeg requests and injects MediaMTX's `token` query
upstream in memory.

The helper's first stdout line must be exactly `READY`, and only after the
private display, `gzclient`, H.264 encoder, and MediaMTX publisher are live.
It must remain in the foreground and own those child processes instead of
daemonizing them; stopping the helper must stop the whole display/capture
pipeline for that session.
The worker completes `SetViewerSource` only after that marker. It kills the
publisher when the lease expires, when another source replaces it, when the
session stops, or when the worker service shuts down.
On Linux and macOS, normal stops first send the helper `SIGTERM` so it can
remove its private display and encoder processes. The worker waits up to
`Worker__Viewer__StopTimeoutSeconds` (five seconds by default), then force-kills
the process tree if the helper has not exited. Failed probes and failed startup
attempts are force-killed immediately.
After `READY`, an unexpected helper exit is watched and restarted for the same
lease up to three times with one-, two-, and four-second delays. An intentional
stop, replacement, expiry, or worker shutdown cancels recovery. Exhausting the
budget leaves the publisher stopped instead of spinning indefinitely.
The publishing helper owns a private POSIX process group. Before recovery the
worker reaps that exact group with bounded `SIGTERM`/`SIGKILL` escalation, so
children left behind by a hard-killed helper cannot collide with its
replacement. A private per-lease lock serializes ownership, and only a
same-owner mode-0700 stale runtime directory is eligible for reclamation.

`gzclient` runs directly on the WSL host; the helper never creates, stops, or
removes a viewer sidecar. Xvfb, FFmpeg, and the outer `bwrap` monitor remain in
the helper's process group. Bubblewrap's `--new-session` places its inner
monitor and the real gzclient in a separate session, while
`--die-with-parent` guarantees that hard-killing the outer monitor also kills
that inner tree. The helper read-only inspects
the assigned worker-managed container and its sole internal network, binds
both ownership labels to the worker/session IDs supplied by the worker,
verifies the exact private IPv4 attachment from both container and network
metadata, verifies that the unique IPAM gateway is locally assigned, sets
`GAZEBO_MASTER_URI` to the container address on port 11345, and exports both
`GZ_IP` and Gazebo Classic's effective `GAZEBO_IP` as that host gateway. A
lease-private
MIT-cookie-protected Xvfb display keeps input and pixels isolated. Production
uses Xorg's Linux abstract Unix endpoint with `DISPLAY=:N`, explicitly starts
Xvfb with `-nolisten tcp`, and verifies the authenticated display is usable
without occupying port `6000+N`. WSLg's read-only `/tmp/.X11-unix` mount can
therefore emit a harmless filesystem-socket warning; the abstract endpoint
remains usable even with systemd `PrivateTmp=true`. The Gazebo window must be
the one stable, viewable, non-transient NORMAL direct child of the X root;
selection-owner, nested Qt, and SPLASH windows are ignored. libX11 moves it to `0,0`,
resizes it to the complete capture surface, focuses it, and verifies the
resulting geometry before FFmpeg starts. Validated browser events are then
injected only into this private display through XTest.

The immutable ROS image and host `gzclient` must both use Gazebo Classic 11.
The release packages `librobotswarm_gazebo_gui_probe.so`, TurtleBot description
meshes, Gazebo models, and RobotSwarm models from that exact image. An
unprivileged `bwrap` mount namespace overlays the packaged ROS share at
`/opt/ros/noetic/share` for `gzclient` only. This is required because gzserver
publishes absolute TurtleBot mesh paths; `GAZEBO_MODEL_PATH` alone cannot make
those visuals appear, and the worker must not require root or modify host
`/opt`. The GUI plugin applies `ROBOTSWARM_VIEWER_RENDER_RATE` to both the
active user camera and Gazebo's GUI render event. The helper does not emit
`READY` until that exact gzclient process reports a hardware D3D renderer and
both measured rates meet `ROBOTSWARM_VIEWER_MIN_RENDER_RATE` (50 FPS cap and
45 FPS minimum by default).

The `bwrap` boundary isolates files, mounts, PIDs, IPC, and the child session;
it deliberately uses `--share-net` so host gzclient can reach the validated
Docker-bridge address. It is not a network-egress sandbox: gzclient inherits
the worker user's host network access. Production therefore trusts the
versioned host Gazebo 11 binary, packaged plugin, and immutable session image;
the exact-IP checks constrain the configured ROS/Gazebo master but do not
firewall a compromised client.

Required settings after the production media path is approved:

```bash
export Worker__Viewer__Enabled="true"
# The GPU deployment supplies an absolute, versioned PublisherExecutable path.
export Worker__Viewer__PublishBaseUrl="rtsp://10.0.0.126:8554"
export Worker__Viewer__StopTimeoutSeconds="5"
export ROBOTSWARM_VIEWER_DISPLAY_TRANSPORT="unix"
export ROBOTSWARM_VIEWER_RENDER_RATE="50"
export ROBOTSWARM_VIEWER_MIN_RENDER_RATE="45"
export ROBOTSWARM_VIEWER_GPU_ADAPTER_NAME="NVIDIA"
```

The production workflow ships the helper with mode `0755`, and the deployment
script writes its absolute, versioned path into every release. The stable
identity file owns the enable switch and the local render settings. When that
switch is `true`, deployment requires the RTSP endpoint, packaged GUI/assets,
and a successful host helper probe. `PrivateTmp=true` remains enabled because
no Docker sidecar needs access to the X endpoint. The helper accepts Unix
display transport only and never opens an X11 network listener.
