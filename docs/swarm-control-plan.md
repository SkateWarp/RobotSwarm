# Swarm control plan

## Target layout

Cloudflare Workers hosts the React frontend at `https://rs.zerav.la`. Its
GitHub integration builds and deploys the frontend after commits to the
configured production branch.

The LAN VM remains the API control plane:

- .NET API at `https://robot.zerav.la`
- PostgreSQL
- session queue and worker registry
- MediaMTX for per-session WebRTC paths
- Cloudflare tunnel for the API and WebRTC signaling

The Windows/WSL machine is the compute worker:

- one Docker container per user session
- one ROS master, Gazebo world, fleet, and task orchestrator per container
- RTX access and fixed CPU/RAM limits
- outbound authenticated SignalR connection to the API
- no ROS, Gazebo, or Docker ports exposed to users

VNC is only for maintenance through an SSH tunnel. The browser viewer uses a
short-lived lease and a stream path owned by the current session.

## Session flow

1. A user creates a session and chooses the robot count.
2. The API puts it in a FIFO queue.
3. An online worker with free capacity receives a durable provision command.
4. The worker creates an isolated container and waits for the requested Burger
   roster.
5. The user can resize the fleet, start a task, pause, resume, cancel, or use
   the emergency stop.
6. ROS status is reported back through the worker hub.
7. Stopping the session removes its container, network, active task, and viewer
   leases.

## Worker-to-ROS heartbeat

Each running worker session publishes `std_msgs/Empty` on
`/swarm/control_heartbeat` at least once every two seconds. The publisher must
not be a detached process inside the session container: pulses must stop when
the worker process loses control of the session. The heartbeat loop must also
run outside the serialized task/fleet command executor so a long provisioning
or resize operation cannot create a false lease expiry.

The task orchestrator measures arrival with wall-clock monotonic time. It stays
disarmed until the first pulse, then latches the normal ROS emergency-stop path
if no pulse arrives within `~control_heartbeat_timeout` (10 seconds by
default). A late pulse does not clear the stop. The worker must restore pulses
and explicitly issue `reset_emergency_stop` before starting a new task.

## Algorithms

### Follow the leader

The leader follows the selected path. Followers use points at fixed distances
along the leader's travelled path, so the chain length grows naturally with
the number of robots. Collision avoidance is applied to every final command.

### Figures and letters

Geometric outlines and glyph strokes are resampled for the current fleet
size. Robot-to-slot assignment uses a minimum-cost match with a small
hysteresis penalty, which reduces long crossings when the formation is
recomputed.

### Collaborative transport

The default controller is the clean-room GRF implementation based on the
referenced paper and repository. It uses local range/line-of-sight neighbors,
object contour tangents, and a lower proposal count for larger fleets. The
previous controller remains available as a runtime fallback.

## Commissioning order

1. Preserve the VM runner's dirty checkout and remove manual/generated files
   from the deployment workspace.
2. Rotate the committed database/JWT values and the exposed tunnel token.
3. Restrict PostgreSQL, VNC, and websockify to loopback or the required LAN
   source.
4. Start Docker Desktop and enable WSL integration for this distribution.
5. Build the ROS image and verify `docker run --gpus all ... nvidia-smi`.
6. Run one headless session, then four sessions with ten robots each.
7. Add the Gazebo X display and H.264 publisher. Validate browser playback,
   lease expiry, and cross-user denial before exposing WebRTC publicly.
8. Add TURN if clients cannot reach the MediaMTX ICE port directly.
9. Only then move the branch through CI and the production deployment
   workflows.

Frontend commits are deployed by Cloudflare's repository integration. The
self-hosted GitHub runner does not build or serve the React application. The
backend workflow performs its path check on a hosted runner first, so a
frontend-only commit does not reach or restart the backend VM.

GPU worker deployment remains a manually approved maintenance action until
the backend has a real draining state. Stop new scheduling, wait for zero
managed sessions, deploy any compatible backend changes first, and then run
the GPU workflow with the exact successful `main` SHA.

The backend workflow reads `DB_USER`, `DB_PASSWORD`, `DB_NAME`, and
`APPSETTINGS_SECRET` from the protected GitHub `production` environment. Copy
the current live values there before the tracked `.env` removal reaches
`main`. Media addresses and the future public viewer URL are repository
environment variables rather than secrets.

Gazebo Classic and ROS Noetic are kept for compatibility with the current
project. They should stay pinned inside the worker image while a ROS 2 / modern
Gazebo migration is evaluated separately.
