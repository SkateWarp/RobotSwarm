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

## Behavior acceptance result

The commissioned 1-10 robot range passed the following visible simulation
matrix at roughly a 3.0 physics real-time factor:

- formations: triangle/3, square/5, letter A/7, letter V/8, diamond/9, and
  letter S/10;
- follow-the-leader: square/6, circle/3, and a full figure-eight lap/10; and
- collaborative transport: 1, 3, 4, and 10 robots, including search motion by
  every robot, discovery notification, simultaneous rendezvous, and full-fleet
  contribution through at most two payload-contact roots.

The ten-robot figure-eight case completed with zero robot/environment
collisions, 0.3009 m minimum obstacle clearance, 0.4293 m minimum inter-robot
distance, 0.0528 m spacing error, and 0.8992 m/s² peak acceleration.

## Visible simulation acceptance

For an N-robot case, the sequential Gazebo matrix reuses `tb3_0` through
`tb3_{N-1}` instead of making new model namespaces for every case. Between
cases it verifies that the fleet roster is empty, the departed models have
disappeared from
`/gazebo/model_states`, and its per-robot `cmd_vel` monitors were unregistered.
This bounds ROS-side state even though Gazebo Classic can retain allocator and
plugin caches after dynamic model deletion.

Ground-truth static-obstacle clearance defaults to 0.13 m from the robot
centre to the obstacle surface. That is just beyond the Burger body/contact
threshold (roughly 0.125 m), so a case cannot pass while the model is already
overlapping an obstacle. An explicit `--min-obstacle-clearance` still overrides
the commissioning default.

## Commissioning order

1. Preserve the VM runner's dirty checkout and remove manual/generated files
   from the deployment workspace.
2. Rotate the committed database/JWT values and the exposed tunnel token.
3. Restrict PostgreSQL, VNC, and websockify to loopback or the required LAN
   source.
4. Start Docker Desktop and enable WSL integration for this distribution.
5. Build the ROS image and verify `docker run --gpus all ... nvidia-smi`.
6. Run one isolated session with its private Gazebo display, then exercise
   concurrent sessions up to the commissioned GPU/CPU capacity.
7. Enable the H.264 publisher only after validating browser playback,
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

`robot.zerav.la` terminates TLS at Nginx Proxy Manager. Set the repository
variable `BACKEND_PUBLISH_IP` to the backend VM's fixed LAN address so NPM can
proxy to `http://10.0.0.126:44336`. Port 44336 must stay LAN-only and should be
accepted only from the NPM host. PostgreSQL is not published by Compose and
must remain private. Keep WebSocket support enabled on the NPM proxy host so
the SignalR hubs can upgrade their connections.

Login, registration, and session creation have separate fixed-window limits.
The backend accepts forwarded client addresses only from loopback and the
exact addresses listed under `ReverseProxy:KnownProxies`. If Nginx Proxy
Manager does not connect from loopback, set
`ReverseProxy__KnownProxies__0` to its source address before commissioning the
login limit; otherwise every browser would share the proxy's rate-limit
bucket.

Gazebo Classic and ROS Noetic are kept for compatibility with the current
project. They should stay pinned inside the worker image while a ROS 2 / modern
Gazebo migration is evaluated separately.
