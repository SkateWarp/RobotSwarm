# Swarm control plan

## Document status

This is the current target architecture and release plan for the commissioning
branch. It supersedes the earlier design that treated public WHEP/ICE/TURN and
a manually confirmed empty GPU worker as prerequisites. The implementation is
present, but the feature-gated viewer and new control-plane changes are not
considered production-accepted until the final two-user test and exact-revision
deployment finish.

See the [implementation status](../IMPLEMENTATION_STATUS.md) for that boundary
and the
[Spanish commissioning report](informe-comisionamiento-final.md) for the
chronology, incidents, screenshots, and raw evidence.

## Target layout

Cloudflare Workers hosts the React frontend at `https://rs.zerav.la`. Its Git
integration builds and deploys the frontend after commits to the configured
production branch. The self-hosted backend runner does not build or serve the
frontend.

The LAN VM remains the control plane:

- .NET API at `https://robot.zerav.la`, reached through Nginx Proxy Manager;
- PostgreSQL on the private Compose network;
- session queue, worker registry, task-outcome monitor, and viewer/drain leases;
- MediaMTX for authenticated RTSP ingest and low-latency HLS conversion; and
- an authenticated HLS reverse proxy in the backend, while MediaMTX port 8888
  remains private.

The Windows/WSL machine remains the compute worker:

- one isolated Docker container per user simulation session;
- one ROS master, Gazebo server, fleet, and task orchestrator per container;
- RTX rendering access plus bounded CPU, memory, and robot capacity;
- an outbound authenticated SignalR connection to the API;
- an optional, lease-owned private X display and `gzclient` sidecar for each
  active scene viewer; and
- no ROS master, Gazebo master, Docker API, or VNC port exposed to a browser.

VNC and websockify are maintenance paths bound to loopback and reached through
an SSH tunnel. They are not used to give end users a shared screen.

## Session and command flow

1. An authenticated user creates a session and chooses a robot count from 1 to
   10.
2. The API validates ownership and capacity, then puts the session in a FIFO
   queue.
3. An online, compatible worker with free capacity receives a durable provision
   command.
4. The worker creates the isolated container and waits for the requested
   TurtleBot3 Burger roster.
5. The owner can resize the fleet, start a task, pause, resume, cancel, or use
   the emergency stop through the backend. The browser never publishes directly
   to ROS.
6. The worker correlates commands and ROS reports with the backend session and
   task IDs. Fleet and task status return over the worker hub.
7. Stopping the session removes its viewer publisher, container, private Docker
   network, task state, and leases without stopping another user's session.

Worker commands are durable and acknowledged. A system-generated task cancel
must finish or remain an explicit barrier before the session can accept another
task; a stale worker report or command envelope must not resurrect a task that
the control plane already marked terminal. These race conditions are part of
the final integrated regression review, not assumptions delegated to the user.

## Task outcome monitoring

ROS reports task acceptance, progress, result evidence, and terminal state. The
backend stores the last report and the last real progress time separately so a
repeated heartbeat cannot hide a stalled finite task.

The control-plane monitor applies bounded timeouts to tasks that remain queued,
remain accepted without starting, or run without progress. When one expires it
records a terminal outcome and issues a correlated cancel. Follow-the-leader is
continuous by design and is therefore excluded from finite progress completion
timeouts.

Transport completion is stricter than a generic `DONE` string. The accepted
result must be correlated to the task, include discovery evidence and the
expected notifications, and confirm useful participation by the complete
requested fleet. Private per-robot diagnostics stay on the worker/test side;
only bounded, validated summary fields cross the control plane.

## Worker-to-ROS heartbeat

Each running worker session publishes `std_msgs/Empty` on
`/swarm/control_heartbeat` at least once every two seconds. The publisher is
owned by the worker process rather than a detached command in the container, so
pulses stop when the worker loses control. Its loop runs outside the serialized
task/fleet executor so provisioning or resize work cannot create a false
expiry.

The task orchestrator measures arrival with wall-clock monotonic time. It stays
disarmed until the first pulse, then latches the normal ROS emergency-stop path
if no pulse arrives within `~control_heartbeat_timeout` (10 seconds by default).
A late pulse does not silently clear the stop. The worker must restore pulses
and explicitly issue `reset_emergency_stop` before starting another task.

## Algorithms

### Follow the leader

The leader follows the selected path. Followers use points at fixed distances
along the leader's travelled path, so the chain grows naturally with the fleet
size. Each robot receives collision avoidance, velocity limits, and smoothed
final commands.

### Figures and letters

Geometric outlines and glyph strokes are resampled for the active fleet. A
minimum-cost robot-to-slot match includes a small hysteresis penalty, reducing
long crossings when the formation is recalculated. The controller can search
for a safer nearby centre when a requested outline intersects an obstacle.

### Collaborative transport

All active robots begin with distributed search motion while the payload
position is unknown. The first finder publishes a task-correlated notice with
the object coordinates. The other robots acknowledge the notice, leave search,
and rendezvous together.

The default planner is the clean-room GRF implementation based on the
[IROS paper and reference repository](../THIRD_PARTY_NOTICES.md); the earlier
planner remains an explicit runtime fallback. Every robot must make useful
pushing contact before the task can be accepted. When the payload contour has
fewer safe contact positions than robots, bounded companion chains let the
remaining robots transmit force through another robot. Direct payload contact
has priority, and the controller does not form a one-by-one waiting queue.

Obstacle and inter-robot avoidance remain active. The safety layer distinguishes
confirmed payload/companion contact from a closer unrelated wall or robot, so
the intended push is allowed without masking environmental collision hazards.

## Visible simulation observations obtained so far

The working log records the following visible samples at approximately a 3.0
physics real-time factor:

- formations: triangle/3, square/5, letter A/7, letter V/8, diamond/9, and
  letter S/10;
- follow-the-leader: square/6, circle/3, and one full figure-eight lap/10; and
- collaborative transport: 1, 3, 4, and 10 robots.

A separate ten-robot search probe recorded motion from every robot, one
discovery notice, acknowledgements from all ten robots, and responses from all
nine non-finders. The accepted transport repetition then demonstrated
simultaneous rendezvous and useful full-fleet contribution through two
payload-contact roots and companion chains. In that transport run the payload
advanced 0.922 m at 0.9997 directional efficiency, with zero collision deltas
and an under-load real-time factor of 2.9505.

The ten-robot figure-eight case completed with zero robot/environment
collisions, 0.3009 m minimum obstacle clearance, 0.4293 m minimum inter-robot
distance, 0.0528 m spacing error, and 0.8992 m/s² peak acceleration.

These are representative acceptance cases, not an exhaustive proof of every
shape, path, world, parameter combination, or number of concurrent sessions.
The repository retains structured raw evidence for the N=10 transport and
search runs. The wider matrix remains a historical observation until selected
cases are repeated through the final public deployment.

For sequential Gazebo cases, the acceptance harness reuses `tb3_0` through
`tb3_{N-1}`. Between cases it verifies that the fleet roster is empty, departed
models no longer appear in `/gazebo/model_states`, and per-robot `cmd_vel`
monitors were unregistered. This bounds ROS-side state even though Gazebo
Classic can retain allocator and plugin caches after model deletion.

Ground-truth static-obstacle clearance defaults to 0.13 m from the robot centre
to the obstacle surface. This lies just beyond the Burger body/contact threshold
(approximately 0.125 m), so an overlapping model cannot pass. The explicit
`--min-obstacle-clearance` option can raise the commissioning threshold.

## Per-user browser viewer

The primary target path no longer depends on public WHEP connectivity:

```text
session Gazebo server
    -> lease-owned private Xvfb + gzclient
    -> FFmpeg H.264
    -> authenticated RTSP publish to MediaMTX
    -> private low-latency HLS origin
    -> authenticated backend HLS proxy
    -> hls.js at rs.zerav.la
```

The viewer lease owns a canonical session path, short-lived publish/read tokens,
expiry, private display, and publisher process group. The browser attaches the
read token to every playlist and MP4-part request. The backend accepts only the
canonical path for that lease, rejects redirects and traversal, bounds response
time and size, and marks media responses private/no-store. On the private origin
hop it replaces the lease with a separate protected CDN credential, which avoids
MediaMTX's browser-session cookies without exposing either credential publicly.

This HLS route avoids a new public stream hostname, public UDP ports, ICE host
advertisement, and TURN. It trades those connectivity requirements for somewhat
higher latency and backend bandwidth. WHEP can remain an optional secondary
transport; TURN is required only if that future public WebRTC route cannot
establish direct connectivity.

The stock Burger model has no supported robot-camera sensor, so the first
capability is `Scene` only. A private per-session `gzclient` is mandatory; a
shared WSLg root display or shared VNC desktop does not satisfy user isolation.

The local host integration test runs two publisher pipelines concurrently with
different displays, paths, and cleanup. It uses real Xvfb/XTest but controlled
Docker and `gzclient` test doubles, so it proves the publisher's local isolation
logic rather than two GPU streams. Production feature gates must remain false
for general use until two independently authenticated public browser sessions
also prove playback, cross-user denial, lease expiry, and stop isolation. They
may be enabled only for a supervised acceptance window and must be disabled
again if that acceptance is not completed.

## Deployment drain

The GPU deployment remains a manually dispatched, protected maintenance
workflow, but draining is automatic. The operator supplies the full current
`main` SHA; the workflow does not accept a “zero sessions confirmed” checkbox.

The workflow:

1. requires a successful `Check project` push run for that exact SHA and
   requires it to remain the current `main` head;
2. acquires a worker-authenticated two-hour drain lease tied to the SHA;
3. puts the worker in `Draining`, excluding it from scheduling;
4. waits for both the backend's tracked session count and a post-request worker
   Docker report to reach zero;
5. runs worker, viewer-helper, rollback, and ROS tests, then builds and probes
   the exact immutable image;
6. revalidates revision, lease lifetime, and both zero-session reports
   immediately before the atomic release switch; and
7. releases the lease only after the new worker registers and produces its
   first readiness heartbeat.

Pre-deployment failures release a lease they acquired. If the atomic worker
activation itself fails, the deployment restores the previous release where
possible and retains a safe draining condition for operator investigation.

When a revision changes backend/worker contracts, deploy the compatible backend
first. The drain route must exist before dispatching a GPU workflow that uses
it.

## Production network boundary

`robot.zerav.la` terminates TLS at Nginx Proxy Manager. The repository variable
`BACKEND_PUBLISH_IP` must be the VM's fixed LAN address, and NPM proxies to
`http://10.0.0.126:44336`. Port 44336 stays LAN-only and should accept traffic
only from the NPM source. WebSocket support remains enabled on that NPM host for
SignalR upgrades.

PostgreSQL is not published by Compose. MediaMTX HLS port 8888 is also not
published. RTSP ingest port 8554 is bound to the backend host's LAN address and
requires a path-scoped publishing token; no source-IP firewall rule is claimed.
VNC 5901 and websockify 6080 remain on loopback for SSH-tunnel maintenance.

The backend accepts forwarded client addresses only from loopback and the exact
addresses listed under `ReverseProxy:KnownProxies`. The protected production
variable `BACKEND_PROXY_IP` supplies Nginx Proxy Manager's source address. The
address observed on the current Compose bridge is `10.0.0.2`; the deployment
validates that the variable contains an IP address and fails before changing a
container when it is absent or malformed. This boundary lets rate limits
distinguish clients instead of placing every user in one proxy-wide bucket.

Backend deployment reads database, JWT, and bootstrap values from the protected
GitHub `production` environment. Media/listen addresses and exact boolean viewer
gates are protected environment variables. Invalid viewer-gate strings fail the
deployment rather than silently enabling a path.

## Remaining release sequence

1. Preserve the current rollback tag and publish the locally reviewed tree;
   require that exact revision to pass GitHub CI.
2. Deploy backend/database changes with viewer publishing and HLS proxy gates
   still false; verify health, migration, worker registration, and drain API.
3. Dispatch the GPU workflow for that same current `main` SHA and verify the
   active revision, image ID, service readiness, zero residual sessions, and
   transport-evidence capability version 1.
4. Set the protected production variable
   `REQUIRE_COLLABORATIVE_TRANSPORT_EVIDENCE=true`, redeploy the same current
   SHA, and leave the gate enabled after one roster-correlated transport result
   is accepted. Until this step it remains false for backend-first compatibility.
5. Enable the local worker publisher identity and the two protected backend
   viewer gates after the RTSP prerequisite probe succeeds.
6. Use two separate browser profiles and accounts to run simultaneous sessions
   with different robot counts/tasks. Verify different displays/streams,
   cross-user `401/403`, token expiry, independent stop, task outcomes, visible
   FPS, and physics real-time factor.
7. If any isolation or outcome criterion fails, turn both viewer gates off and
   use the recorded rollback revision; do not make VNC the user-facing fallback.
8. Remove temporary test accounts/sessions and complete the commissioning
   report with the accepted “after” screenshots and final revision.

## Deferred work

- Add public WHEP and TURN only if its lower latency justifies the additional
  DNS, UDP, ICE, and relay operations.
- Add a real robot-camera sensor and a separately authorized `RobotCamera`
  path if the platform requires it.
- Introduce an explicit backend/worker protocol-version negotiation beyond the
  current compatible-deployment order.
- Automate bounded image/release garbage collection without deleting images
  referenced by a rollback release.
- Plan migration from end-of-life ROS Noetic and Gazebo Classic to ROS 2 and a
  supported Gazebo release.
