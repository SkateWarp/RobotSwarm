# RobotSwarm implementation status

**Snapshot date:** 2026-07-19
**Scope:** current commissioning branch, not an assertion that every change is
already active in production

The old version of this file described the task orchestrator, the three
behaviours, launch files, worlds, and RViz configuration as missing future
work. Those components now exist. This document separates what is implemented
in the repository from what has been demonstrated in a visible simulation and
what still requires final production acceptance.

The detailed Spanish commissioning log, including failed attempts and visual
evidence, is available in
[docs/informe-comisionamiento-final.md](docs/informe-comisionamiento-final.md).

## Framework alignment

| Layer | Current responsibility | Repository status |
| --- | --- | --- |
| React frontend | User-owned session, fleet and task controls; live status; private HLS scene player | Integrated in the commissioning branch |
| .NET backend | Authentication, ownership checks, session queue, worker commands, task outcomes, viewer leases and deployment drain leases | Integrated in the commissioning branch |
| .NET GPU worker | Isolated Docker session lifecycle, ROS command bridge, heartbeats, immutable image selection and viewer-publisher supervision | Implemented; final release deployment remains pending |
| ROS Noetic and Gazebo | Dynamic TurtleBot3 Burger fleet, safety control, task orchestration and three scalable behaviours | Implemented and exercised in visible simulation |
| MediaMTX and viewer helper | Private X display and `gzclient` per lease, H.264/RTSP publishing and internal low-latency HLS origin | Implemented with feature gates disabled by default |

The layers use one correlated lifecycle rather than separate ad-hoc controls:
the frontend calls the authenticated backend, the backend issues durable worker
commands, the worker translates those commands into the private ROS namespace,
and ROS reports fleet and task state through the worker hub. A browser is not
given Docker, ROS, Gazebo, VNC, or MediaMTX administrative access.

Contract alignment is therefore implemented. Final production acceptance is
not complete until the same revision passes CI, is deployed to both the backend
VM and GPU worker, and two independent browser users prove session and stream
isolation.

## Implemented components

### ROS package and simulation

The package under `swarm_ws/src/robot_swarm_bridge` contains:

- generated messages and services for fleet and task control;
- `scripts/core/task_orchestrator.py`, including command validation, task-ID
  correlation, pause/resume/cancel, emergency stop, status reporting, and a
  fail-closed control heartbeat;
- dynamic fleet management and normalized `tb3_0` through `tb3_9` runtime IDs;
- obstacle and inter-robot avoidance applied to final motion commands;
- launch files for the main swarm, follow-the-leader, formation, and transport
  scenarios;
- `empty_arena.world`, `swarm_arena.world`, the transport payload model, and an
  RViz configuration; and
- unit and live-acceptance probes for algorithms, physics, cleanup, GUI
  rendering, safety clearance, search notification, and payload transport.

### Scalable behaviours

All three requested behaviours are present and calculate their assignments from
the active fleet instead of requiring a fixed robot count:

1. **Follow the leader.** Followers sample the travelled leader path at
   distance offsets, producing a continuous chain whose length follows the
   fleet size.
2. **Figures and letters.** Geometry and glyph strokes are resampled to the
   number of active robots, then matched to robots with a minimum-cost
   assignment and crossing-reduction hysteresis.
3. **Collaborative transport.** Every robot searches while the object is
   unknown. The first finder publishes the correlated object position; the
   remaining robots acknowledge it and rendezvous. Robots may push the payload
   directly or push through a bounded companion chain, while object contact is
   preferred. Completion evidence is correlated to the task and must represent
   the full requested fleet contribution.

The transport controller uses the clean-room GRF planner by default and retains
the previous planner as an explicit fallback. Collision avoidance remains
active during search, rendezvous, formation, following, and transport; payload
contact is treated separately so the safety layer does not incorrectly reject
the intended push.

### Web control plane

The production-oriented control path includes:

- authenticated and account-owned simulation sessions;
- FIFO scheduling onto registered workers with bounded per-session robot
  counts;
- fleet resize and task start, pause, resume, cancel, and emergency-stop
  operations;
- durable worker command acknowledgement and session heartbeats;
- correlated task progress and terminal outcomes, plus timeouts for finite
  tasks that stop making progress; and
- a worker-authenticated drain lease tied to the exact Git revision used by the
  GPU deployment workflow.

Follow-the-leader is intentionally continuous and is not failed merely because
it remains active without reaching a finite completion value. Finite formation
and transport tasks are monitored for acceptance and progress.

### Private browser viewer

The implemented primary delivery path is:

```text
private X display + gzclient
        -> FFmpeg H.264
        -> authenticated RTSP ingest
        -> MediaMTX low-latency HLS (private Compose network)
        -> authenticated backend proxy
        -> hls.js in the owning user's browser
```

Each lease receives a canonical path, a private publish token, a separate read
token, an expiry, and a private display/runtime directory. The browser sends its
read token for every playlist and media-part request. The backend validates it
and uses a different protected CDN credential toward MediaMTX; the browser
credential does not cross that private boundary. MediaMTX HLS port 8888 is not
published on the host. WHEP remains an optional secondary transport; public
ICE/TURN is no longer a prerequisite for accepting the HLS path.

The stock Burger model has no supported camera sensor, so the honest current
viewer capability is `Scene`. `RobotCamera` requests are rejected. Publishing
and HLS proxying remain disabled by default until the final two-user public
browser test succeeds.

### Deployment and host hardening

- Frontend deployment remains Cloudflare's Git integration.
- Backend deployment remains the protected backend workflow on the LAN VM.
- GPU worker deployment is a separate manually dispatched, protected workflow.
  It acquires and validates a backend drain lease automatically; an operator
  checkbox is not accepted as evidence that the worker is empty.
- The GPU release uses the exact CI-passed current `main` SHA and the exact
  content-addressed ROS image.
- VNC and websockify are maintenance services bound to loopback. They are not
  the multi-user browser viewer.
- PostgreSQL remains private, while the backend is published only on the
  configured LAN address used by Nginx Proxy Manager.

## Evidence obtained so far

The following statements refer to tests already performed, not to the pending
final public rollout:

- The working log records a visible Gazebo matrix of formations, letters,
  follow-the-leader, and collaborative transport across representative points
  of the 1–10 robot range. The repository retains the full structured evidence
  for the N=10 transport and search cases; the matrix is a historical
  observation until representative cases are repeated on the final deployed
  revision.
- The repeated ten-robot transport run reached `DONE/completed`; all ten robots
  rendezvoused and contributed through two payload-contact roots and companion
  chains, with zero recorded collision deltas.
- During that run the payload advanced 0.922 m with 0.9997 directional
  efficiency; the measured physics real-time factor was 2.9505 under load.
- A separate distant-search run recorded motion from 10/10 robots, one finder
  notification, 10/10 acknowledgements, responses from the other 9 robots, and
  zero collision deltas.
- Two local publisher pipelines have run concurrently with distinct displays,
  stream paths, runtime directories, and cleanup. This proves host-side media
  isolation, but it does **not** replace the pending public two-account browser
  test through `rs.zerav.la` and `robot.zerav.la`.
- Production maintenance ports were checked after hardening: VNC and
  websockify listen on loopback. The viewer feature gates were intentionally
  left off during the pre-release baseline.

Raw measurements, screenshots, incident analysis, and the distinction between
negative and accepted runs are recorded in the
[commissioning report](docs/informe-comisionamiento-final.md).

## Work that remains before declaring the release complete

1. Require the exact published revision to pass GitHub CI. The integrated
   concurrency/security review and complete local suite are already closed.
2. Merge and deploy the backend contract and database migration first, with
   viewer feature gates still disabled.
3. Deploy the same current `main` SHA to the GPU worker through the automatic
   drain workflow and verify service readiness and the immutable image ID.
4. Enable worker publishing and the backend HLS proxy only after their
   prerequisites are healthy.
5. Run two simultaneous, independently authenticated browser sessions with
   different fleets/tasks and prove distinct displays and streams, cross-user
   denial, lease expiry, independent stop/cleanup, visible FPS/real-time factor,
   and terminal task outcomes.
6. Remove temporary accounts and sessions, finish the Spanish report, and keep
   the rollback tags until the release has remained healthy.

Until these steps finish, “implemented” must not be read as “final production
acceptance passed”.

## Known limitations

- ROS Noetic and Gazebo Classic are compatibility dependencies and are already
  end-of-life; migration to ROS 2 and a current Gazebo release is separate work.
- HLS avoids public UDP/ICE/TURN but adds latency and sends media through the
  backend proxy.
- Browser viewing is scene-only for the current Burger model.
- Simultaneous-session capacity must be accepted on the actual production GPU;
  the 1–10 robot algorithm range is not a promise of ten concurrent sessions.
- The live matrix samples representative robot counts and shapes. It does not
  imply that every shape, path, world, and parameter combination has been
  exhaustively verified.

## Current documentation

- [Swarm control plan](docs/swarm-control-plan.md)
- [GPU worker deployment](docs/gpu-worker-deployment.md)
- [Scene viewer publisher](docs/viewer-publisher.md)
- [Spanish final commissioning report](docs/informe-comisionamiento-final.md)
