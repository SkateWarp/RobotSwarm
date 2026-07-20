# RobotSwarm implementation status

**Snapshot date:** 2026-07-20
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
| React frontend | User-owned session, fleet and task controls; live status; private HLS scene player | PR #100 merge `baba2c1` integrated and published by Cloudflare |
| .NET backend | Authentication, ownership checks, session queue, worker commands, task outcomes, viewer leases and deployment drain leases | PR #100 merge `baba2c1` deployed and publicly healthy |
| .NET GPU worker | Isolated Docker session lifecycle, ROS command bridge, heartbeats, immutable image selection and viewer-publisher supervision | PR #100 code integrated; deployment rolled back safely to `62a136a` after the headless `xprop` preflight defect in I-063 |
| ROS Noetic and Gazebo | Dynamic TurtleBot3 Burger fleet, safety control, task orchestration and three scalable behaviours | Implemented and exercised in visible simulation |
| MediaMTX and viewer helper | Private X display and `gzclient` per lease, H.264/RTSP publishing and internal low-latency HLS origin | Implemented; gates active only inside the supervised commissioning window |

The layers use one correlated lifecycle rather than separate ad-hoc controls:
the frontend calls the authenticated backend, the backend issues durable worker
commands, the worker translates those commands into the private ROS namespace,
and ROS reports fleet and task state through the worker hub. A browser is not
given Docker, ROS, Gazebo, VNC, or MediaMTX administrative access.

The current control-plane revision is merge
`baba2c1fb4bc73dcd96254a7ab63a16175e6bce9` from PR #100. Its PR and `main`
CI runs passed, Cloudflare published the frontend, and the backend deployment
finished healthy. The GPU workflow passed its tests, ROS image build and NVIDIA
smoke, but its final host probe depended incorrectly on an ambient `DISPLAY`
for `xprop -version`. Rollback restored worker release `62a136a` and scheduling
resumed. I-063 records the small headless preflight hotfix; public-browser
acceptance remains pending until that hotfix is integrated and the worker uses
the same exact revision as the backend.

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

### Administración web integrada por el PR #100

El cambio integrado completa las secciones que todavía dependían de rutas
heredadas o no mostraban el estado real del plano de control:

- **Historial de tareas:** la pantalla consulta `TaskRun`, paginado y filtrado
  por la cuenta propietaria de la sesión. `TaskLog` se conserva únicamente para
  compatibilidad y diagnóstico administrativo del flujo antiguo; ya no
  alimenta el historial de usuario.
- **Plantillas de tareas:** el catálogo administrativo expone únicamente las
  operaciones reales `GET` y `PUT`. La interfaz permite listar y editar nombre
  y tipo; no presenta creación o eliminación que el backend no implemente.
- **Robots:** el registro persistente dispone de búsqueda, alta, edición y
  desactivación. La identidad autenticada fija el propietario al crear, un
  usuario solo modifica sus robots y un administrador puede supervisar el
  inventario activo completo. Los robots públicos continúan siendo visibles,
  pero no editables por terceros.
- **Grupos de robots:** la sección administrativa gestiona grupos y membresía,
  exige confirmación para transferir un robot y deja los miembros disponibles
  al eliminar un grupo. Se retiró la antigua ruta de “asignar tarea”, porque
  solo generaba `TaskLog` y cambiaba estados; nunca ejecutaba ROS. Las tareas
  reales siguen iniciándose desde Control de simulación.
- **Operación de sesión:** el frontend muestra el roster runtime informado por
  el worker, sus roles, namespaces, estados y última actualización. Reintenta
  también el primer fallo de SignalR sin requerir recargar la página y pide
  confirmación antes de detener y liberar una sesión completa.
- **Visor:** `Cerrar visor` revoca el lease propio, libera entradas y emite el
  comando durable `StopViewer`, que detiene solo ese publicador sin apagar ROS,
  Gazebo, el contenedor o la red de la sesión.
- **Usuarios:** la navegación GTS y el encabezado administrativo utilizan el
  nombre visible «Usuarios», conservando el CRUD y la autorización de
  administrador existentes.

Una ejecución focal local, sin consumir minutos de GitHub Actions, aprobó
31/31 pruebas de backend, 19/19 del worker y 39/39 de frontend para este delta.
El lint focal y la compilación de producción de las nuevas pantallas
administrativas también aprobaron. Después de ese corte, las suites locales integrales de
backend (213/213), worker (121/121) y ROS (362/362), además del auxiliar del
publicador, aprobaron sobre el árbol actual. Frontend aprobó 132/132 pruebas en
22 suites, el lint de todos los archivos modificados y una compilación de
producción. PR #100 y su repetición sobre `main` aprobaron después las mismas
capas. Estos resultados automatizados todavía no sustituyen la aceptación
visible posterior al despliegue coordinado del worker.

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
and HLS proxying are protected gates. They are active on the last compatible
backend release for the supervised commissioning window, but that state does
not count as production acceptance until the worker runs the same post-hotfix
revision as the control plane.

PR #100 also adds an explicit, owner-scoped viewer close.
Revocation is idempotent and fenced against a replaced lease. The worker keeps
a short tombstone so a delayed start cannot revive a viewer that was already
closed. This behavior has automated regression coverage; operational proof is
pending because the GPU worker rolled back before activation.

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
- The most recent isolated N=10 transport repetition recorded 977 search
  samples with all ten robots moving, one finder plus nine notices and
  acknowledgements, useful pushing by all ten robots, 59.14% full-roster
  contribution, 0.5044 m progress, RTF 2.9672, 49.960 visible FPS on the RTX
  3080, and no unexpected contact.
- The local host integration test has run two publisher pipelines concurrently
  with distinct displays, stream paths, runtime directories, and cleanup. It
  uses real Xvfb/XTest with controlled Docker and `gzclient` test doubles, so it
  proves local orchestration isolation, not two simultaneous GPU media streams.
  It does **not** replace the pending public two-account browser test through
  `rs.zerav.la` and `robot.zerav.la`.
- Production maintenance ports were checked after hardening: VNC and
  websockify listen on loopback. The viewer gates were off during the initial
  baseline and later enabled together for the supervised commissioning window.
- The PR #100 management pages are present in the public Cloudflare bundle and
  its backend is deployed. No claim is made yet that `StopViewer` or the runtime
  robot monitor has been exercised through the public browser with the final
  worker.

Raw measurements, screenshots, incident analysis, and the distinction between
negative and accepted runs are recorded in the
[commissioning report](docs/informe-comisionamiento-final.md).

## Work that remains before declaring the release complete

1. Publish the focused I-063 hotfix after its headless regression, real
   `/usr/bin/xprop` dependency probe, publisher suite, deployment/rollback suite
   and independent review have passed locally.
2. Let CI and the serialized backend workflow deploy the resulting exact merge
   SHA, then dispatch the GPU worker once for that same revision. Do not rerun
   the already diagnosed failed workflow from `baba2c1`.
3. Verify worker health, `StopViewer` capability negotiation, immutable image
   identity, NVIDIA rendering and zero residual sessions after activation.
4. Run two simultaneous, independently authenticated browser sessions with
   different fleets/tasks and prove distinct displays and streams, cross-user
   denial, lease expiry, independent stop/cleanup, visible FPS/real-time factor,
   and terminal task outcomes.
5. Repeat the representative formation, leader-following, normal transport and
   loaded-payload gates on the deployed revision.
6. Capture the new History, Templates, Robots, Groups, Runtime monitor, Viewer
   close and Users pages only after that public rollout; sanitize and hash the
   real images.
7. Remove temporary accounts and sessions, finish the Spanish report, and keep
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
- Persistent robots and administrator groups are inventory metadata. They do
  not select the exact runtime identities inside a simulation session; the
  worker roster is the authoritative source for live ROS/Gazebo instances.
- The PR #100 checks and local hotfix regressions are not a substitute for the
  pending post-hotfix GPU deployment and public acceptance.

## Current documentation

- [Swarm control plan](docs/swarm-control-plan.md)
- [GPU worker deployment](docs/gpu-worker-deployment.md)
- [Scene viewer publisher](docs/viewer-publisher.md)
- [Production acceptance harnesses](scripts/acceptance/README.md)
- [Spanish final commissioning report](docs/informe-comisionamiento-final.md)
