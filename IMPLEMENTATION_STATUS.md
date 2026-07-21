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
| React frontend | User-owned session, fleet and task controls; live status; private HLS scene player | PR #101 merge `538ba066` published by Cloudflare; current accessibility/acceptance markers are local candidate changes |
| .NET backend | Authentication, ownership checks, session queue, worker commands, task outcomes, viewer leases and deployment drain leases | PR #101 merge `538ba066` deployed and healthy; internal task-serialization retry is in the local candidate |
| .NET GPU worker | Isolated Docker session lifecycle, ROS command bridge, heartbeats, immutable image selection and viewer-publisher supervision | PR #101 merge `538ba066` active; AF_NETLINK sandbox correction is in the local candidate |
| ROS Noetic and Gazebo | Dynamic TurtleBot3 Burger fleet, safety control, task orchestration and three scalable behaviours | Implemented and exercised in visible simulation |
| MediaMTX and viewer helper | Private X display and `gzclient` per lease, H.264/RTSP publishing and internal low-latency HLS origin | Implemented; gates active only inside the supervised commissioning window |

The layers use one correlated lifecycle rather than separate ad-hoc controls:
the frontend calls the authenticated backend, the backend issues durable worker
commands, the worker translates those commands into the private ROS namespace,
and ROS reports fleet and task state through the worker hub. A browser is not
given Docker, ROS, Gazebo, VNC, or MediaMTX administrative access.

PR #101 previously deployed merge
`538ba0660c3a070cd600d1b224dc89ec1dd1dbe7`. Its PR and `main` CI, Cloudflare
publication, backend workflow and one GPU workflow passed, so the three layers
were aligned at that checkpoint. The latest read-only backend preflight instead
reported abbreviated revision `1182dec` and `Healthy`; it did not inspect every
layer and is not evidence that the local candidate was deployed. Public
acceptance after the PR #101 checkpoint
found that the worker's systemd sandbox omitted AF_NETLINK and that two valid
task starts could surface a PostgreSQL serialization conflict. Both fixes are
implemented and locally tested. The final local review also tightened account
and session serialization, held-input release and browser-FPS evidence. None of
those candidate changes is described as deployed: the release remains open
until one exact revision passes the new CI/deployment/acceptance cycle.

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

The live probes use correlated task and delete identifiers. They stop the exact
task before mutating shared Gazebo state, reject partial fleet deletion, verify
an empty roster and no residual TurtleBot models or velocity monitors, and
propagate cleanup failures into the final nonzero result. This is acceptance
instrumentation: the same conditions still have to be observed on the deployed
GPU revision.

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

Account and session mutations now share an explicit lock order. A global
administrator-set advisory lock protects the last-enabled-administrator rule;
email-changing flows then take a global email-set lock, followed by account locks
in account-ID order. These resources order role, email, password, disablement
and session mutations. The authenticated account is re-read only after the
shared lock is held and its enabled flag, role and `account_version` claim must
still agree.
On PostgreSQL, that read uses `SELECT ... FOR SHARE`: if a `SERIALIZABLE`
transaction fixed an old snapshot while waiting, PostgreSQL raises `40001` and
the route performs a real new attempt instead of accepting stale credentials.
The reproduced stale-snapshot case now returns HTTP 401 and leaves no session.
Administrative requests that mutate a different account do not rely only on
the authorization decision made before the handler. When the operation affects
administrator membership, it first takes the global administrator lock; it
then acquires the actor account shared and the target account exclusive in
account-ID order. The actor is validated again while those locks are held. A
request whose Admin actor was revoked while waiting now returns HTTP 401 and
leaves the target account unchanged.

Terminal-session reconciliation is part of the worker heartbeat transaction.
It locks terminal sessions in stable order before loading their commands, and
commits before SignalR publication. Account cleanup and the heartbeat therefore
share one causal command sequence rather than competing to insert the same
`StopSession`. The public OpenAPI/Swagger contract also declares the possible
HTTP 409 returned by a session `DELETE` after its bounded serialization retries.

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
  administrador existentes. La acción «Crear usuario» queda en el encabezado
  y el contenido ya no reserva una barra lateral vacía.
- **Criterio visual común:** Control, Historial, Plantillas, Robots, Grupos y
  Usuarios comparten tipografía, capitalización, encabezados, radios, bordes y
  foco visible. Se retiraron los banners de gradiente y las animaciones
  ornamentales; el inventario de Robots usa una tabla y las tarjetas quedan
  reservadas para membresías de Grupos, donde sí expresan una relación.

El corte integral histórico aprobó backend 213/213, worker 121/121, ROS 362/362
y frontend 132/132. El candidato posterior a la aceptación aprobó backend
216/216 y ROS 391/391; los contratos ejecutables aprobaron API 8/8, visor 10/10 y
secciones 14/14. Un recorrido público con Chrome normal aprobó Historial para
User, el menú reducido, 403 en dos endpoints Admin y la redirección de las cuatro
rutas restringidas, con limpieza completa. Sobre el árbol local completo
aprobaron worker 121/121, frontend 141/141 en 28 suites, el lint de todos los
archivos frontend modificados y el build de producción. Falta publicar el único
PR final y repetir la aceptación sobre su SHA desplegado.

El cierre local posterior conserva esas cifras como antecedentes y registra el
estado vigente por separado. La suite completa de backend sin conexión
PostgreSQL aprobó 250 pruebas y omitió por diseño las 8 opt-in (258 descubiertas);
el filtro no-PostgreSQL confirmó 250/250 y las focales de cuentas 23/23. Al
habilitar PostgreSQL 17.10 aprobaron 8/8; el build backend terminó con cero
errores y cuatro advertencias heredadas. Worker aprobó 124/124, ROS 427/427 y
frontend 149/149 en 28 suites, además del lint de 75 archivos y el build de
producción. Los siete arneses offline aprobaron 193/193 contratos, desglosados
como 16+38+13+44+37+30+15. También aprobaron `py_compile` sobre 14 módulos, la
sintaxis Bash y `git diff --check`. Son resultados locales del candidato, no
evidencia de CI, despliegue ni postdespliegue.

El delta I-088 endurece tres límites de aceptación. El smoke iniciado desde la
interfaz ya no considera suficiente que la API declare cuatro buscadores: cada
robot debe recorrer al menos 0,015 m durante `SEARCH`. En ROS,
`ObstacleAvoidance` detecta el flanco filtrado en el mismo ciclo de seguridad y
`CollaborativeTransport` sella UUID y secuencia de fuente, tarea, fase, secuencia
de control y tiempos. El stream v2, acotado a 128 eventos, forma parte del mismo
`/transport/status`, incluido el terminal. El orquestador lo valida y copia a
`/swarm/status.collision_events` de forma idempotente; no produce esa causalidad
y el `Bool` legado no incrementa el contador durante transporte. Reinicios,
huecos, regresiones, metadatos inválidos o falta de capacidad fallan cerrado.
Como la máscara ya excluye carga y cadena declarada, todo contacto filtrado que
permanece es inesperado en cualquier fase; el atraque se prueba aparte con
geometría, contactos declarados y GRF. La matriz añade `transport_grf_n2`, cuyo
contrato exige dos raíces sobre la carga y cero compañeros. Ese caso todavía no
tiene una corrida física N=2.

I-089 corrigió el desfase entre los rótulos del arnés y la navegación real. Las
seis entradas canónicas son Plantillas, Historial, Control, Robots, Grupos y
Usuarios, y una regresión comprueba nombre, ruta y rol leyendo
`navigationGTSConfig.js`. I-090 trasladó al backend la validación de Create, PUT
y PATCH, la política de contraseña 8–16 y el correo canónico con exclusión
Admin→correo→cuentas, columna generada e índice único. I-091 eliminó la
divergencia entre `Trim()` y `btrim()` mediante el mismo conjunto explícito de
espacio, TAB, LF, VT, FF y CR en C# y PostgreSQL; la migración falla sin cambios
ante datos históricos no soportados o duplicados. El preflight productivo de
solo lectura observó `1182dec`, estado `Healthy`, nueve cuentas y cero anomalías
agregadas, sin registrar PII. No demuestra despliegue del candidato.

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
and HLS proxying are protected gates. They are active in the supervised
commissioning window. With a temporary AF_NETLINK diagnostic override, two
sessions produced independent HLS playlists and enforced token isolation. That
observation proves I-064's cause but is not final acceptance; the versioned unit
must be deployed and the override removed first.

PR #100 also added an explicit, owner-scoped viewer close.
Revocation is idempotent and fenced against a replaced lease. The worker keeps
a short tombstone so a delayed start cannot revive a viewer that was already
closed. Leaving native fullscreen releases every held input. `Escape` is kept
local to the browser, including auto-repeat and the case where its prior key-up
was lost, so it cannot become a Gazebo key after reopening fullscreen. The
interaction button only arms the channel; it does not synthesize a Gazebo
input, and disabling it emits one global release. The
loaded and matrix harnesses also apply their shared
`MINIMUM_BROWSER_VIDEO_FPS` threshold (currently 27.0) before accepting a
capture taken during `PUSH`. Automated coverage exists; the final two-browser
run must still close and reopen one viewer while its ROS task continues.

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
- The local loaded N=4 v11 gate is accepted. The fixed-command trials moved the
  0.75 kg payload 0.0070/0.0340/1.0424 m for one/two/four robots, an externally
  recalculated 148.9143x gain, at RTF 2.9969/2.9962/2.9975. GRF reached
  `DONE`: `tb3_1` notified three peers, all four searched, rendezvoused and
  pushed through two payload roots and two companions, and 1,598/1,598 batches
  were useful. Goal progress was 0.5002 m at 0.9946 efficiency and external RTF
  2.9756. Ground-truth recorded one declared root docking contact while the
  filtered safety-contact counter stayed at zero. The concurrent RTX 3080 probe
  measured 58.816 camera FPS, 58.831
  post-render callbacks/s and RTF 2.996 in a 1618x869 viewport.
- The first outer validation of v11 exposed a harness assumption rather than a
  physical failure: fresh fleet allocations were `tb3_0`, then `tb3_1`–`tb3_2`,
  then `tb3_3`–`tb3_6`, instead of restarting at zero. The gate now accepts an
  arbitrary initial offset while requiring canonical `tb3_<n>` names, equal
  progress/connection maps, contiguous and disjoint blocks, and a monotonic
  allocation sequence. The subsequent freeze reached 38/38 loaded-harness
  contracts and includes rejection of a `PUSH` capture below the shared
  browser-video threshold.
  Full ROS passed 427/427; frontend passed 149/149 in 28 suites, lint and
  production build.
- The current local software freeze passed 250 backend tests with eight
  PostgreSQL-only cases intentionally skipped in the unconfigured run (258
  discovered). The non-PostgreSQL filter passed 250/250 and the account-focused
  group passed 23/23. Those eight opt-in cases passed 8/8 against PostgreSQL
  17.10; worker passed 124/124. The database
  model probe produced an empty migration and an unchanged snapshot after
  preserving `SimulationSession.Revision` as a concurrency token. Across the
  seven acceptance drivers, 193/193 offline contracts passed
  (16+38+13+44+37+30+15). The N=2 addition proves the exact two-root,
  zero-companion contract in the catalog; it is not a physical N=2 run. These
  are still predeployment results.
- On 2026-07-21 the frozen candidate re-passed every suite and, additionally,
  a local rehearsal of the complete CI pipeline: the deployment/rollback guard
  script, direct execution of the seven harness contracts, Release builds and
  Release test runs (backend 250/250 plus 8/8 PostgreSQL opt-in, worker
  124/124), the idempotent `dotnet ef` migrations script including
  `AddCanonicalAccountEmail`, the production Compose check, the pinned
  MediaMTX validation (H.264 publish/read, publish-token read denial,
  cross-path denial, HTTP auth events, and HLS playlist/init/part behind the
  CDN secret), the viewer-publisher host test, frontend lint/test/build on
  Node 18, the ROS unittest sweep (427) and the `swarm_ws` image build. A
  read-only probe of the public backend the same day returned `Healthy`.
  These remain local rehearsal results on the working tree: they do not
  replace the required CI run, deployment and public acceptance of one exact
  committed revision.
- The local host integration test has run two publisher pipelines concurrently
  with distinct displays, stream paths, runtime directories, and cleanup. It
  uses real Xvfb/XTest with controlled Docker and `gzclient` test doubles, so it
  proves local orchestration isolation, not two simultaneous GPU media streams.
  It does **not** replace the pending public two-account browser test through
  `rs.zerav.la` and `robot.zerav.la`.
- Production maintenance ports were checked after hardening: VNC and
  websockify listen on loopback. The viewer gates were off during the initial
  baseline and later enabled together for the supervised commissioning window.
- The management pages are present in the public Cloudflare bundle. The User
  role has exercised Historial and real backend denial for administrative
  endpoints. The Admin traversal and `StopViewer` lifecycle require the final
  candidate deployment.

Raw measurements, screenshots, incident analysis, and the distinction between
negative and accepted runs are recorded in the
[commissioning report](docs/informe-comisionamiento-final.md).

## Work that remains before declaring the release complete

1. Publish the single reviewed I-064–I-091 candidate in one PR and use one
   normal CI run. After merge, let Cloudflare and the
   backend deploy the exact SHA, then dispatch the GPU worker exactly once.
2. Remove the diagnostic systemd override and verify the installed versioned
   unit, worker health, immutable image identity and zero residual sessions.
3. Run two simultaneous, independently authenticated browser sessions with
   different fleets/tasks and prove distinct displays and streams, cross-user
   denial, lease expiry, independent stop/cleanup, visible FPS/real-time factor,
   and terminal task outcomes.
4. Run the Admin section traversal, then restore and recheck the User role.
5. Repeat the representative formation, leader-following, N=1/2/3/4/10 transport
   and loaded-payload+GRF gates on the deployed revision. This includes the
   first physical N=2 GRF run. Pair the loaded run
   with NVIDIA ≥45 FPS and RTF ≥2.90 preflight evidence.
6. Capture the new History, Templates, Robots, Groups, Runtime monitor, Viewer
   close and Users pages only after that public rollout; sanitize and hash the
   real images.
7. Remove temporary roles, tokens, accounts, sessions and profiles, finish the
   Spanish report, and keep the rollback tags until the release has remained
   healthy.

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
- The deployed PR #101 checks and the local I-064–I-091 regressions are not a
  substitute for the pending candidate deployment and public acceptance.

## Current documentation

- [Swarm control plan](docs/swarm-control-plan.md)
- [GPU worker deployment](docs/gpu-worker-deployment.md)
- [Scene viewer publisher](docs/viewer-publisher.md)
- [Production acceptance harnesses](scripts/acceptance/README.md)
- [Spanish final commissioning report](docs/informe-comisionamiento-final.md)
