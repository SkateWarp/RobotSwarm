# Swarm control plan

## Document status

This is the current target architecture and release plan for the commissioning
branch. It supersedes the earlier design that treated public WHEP/ICE/TURN and
a manually confirmed empty GPU worker as prerequisites. PR #100 integrated the
management and viewer-lifecycle delta. PR #101 then closed the `xprop` preflight
failure and deployed merge `538ba066` to Cloudflare, backend and the GPU worker.
The first two-user acceptance exposed the AF_NETLINK sandbox defect I-064 and a
serializable task-start race I-065. Those fixes, the visible section harness,
the stronger SEARCH/load gates, the visual consistency pass I-068 and the
fail-closed acceptance cleanup I-069/I-070 form the current candidate. It is not
production-accepted until one exact new revision is deployed to all three
layers and passes the full browser and ROS matrix.

The earlier GitHub certificate incident I-053 and the `xprop` incident I-063
remain closed. PR #101 proved three-layer alignment at `538ba066` at that
checkpoint. A later read-only backend preflight reported `1182dec` and
`Healthy`, but did not inspect all three layers; current cross-layer alignment
is therefore not asserted. The not-yet-published I-064–I-091 candidate remains
outside production.

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
7. Closing only the viewer revokes its lease, releases held input and stops the
   exact publisher while keeping ROS, Gazebo, the container and private network
   alive. Stopping the complete session is a separate, confirmed action that
   removes all those resources without stopping another user's session.

Worker commands are durable and acknowledged. A system-generated task cancel
must finish or remain an explicit barrier before the session can accept another
task; a stale worker report or command envelope must not resurrect a task that
the control plane already marked terminal. These race conditions are part of
the final integrated regression review, not assumptions delegated to the user.

Las mutaciones autenticadas siguen un orden común de exclusión. Los cambios que
pueden afectar al último administrador habilitado toman primero un advisory
lock global; después, cada cambio de rol, contraseña, estado o sesión usa el
lock exclusivo de la cuenta. Las rutas de sesión toman su contraparte compartida
y solo entonces vuelven a leer `Enabled`, `Role` y `Updated`, que debe coincidir
con el claim JWT `account_version`. En PostgreSQL, `SELECT ... FOR SHARE` evita
que una transacción `SERIALIZABLE` utilice el snapshot que fijó antes de esperar:
el snapshot obsoleto provoca `40001`, se inicia un intento realmente nuevo y el
principal anterior termina en 401 sin insertar una sesión. El modelo y su
snapshot conservan además `SimulationSession.Revision` como token de
concurrencia; el probe de migración no produjo operaciones y dejó el snapshot
estable.

Una mutación administrativa sobre otra cuenta vuelve a validar también al
actor, no solo al objetivo. Cuando aplica la regla del último administrador,
toma primero el lock global; después adquiere la cuenta del actor en modo
compartido y la del objetivo en modo exclusivo, siempre por identificador para
evitar invertir el orden entre dos peticiones. La validación del actor ocurre
después de esos locks. Si su rol Admin, estado o versión fueron revocados
mientras esperaba, la ruta devuelve 401 y no modifica la cuenta objetivo.

El `heartbeat` encierra la reconciliación terminal, el guardado y el commit en la
misma transacción. Las sesiones terminales se bloquean en orden antes de cargar
sus comandos; solo después del commit se publica SignalR. Así, el cleanup de una
cuenta y la reconciliación del worker comparten una única secuencia
`StopSession`. Si un `DELETE` de sesión agota sus reintentos acotados, devuelve
409; ese resultado también queda declarado en Swagger/OpenAPI.

## Secciones administrativas y fuente de verdad

La auditoría posterior al PR #99 encontró que varias pantallas existían en la
navegación, pero todavía llamaban rutas heredadas o presentaban acciones que no
tenían un efecto equivalente en ROS. El cambio integrado por PR #100 las alinea de la
siguiente manera:

| Sección | Fuente y autorización | Función implementada | Límite explícito |
| --- | --- | --- | --- |
| Historial de tareas | `TaskRun` de sesiones pertenecientes al usuario autenticado | Lista paginada, filtros por tipo/estado/resultado y detalle de parámetros, resultado, motivo y tiempos | `TaskLog` es un registro global heredado; queda fuera de la navegación y restringido al administrador por compatibilidad/diagnóstico, sin mostrarse como ejecución ROS actual |
| Plantillas de tareas | Catálogo `TaskTemplate`, rol `Admin` | `GET` para listar y `PUT` para editar nombre y tipo con validación | No se ofrecen crear ni eliminar, porque esas operaciones no existen en el contrato real |
| Robots | Registro persistente `/Robots`; propietario autenticado y excepción administrativa | Buscar, registrar, editar y desactivar; un administrador ve el inventario activo completo | Un robot público puede consultarse, pero solo su propietario o un administrador puede modificarlo; este registro no es el roster runtime de Gazebo |
| Grupos de robots | Registro global de administración, rol `Admin` | Crear, editar y eliminar grupos; agregar, quitar o transferir membresía con confirmación | Se retiró `POST /RobotGroups/{id}/tasks`: generaba `TaskLog` y estados, pero no enviaba órdenes a ROS |
| Robots de esta sesión | Roster informado por el worker para la sesión propiedad del usuario | Estado, rol, namespace, ordinal, última actualización y resumen operativo | No fabrica posición ni velocidad: esos campos no forman parte del contrato actual |
| Usuarios | CRUD de cuentas existente, rol `Admin` | Navegación y encabezado coherentes bajo el nombre «Usuarios» | No cambia el modelo de roles ni crea permisos nuevos |

Las seis secciones utilizan capitalización normal, tipografía Inter,
encabezados sobrios, foco visible y superficies delineadas en lugar de
gradientes y sombras decorativas. Robots se presenta como tabla; Grupos conserva
tarjetas únicamente para representar membresías. «Crear usuario» queda junto al
título y el contenido no reserva una barra lateral vacía.

Las tareas ROS se inician exclusivamente en Control de simulación. Un grupo
administrativo puede ayudar a organizar el inventario, pero no selecciona por
sí mismo las instancias `tb3_*` de una sesión ni sustituye el roster que publica
el worker.

El mismo espacio de simulación conserva sondeo periódico si SignalR no está
disponible, muestra esa degradación y reintenta también un fallo del primer
`start()` con espera exponencial acotada. El usuario puede solicitar una
reconexión inmediata. Detener la sesión completa abre una confirmación que
explica que se cancelarán tareas y se liberarán visor, Gazebo, ROS, contenedor y
red privada.

La validación histórica del delta aprobó backend 213/213, worker 121/121, ROS
362/362 y frontend 132/132. El candidato actual ya aprobó backend 216/216 y ROS
391/391; sus contratos de aceptación aprobaron API 8/8, visor 10/10 y secciones
14/14. Además, un Chrome normal comprobó con rol User el Historial, el menú
reducido, los 403 reales del backend y las cuatro redirecciones administrativas,
con perfil y capturas saneadas. El árbol completo aprobó worker 121/121,
frontend 141/141 en 28 suites, lint del delta y build de producción. El siguiente
paso es el único CI del candidato y la aceptación del SHA desplegado.

En el freeze local vigente la suite completa de backend sin PostgreSQL configurado
aprobó 250 pruebas y omitió las 8 opt-in (258 descubiertas); el filtro ordinario
confirmó 250/250 y las focales de cuentas 23/23. Las ocho aprobaron 8/8 contra
PostgreSQL 17.10. También aprobaron worker 124/124, ROS 427/427 y frontend 149/149
en 28 suites. Los siete arneses offline sumaron 193/193 contratos:
16+38+13+44+37+30+15. Quedaron en verde la compilación de 14 módulos Python, la
sintaxis Bash, el lint de 75 archivos frontend, el build de producción y
`git diff --check`. Estas cifras no sustituyen el único CI ni los recorridos
sobre el SHA desplegado.

I-088 impide que una fase `SEARCH` declarada sustituya el movimiento observado:
el smoke web integra la trayectoria de cada robot y exige al menos 0,015 m por
unidad. La causalidad del contacto tampoco depende ya del `task_lock` ni de dos
callbacks del orquestador. `ObstacleAvoidance` detecta el flanco filtrado dentro
de su evaluación y `CollaborativeTransport` sella UUID y secuencia de fuente,
tarea, fase, secuencia de control y tiempos. El stream v2 viaja, hasta 128
eventos, dentro del mismo `/transport/status`, incluso en el terminal. El
orquestador lo valida y copia de manera idempotente; el `Bool` de compatibilidad
no incrementa el transporte. Reinicios, huecos, regresiones, metadatos inválidos
o falta de capacidad fallan cerrado.

La carga y los vecinos declarados de la cadena ya están excluidos de ese contador
por la máscara de seguridad. Todo contacto filtrado restante es inesperado en
cualquier fase. El atraque se acredita aparte mediante geometría, muestras de
contacto y GRF. La matriz incluye además N=2 con dos raíces sobre la carga y cero
compañeros, pero este cierre solo valida su contrato: no existe todavía una
corrida física N=2.

I-089 alinea el arnés visible con Plantillas, Historial, Control, Robots, Grupos y
Usuarios y hace que una regresión lea la configuración React real. I-090 traslada
la política de cuentas al backend para Create/PUT/PATCH y refuerza el correo
canónico con exclusión ordenada, columna generada e índice único. I-091 fija la
misma frontera ASCII de seis caracteres en C# y PostgreSQL, añade un `CHECK`
fail-safe y cubre la equivalencia contra PostgreSQL 17.10. Estas correcciones son
locales y aún necesitan el despliegue y los recorridos de la sección final.

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

The current, not-yet-deployed candidate has one newer visible result. The N=1
transport traversed SEARCH, APPROACH, PUSH, and DONE; the practice crate advanced
0.5005 m at RTF 2.9964 with one useful pusher, 100% useful contribution, and no
collisions. The immediately preceding exact-scene GUI probe measured 61.888
camera FPS and 62.498 post-render callbacks per second on the RTX 3080. This does
not stand in for a loaded payload or any post-deployment case.

The following visible N=3 candidate run then covered the multi-robot branch.
`tb3_0` notified both teammates, all three completed rendezvous and physical
engagement, and the complete roster contributed usefully in roughly 99% of its
applicable samples. The practice crate advanced approximately 0.5005 m at
0.9984 directional efficiency, RTF remained about 2.996, and no collisions or
unexpected contacts were recorded. Its exact-scene GUI probe measured 57.907
camera FPS and 58.887 post-render callbacks per second. Its wall-time budgets
are 115 seconds for SEARCH, 125 for APPROACH, and 45 for PUSH under a 290-second
hard cap; they do not relax the physical completion gates.

El candidato cargado N=4 aportó después una evidencia local más exigente. El
primer tope de 180 s interrumpió un empuje todavía útil a los 0,2335 m, con 752
muestras y 4/4 robots contribuyendo; su cronología dio lugar a presupuestos de
60/100/190 s para `SEARCH`/`APPROACH`/`PUSH`, bajo un tope de 355 s. V10 llegó
físicamente a `DONE`, pero se rechazó porque su medición de capacidad incluía
aproximadamente 0,52 s de parada. Los extremos temporales se capturan ahora
antes de esos comandos y una regresión preserva esa separación.

La repetición v11 aprobó la compuerta local. Los ensayos de uno, dos y cuatro
robots movieron la carga 0,0070/0,0340/1,0424 m, con ganancia exterior
148,9143× y RTF 2,9969/2,9962/2,9975. En GRF, `tb3_1` notificó a los tres
compañeros; 4/4 buscaron, completaron el rendezvous y empujaron mediante dos
raíces y dos compañeros. La carga avanzó 0,5002 m, las 1598 muestras fueron
útiles para la flota completa, la eficiencia fue 0,9946 y el RTF exterior
2,9756. Ground-truth registró un atraque declarado de raíz y el contador filtrado
no registró contactos de seguridad. Durante
`PUSH`, la sonda visible midió 58,816/58,831 FPS y RTF 2,996 en la RTX 3080.

La primera validación exterior de v11 supuso erróneamente que cada flota
reiniciaría sus ordinales en cero. El contrato corregido admite el offset fresco
del gestor, pero exige namespaces canónicos, bloques contiguos y distintos,
mapas iguales y asignación monotónica. El arnés vigente aprobó 38/38 e incluye
una regresión donde la captura durante `PUSH` se rechaza si cualquiera de sus
dos lecturas HLS queda por debajo de `MATRIX.MINIMUM_BROWSER_VIDEO_FPS`,
actualmente 27,0.
La evidencia real anterior y el contrato endurecido aprobaron en sus alcances
respectivos. El supervisor conserva además el eco dinámico de progreso sin
rebajar el mínimo físico de 0,50 m. N=4 cargado queda aceptado localmente; su
repetición postdeploy continúa en la secuencia de liberación siguiente.

Las muestras del mismo task siguen el orden no decreciente `SEARCH` →
`APPROACH` → `PUSH`; un retorno a una fase anterior invalida la corrida aunque
masa, roster y progreso parezcan correctos.

La contabilidad de contactos conserva cada episodio desde su flanco ascendente.
`ObstacleAvoidance` lo detecta en el ciclo de seguridad y transporte publica su
contexto causal v2 en el mismo status, sin inferir la fase desde una muestra
posterior. El payload y los vecinos declarados de la cadena ya se excluyen por
máscara; cualquier incremento que permanezca en el contador filtrado es un
contacto inesperado en todas las fases. El atraque se demuestra aparte por
contactos declarados, geometría y GRF. Si el stream no explica por completo el
incremento agregado, el gate falla cerrado.

El catálogo de matriz añade `transport_grf_n2`. Su contrato exige el roster
exacto durante `SEARCH` → `APPROACH` → `PUSH` → `DONE`, aviso y rendezvous de
ambos robots, dos raíces de carga, cero compañeros y dos empujadores útiles. Es
una regresión instrumental; no amplía la lista histórica de corridas visibles
ni constituye todavía evidencia física N=2.

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

PR #100 adds `DELETE` for the exact owner/session/lease and a
durable `StopViewer` command. Revocation is committed before notifying the
worker, drains the matching controller grant, and is idempotent under repeated
or concurrent close requests. The worker stops only a publisher whose active
lease matches the command. A bounded tombstone rejects a delayed start for an
already revoked lease, while closing an older lease cannot stop a newer
replacement. The frontend sends `releaseAll` before requesting close and then
removes the private stream and interaction state.

El cambio de fullscreen también es una frontera de control. Cuando el navegador
sale por sí mismo, se libera una sola vez toda entrada retenida. `Escape` no se
envía a Gazebo al abandonar fullscreen; sus repeticiones automáticas se consumen
mediante `event.repeat`, y una nueva entrada en fullscreen acepta otro Escape
aunque el `keyup` anterior se haya perdido. La regresión correspondiente forma
parte de las 149 pruebas frontend vigentes.

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
logic rather than two GPU streams. The production gates are currently active
only for the supervised commissioning window. They do not become a general-use
approval until two independently authenticated public browser sessions prove
playback, cross-user denial, lease expiry, and stop isolation; they must be
disabled together if that acceptance is abandoned or rejected.

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

1. Commit the reviewed and locally validated I-064–I-091 candidate once, open
   one PR and use one normal CI run;
   do not rerun successful jobs merely to collect duplicate evidence.
2. After merge, let Cloudflare and backend deploy the exact merge SHA. Remove
   the temporary AF_NETLINK override before the GPU switch, dispatch the GPU
   workflow exactly once and verify that the installed unit itself contains
   AF_NETLINK and the new preflight.
3. Use the versioned [production acceptance harnesses](../scripts/acceptance/README.md)
   with two visible Chrome profiles and separate accounts. Require N=3/N=7,
   different streams, HLS isolation, task overlap, fullscreen/input, independent
   `Cerrar visor`, peer survival and complete cleanup.
4. Repeat the User section run, elevate one controlled account to Admin only for
   the Admin run, revoke refresh tokens at both role transitions, visit
   Historial, Plantillas, Robots, Grupos and Usuarios, delete only the owned
   temporary group, then restore and recheck User denial.
5. Run every final formation, follow-leader and N=1/2/3/4/10 transport scenario
   in fresh sessions. The N=2 case is its first physical GRF execution. SEARCH
   must show sustained motion by the whole assigned roster, followed by one
   notice, rendezvous and complete useful push.
6. Pair the loaded-payload+GRF probe with the visible NVIDIA preflight on the
   same Gazebo master. Require at least 45 render FPS and RTF 2.90, then restore
   the practice payload and verify no active task, robot or process remains.
7. Remove the diagnostic systemd file, temporary roles/tokens, sessions,
   profiles and artifacts that contain operational identifiers. Add only
   sanitized screenshots and checksums to the Spanish commissioning report.

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
