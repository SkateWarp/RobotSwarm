# Plan de control del enjambre

## Estado del documento

Este documento conserva la arquitectura objetivo y la evolución del plan de
liberación. Las afirmaciones intermedias sobre I-064–I-135 son cortes
históricos, no el estado vigente. La PR #106 integró ese conjunto como
`1448a31bbbbfd77588bada109947098cc95d9dda`; CI, Cloudflare, backend y el
despliegue GPU exacto aprobaron.

Sobre `1448a31` aprobaron API multiusuario N=3/N=7, dos visores privados en
Chrome visible, interacción, pantalla completa, responsive a 360/768/1366/1920
px, seguimiento N=3/N=6/N=10 y transporte N=2/N=3/N=4/N=10. El marcador
fantasma magenta apareció en producción y la interfaz envía una tolerancia de
llegada de 0,25 m sin cambiar la física normal de 0,25 kg y fricción 0,05. La
carga de 0,75 kg y fricción 0,25 continúa como perfil de aceptación separado.

La prueba postdeploy también encontró trabajo real pendiente. Las seis
formaciones terminaron antes del gate activo y transporte N=1 perdió su informe
gráfico después del TTL, aunque ROS, movimiento y HLS habían aprobado. I-136–
I-138 sacan el solver del callback de odometría, alinean la envolvente de
evasión, correlacionan posición/yaw al commit y permiten hasta dos replans en
cero solo cuando la escena permanece estable y sin contacto. I-139 preserva el
informe antes de esperar ROS, sondea la transición de cierre y renueva un visor
únicamente después de demostrar que el lease anterior fue desmontado. Conserva
la propiedad del lease nuevo aunque falle una comprobación intermedia y no puede
declarar limpieza con un binding desconocido. I-140 obliga a cada ruta de
parada, pausa, emergencia, replan y shutdown a comprobar si todos los publishers
locales aceptaron `Twist=0`; un fallo queda correlacionado en `FAILED` y no se
presenta como parada física confirmada. I-141 publica el estado estacionario
`forming` antes de una planificación grande, concede una gracia acotada y
proporcional únicamente mientras no existen asignaciones, y tolera el
asentamiento normal de Gazebo sin retirar la revalidación geométrica. El primer corte local aprobó 524/524
pruebas ROS, 77/77 de formación en Python actual y 3.8, 253/253 contratos y una
formación S N=10 con 10/10 asignaciones, RTF 2,9929 y cero colisiones. El freeze
posterior sustituyó esas cifras provisionales: 576/576 ROS, 253/253 contratos y
revisión independiente sin P0/P1. La imagen exacta `6f1af927…4cb5`, sin montar
fuentes, aprobó triángulo N=3 y S N=10 durante 75 s activos, con errores finales
0,0921/0,0974 m, RTF 2,9965/2,9875, 58,493/57,507 FPS NVIDIA y cero colisiones.
N=1 también volvió a aprobar contra `1448a31` usando I-139 en el arnés. El plan
suma ahora un freeze local de 624/624 pruebas y una S N=10 visible de I-141:
75,0004 s activos, error máximo 0,0952 m, RTF 2,9851 y cero colisiones. El plan
suma además I-142: el gate API y dos Chrome visibles aprobaron aislamiento,
entrada, fullscreen, concurrencia, cierre/reapertura y video cercano a 30 FPS.
Los cambios de I-142 pertenecen al instrumento y corrigen únicamente una
formación de prueba ya satisfecha, un seek hacia el borde vivo HLS y la
transición de repintado al salir de fullscreen. I-143 separa la liberación
segura de cada corredor de la convergencia estricta del slot: el freeze final
aprobó 625/625 pruebas ROS y S/N=10 con error 0,0936 m, RTF 2,9912 y cero
colisiones. I-144 fue publicada mediante la PR #108 e I-145 mediante la PR
#109. El worker GPU exacto de `2445a37` quedó activo después de que el reintento
DNS acotado aprobara el despliegue `30055847809`. La aceptación posterior
aprobó cuadrado N=5, pero encontró que el frontend abandonaba HLS a los 30 s
mientras la ruta gráfica completa consumía cerca de 28 s sin margen para
variación. I-146 eleva solo esa ventana inicial a 60 s y conserva el estado
saneado del fallo en el arnés. El plan sigue abierto hasta publicar esta última
corrección, desplegar su SHA exacto y repetir las filas afectadas y la carga.

El [estado de implementación](../IMPLEMENTATION_STATUS.md) mantiene la frontera
vigente y el [informe de comisionamiento](informe-comisionamiento-final.md)
conserva en español la cronología, los incidentes, las capturas y la evidencia
estructurada.

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

Dos URLs que seguían montando módulos heredados ya no abren pantallas falsas.
`/apps/configs/task` redirige a `/apps/GTS/task-templates`, protegida como
administración, y `/apps/dashboard/tasks` redirige a
`/apps/GTS/taskLogs`, el historial real del usuario. Con ello se retiran del
recorrido los consumidores de `/api/TaskActivity` y
`/api/TaskForm/widget`, que no forman parte de la API vigente.

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

La validación histórica de aquel delta aprobó backend 213/213, worker 121/121,
ROS 362/362 y frontend 132/132. Un corte posterior aprobó backend 216/216 y ROS
391/391; sus contratos de aceptación aprobaron API 8/8, visor 10/10 y secciones
14/14. Además, un Chrome normal comprobó con rol User el Historial, el menú
reducido, los 403 reales del backend y las cuatro redirecciones administrativas,
con perfil y capturas saneadas. El árbol completo aprobó worker 121/121,
frontend 141/141 en 28 suites, lint del delta y build de producción. Estas cifras
describen una etapa anterior a los PR #102–#104.

En el freeze local que precedió a esa integración, la suite completa de backend sin PostgreSQL configurado
aprobó 250 pruebas y omitió las 8 opt-in (258 descubiertas); el filtro ordinario
confirmó 250/250 y las focales de cuentas 23/23. Las ocho aprobaron 8/8 contra
PostgreSQL 17.10. También aprobaron worker 124/124, ROS 427/427 y frontend 149/149
en 28 suites. Los siete arneses offline sumaron 193/193 contratos:
16+38+13+44+37+30+15. Quedaron en verde la compilación de 14 módulos Python, la
sintaxis Bash, el lint de 75 archivos frontend, el build de producción y
`git diff --check`. Estas cifras no sustituyen el único CI ni los recorridos
sobre el SHA desplegado.

El freeze provisional anterior a I-117–I-125, integrado hasta `e18926a`, aprobó backend
253/253, con 8 casos PostgreSQL opt-in omitidos por diseño; frontend 164/164 en
30 suites; ROS 461/461; y los siete arneses 241/241 contratos:
17+49+13+61+44+41+16. El cierre numérico de formación aprobó 6/6 focales,
50/50 de formación/rutas/live y 1/1 de evitación. El cierre de apagado de la
carga aprobó 81/81 pruebas loaded, 170/170 ROS/mundo, 16/16 frontend focales y
53/53 backend focales. Worker aprobó 129/129, PostgreSQL 17 opt-in 8/8, el lint
focal, el build de producción frontend, el publicador del visor y las pruebas
de despliegue/rollback GPU quedaron en verde. El primer build frontend encontró
un `EACCES` en residuos ignorados de `build/`; después de corregir solo su
propiedad, aprobó sin cambiar código. Un build intermedio etiquetado
`robotswarm-ros:local-final-candidate` terminó catkin al 100 %, con ID
`sha256:db8ffda30d79e5e22e1bbfe66978faedb118bb9c43b3e25dedd161807833be14`,
4.231.139.487 bytes y fecha 2026-07-22T03:10:23Z. Ese ID se descartó como
candidato final al aparecer hallazgos P1 posteriores. Aunque el código quedó
corregido, esa imagen continúa siendo anterior y fue reemplazada después por el
artefacto seguro descrito abajo. `511e47c`
aprobó posteriormente 99/99 pruebas focales de
formación y 469/469 en su ejecución ROS aislada. `377a0e3` aprobó 206/206 de
seguimiento+lifecycle y 473/473 en la ejecución global aislada. También
aprobaron la compatibilidad con Python 3.8 y `git diff --check`. En ese corte aún
faltaban la suite combinada y la nueva imagen, completadas abajo; CI, despliegue
y pruebas visibles siguen pendientes. Son resultados locales
posteriores a `9f49e17`: no se atribuyen a esa base ni al futuro SHA antes de
que el CI selle el commit exacto.

I-126 se verificó de forma aislada como
`bd8575593aa24c2d3ac0a878641d0d235fbf6bbe` y se integró como `568979d`; aprobó
40/40 pruebas de seguimiento y 483/483 globales. I-127 se integró como
`4450c13` y aprobó 172/172 pruebas de lifecycle y 487/487 globales. Ambos cortes
aprobaron la comprobación de sintaxis con Python 3.8 y `git diff --check`. Estas
cifras no forman todavía el freeze combinado.

I-128 reprodujo que el solver de seguimiento se
bloqueó con el líder en `(0; 0)`; tras recibir una odometría válida en `(3; 3)`,
el resultado antiguo confirmó `anchor=(0; 0)` y emitió un primer giro de
aproximadamente −0,0625 rad/s.

I-129–I-134 ampliaron el mismo análisis. Formación agotó un timeout de
0,75 s al pasar el reloj de 10,0 a 11,0 y aun publicó 0,22 m/s; seguimiento N=1
repitió la ventana con 0,1 m/s. Una instantánea truncada de formación con
`name=['moving_box']` y `pose=[]` vació la escena y permitió 0,22 m/s hacia el
objetivo bloqueado. Por último, un cuaternión crudo con `Inf` produjo yaw finito
2,356 y atravesó modelo/odometría hasta generar 0,22 m/s y −1,5 rad/s.
I-133 mostró que un comando finito y dentro del límite todavía podía usar
`linear.y/z` o `angular.x/y`, ejes incompatibles con el Burger. I-134 encontró
que un nombre configurado duplicado en `ModelStates` podía limpiar una
invalidación en orden NaN→válido o entregar dos poses ambiguas.

El grupo quedó integrado como `07da8f4`. La primera iteración aprobó 95/95 de
seguimiento+formación antes del último ajuste; la versión final aprobó 6/6
regresiones focales y 497/497 globales en 109,067 s. También aprobaron
`py_compile` bajo Python 3.8.10 para tres controladores y tres archivos de prueba
y `git diff --check`. Son resultados aislados, no el freeze combinado.

La repetición exacta del árbol principal `07da8f4`, junto con los cambios aún no
confirmados de documentación y mundo, aprobó 497/497 pruebas ROS en 109,460 s,
241/241 contratos de aceptación (17+49+13+61+44+41+16) y 8/8 pruebas de física
del mundo, todos con RC=0. También aprobaron Python 3.8.10 para tres
controladores y sus tres tests, `git diff --check`, tres workflows YAML, cuatro
variantes Compose y el escaneo de secretos. La única coincidencia fue la URL
del fixture negativo en `WorkerOptionsValidatorTests.cs:44`; no es un secreto.
Este corte combinado precede al ajuste de cámara y se conserva para
trazabilidad; no es CI, despliegue ni postdeploy. La primera
imagen `robotswarm-ros:local-final-safe`, ID
`sha256:02fbd5c2302d3af0eb9e543af04bb4507292786b7155b39b54e9d294b27f4864`,
completó catkin y sirvió para validar el algoritmo y la física. Sus capturas
mostraron después que la cámara predeterminada recortaba el marcador en el borde
sur; ese ID quedó supersedido solamente por el ajuste de encuadre. La imagen
reconstruida con la misma etiqueta completó catkin al 100 %, con ID
`sha256:e17579ed83e0c37a9ff9b03817652aeb935573b307801ddc5863d29f2a92ae0d`,
tamaño 4.231.381.706 bytes y fecha
`2026-07-22T04:53:51.679721236Z`. Fue el artefacto local exacto de la etapa
I-128–I-135 y hoy es histórico. Después de ese ajuste y rebuild, la suite
completa volvió a aprobar 497/497 pruebas en 109,592 s, con `OK` y RC=0. Ese
resultado sustituyó al de 109,460 s en su etapa; el freeze vigente es
`6f1af927…4cb5` con 576/576 pruebas.

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
contacto y GRF. En aquel corte, la matriz solo validaba el contrato N=2 con dos
raíces sobre la carga y cero compañeros. I-092 registra la corrida física N=2
posterior y su límite gráfico; ya no se considera un caso nunca ejecutado.

I-089 alinea el arnés visible con Plantillas, Historial, Control, Robots, Grupos y
Usuarios y hace que una regresión lea la configuración React real. I-090 traslada
la política de cuentas al backend para Create/PUT/PATCH y refuerza el correo
canónico con exclusión ordenada, columna generada e índice único. I-091 fija la
misma frontera ASCII de seis caracteres en C# y PostgreSQL, añade un `CHECK`
fail-safe y cubre la equivalencia contra PostgreSQL 17.10. Estas correcciones se
integraron mediante los PR #102–#104; las correcciones nuevas I-093–I-105 son las
que aún necesitan despliegue y los recorridos de la sección final.

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

La recepción de odometría valida posición, yaw y los cuatro componentes del
cuaternión antes de actualizar estado. Una muestra inválida conserva juntos la
última pose y su timestamp anterior; no puede aparentar que la fuente sigue
fresca. Los `ModelStates` de obstáculos también fallan cerrado. El control reúne
primero el lote completo del líder y sus seguidores, comprueba finitud, ejes y
límites Burger y recién después lo publica. Si falla un dato, un cálculo o la
evitación, todos reciben stop y el estado `failed` conserva el identificador de
la tarea; una parada normal correlacionada permanece `stopped`.

El planificador de trayectoria trabaja fuera del lock para no bloquear el
control. I-126 mostró que, si `moving_box` pasaba de `(8; 8)` a `(1,5; 1,5)`
durante ese cálculo, el resultado antiguo podía liberar 0,00625 m/s. La
corrección integrada `568979d` compara la escena con tolerancias de
0,01 m/0,02 rad,
revalida de forma optimista vuelta completa y ruta de entrada y vuelve a
correlacionar la escena bajo lock antes del commit. Otro cambio durante la
revalidación descarta el resultado y mantiene stop hasta replanificar.

La escena no es la única entrada mutable del solver. I-128 reprodujo que las
posiciones y yaw de la cadena podían cambiar durante el cálculo sin invalidar el
resultado. `07da8f4` toma y compara esa odometría dos veces, con las mismas
tolerancias explícitas al jitter utilizadas para la escena. Un desplazamiento
material descarta el plan, mantiene velocidad cero y replanifica desde la
muestra nueva.

I-130 queda cubierta por un gate final de escena y odometría inmediatamente
antes del lote de seguimiento. Un controlador N=1 ya no puede publicar 0,1 m/s
ni seguir `active` cuando el cálculo cruzó el timeout.

### Figures and letters

Geometric outlines and glyph strokes are resampled for the active fleet. A
minimum-cost robot-to-slot match includes a small hysteresis penalty, reducing
long crossings when the formation is recalculated. The controller can search
for a safer nearby centre when a requested outline intersects an obstacle.

La matriz visible encontró que el centro de una formación móvil comenzaba su
órbita antes de que los robots ocuparan los slots. El grupo perseguía entonces
una meta que ya se movía y la tarea podía llegar a estado terminal antes de la
muestra activa. El plan corregido ancla la trayectoria en el centro real de la
flota, inmoviliza ese centro hasta completar un tiempo de asentamiento y mantiene
la orientación de letras y figuras mientras se desplazan.

`07da8f4` cierra I-129, I-131 e I-132. El gate de formación vuelve a comprobar
la frescura literalmente después del cálculo geométrico e inmediatamente antes
de publicar; las longitudes distintas de `name` y `pose` invalidan de forma
atómica el snapshot; y los cuatro componentes crudos de cada cuaternión se
validan antes de convertirlo a yaw. Ante cualquiera de estas condiciones no
sale el primer comando no nulo y la tarea falla correlacionada con stop global.

El mismo commit cierra I-133 al rechazar explícitamente `linear.y/z` y
`angular.x/y`, aunque sean finitos y pequeños. I-134 hace fallar cerrado los
callbacks de formación, seguimiento y transporte ante cualquier nombre
duplicado; la recuperación solo ocurre después de una instantánea completa y
unívoca. El conjunto de modelos configurados se filtra además por el mundo
activo para no convertir exclusiones ajenas al escenario en fallos espurios.

Para una trayectoria circular no basta con validar el primer fotograma. El
controlador muestrea una vuelta completa de la huella rígida, añade margen para
los arcos entre muestras, busca un centro seguro y puede reducir el radio de
forma determinista hasta el mínimo configurado. Luego calcula rutas de entrada
por lotes, reservando slots ya ocupados. La geometría de modelos vivos se vuelve
a leer entre la instantánea y el commit y otra vez antes de publicar el lote
completo de `Twist`; ante un obstáculo nuevo, se invalida el plan y solo se
publica parada. Así se cierra la ventana TOCTOU sin mandar comandos parciales.

La revisión numérica posterior trata `NaN` e `Inf` como una pérdida de datos,
no como posiciones válidas. La callback de odometría rechaza de forma atómica
`x`, `y` o yaw no finitos y conserva la última muestra válida. Cada ciclo vuelve
a validar todos los robots y todos los modelos, incluso durante `MOVING`. Una
excepción inesperada del control vivo cambia la tarea a `FAILED`, cancela el
solver y su asignación pendientes y publica velocidad cero para toda la flota.

El commit `511e47c` cierra el gate que faltaba. Adaptive construye primero los
objetivos deformados y revalida esas coordenadas efectivas antes del lote; los
modos lineal y waypoints conservan un `placement_plan` con arena, exclusiones y
modelos vivos. Un plan ausente solo permite la espera inicial sin asignaciones
ni rutas. La flota completa de `Twist` se valida por finitud, ejes permitidos y
límites físicos del Burger antes del primer publish. Si un elemento falla, la
tarea pasa a `FAILED`, cancela el solver y envía parada a toda la flota.

La ejecución productiva de `1448a31` mostró que esas guardas eran correctas,
pero que el cálculo todavía podía bloquear la última callback de odometría. El
solver de diez robots tardaba varios segundos y el timeout estricto era 0,75 s.
I-136 lo mueve a un worker único y coalescente; una generación impide que un
resultado de otra tarea, forma o flota llegue al control. Cada inicio exige una
muestra nueva por robot. I-137 intersecta los límites propios de evasión con la
envolvente de formación y los máximos del Burger, de modo que `apply_avoidance`
no puede devolver 2,84 rad/s a un controlador configurado para 1,5 rad/s.

I-138 cubre la segunda ventana observada: durante el solve, los robots recién
creados podían asentarse hasta 0,139 m. El snapshot incluye posición y yaw y se
correlaciona en el commit. Si la escena no cambió, las poses son finitas y no
existe contacto, se mantiene `Twist=0` y se recalcula desde la foto viva hasta
dos veces. Una escena móvil, contacto, dato corrupto o churn persistente falla
cerrado sin reducir el margen de 0,30 m. La reproducción S N=10 utilizó un
replan y aprobó con error 0,0985 m, RTF 2,9929 y cero colisiones.

El freeze posterior se ejecutó desde una imagen nueva y sin bind del checkout.
La repetición N=3 conservó una ventana `moving` de 75,0346 s, error máximo
0,0921 m y RTF 2,9965. N=10 conservó 75,0411 s, diez asignaciones y error máximo
independiente de 0,0974 m; el estado del comportamiento informó 0,0981 m. Su RTF
fue 2,9875. Ambas registraron cero colisiones. Las sondas visibles concurrentes
identificaron D3D12 en la RTX 3080 y midieron 58,493/57,507 FPS con RTF
2,996/2,984; el intento previo con `llvmpipe` se rechazó y no se mezcló con
estos resultados.

Los presupuestos de formación se expresan en tiempo de pared, pero se dimensionan
con un piso conservador RTF 2,7. Los tiempos simulados de adquisición medidos
para N=3/5/7/8/9/10 fueron 15,15/86,25/156,65/126,05/115,20/208,75 s. Los
límites de pared 35/40/65/55/50/85 s dejan respectivamente
79,35/21,75/18,85/22,45/19,80/20,75 s simulados de margen. El control conserva
la velocidad lineal nominal máxima del Burger, 0,22 m/s; el aumento de timeout
no acelera la física ni relaja la precisión.

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

El destino aparece además en Gazebo como una huella magenta y una caja
translúcida de 0,4 m, unidas a un modelo cinemático, sin gravedad ni colisión.
Se evita el modelo `static` porque Gazebo Classic no refresca de forma fiable su
visual después de `set_model_state`. Transporte publica una sola actualización
latched a `/gazebo/set_model_state` y confirma la posición
observada por `/gazebo/model_states`; no realiza una llamada RPC bajo los locks
del comportamiento. El estado `command_published/synchronized/position` cruza
una lista permitida del orquestador hasta el frontend. Si falla este apoyo
visual, la tarea continúa y la interfaz lo presenta como advertencia, no como
éxito ficticio.

`arrival_tolerance` se puede ajustar entre 0,15 y 0,75 m y vale 0,25 m por
defecto en la interfaz; el valor legado de 0,50 m se conserva cuando el
parámetro no existe. Reducirlo obliga a acercar más la caja al mismo objetivo.
No se reducen masa ni fricción para producir más desplazamiento aparente: la
aceptación cargada mantiene 0,75 kg y `mu=0.25`, porque ese perfil distingue el
empuje colectivo del esfuerzo de un solo robot. Para ensayos rápidos ya existe
el perfil de práctica de 0,25 kg y `mu=0.05`.

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

El corte local final añadió dos corridas visibles N=4 con RC=0 sobre la imagen
`02fbd5c2…27f4864`, todavía anterior únicamente al ajuste posterior de cámara.
Al omitir `arrival_tolerance`, el perfil legado de 0,50 m desplazó la carga
0,5022 m, terminó a 0,4978 m de la meta, mantuvo RTF 2,9965, acreditó 4/4
empujadores útiles y no registró colisiones. Para reproducir el valor actual de
la interfaz sin alterar el repositorio, el segundo ensayo inyectó 0,25 m
solamente en memoria dentro del arnés. Ese ensayo desplazó la carga 0,7535 m,
terminó a 0,2465 m, mantuvo RTF 2,9963 y acreditó 4/4 empujadores útiles, una
fracción simultánea de 0,8707, velocidad máxima de 0,1664 m/s, aceleración
máxima en ventana de 0,6281 m/s² y cero colisiones. `tb3_1` avisó del hallazgo
a `tb3_0`, `tb3_2` y `tb3_3`; hubo picos de cuatro robots en búsqueda y cuatro
en reunión, mientras el marcador quedó publicado y sincronizado en `x=-3.5`,
`y=-4.0`.

El cliente visible de esas corridas identificó
`D3D12 (NVIDIA GeForce RTX 3080)`, 57,182 FPS de cámara, 58,795 eventos de
posrenderizado por segundo, RTF 2,997 y un viewport 990×334. La evidencia
temporal se conserva en
`/tmp/robotswarm-local-final-visible-20260722T0444Z/`; los SHA-256 de las
capturas `approach`, `search`, `done` y `zoomout2` son
`fa356ca8b084ee62e093f5993b1134a5b65ac2a857e492096e1906a0677c19d6`,
`d27774ed3560056919728b83dce8386a28187ba4f0e9d7234ac8959e706aff7b`,
`044573e7ca9a43d347947e51e2db6560e364006b737f4242f7c49c445ca049b4` y
`9e88ce8444111379ce740c2c7ae4fb01e20c516dcc585f5da4c4c12fe6e1e6c7`,
respectivamente.

La revisión visual independiente detectó que la pose de cámara
`0 -12 10 0 0.7 1.5708` recortaba el marcador cerca de `y=-4`; la captura
`zoomout2` demostró que el modelo sí estaba sincronizado y aisló el defecto en
el encuadre. I-135 cambió la pose a `0 -14.4 12 0 0.72 1.5708` y fijó ese
contrato en la prueba del mundo. Sobre la imagen reconstruida `e17579ed…ae0d`,
la sonda visible exacta midió 57,279 FPS de cámara, 58,791 eventos de
posrenderizado por segundo y RTF 2,997 con D3D12/RTX 3080 y viewport 990×334.
`SetModelState` llevó el marcador a `(4,4)`, `(-4,4)`, `(4,-4)` y `(-4,-4)`;
la huella y la caja permanecieron completas y dentro de los muros en los cuatro
casos. Aprobaron 8/8 pruebas del mundo y `git diff --check`, por lo que I-135 se
cierra localmente. En
`/tmp/robotswarm-local-final-camera-20260722T0454Z/`, las capturas NE, NW, SE y
SW tienen SHA-256
`75c53cf725a69c20568d00a529754d463bb79957c2e10e904d4b46e2c22ee839`,
`84750bdbb15eddb1babaeffedf10205de84a816fd6e474e13a28f35a887152ca`,
`ade73f6e43125f0945a60606d3db7cb6e10538abc19b2d3fb18e1ead1b848e14` y
`86f32de8ad0628c0fae1d97b53a1ae3ed9dcf86f73fa753ec5c89a886ea48f51`.
Toda esta evidencia es local y no productiva.

Finalmente se repitió el caso completo sobre la imagen corregida
`e17579ed…ae0d`, sin eliminar la flota durante la corrida y con 0,25 m inyectados
solo en memoria. Terminó `PASS`/RC=0: el objeto avanzó 0,7523 m, quedó a
0,2477 m de la meta y mantuvo RTF 2,9962. Los 4/4 robots fueron útiles, la
fracción simultánea fue 0,9355, hubo picos de cuatro buscadores y cuatro robots
en reunión, y `tb3_1` avisó a los otros tres. No se registraron colisiones; la
velocidad máxima fue 0,1649 m/s y la aceleración máxima en ventana, 0,7883
m/s², inferior al límite de 1,0 m/s². La cronología de pared situó `SEARCH` en
0,278 s, `APPROACH` en 39,191 s, `PUSH` en 107,725 s y `DONE` en 131,991 s. El
marcador quedó sincronizado en `x=-3.5`, `y=-4.0`.

En el mismo directorio temporal, `final-n4-search.png`, SHA-256
`d036d9814b893441f2c57be52a4ab58415339f85bfeba6055177820417211f11`,
es la única captura de esta repetición aceptada como evidencia visual de fase:
muestra simultáneamente el objeto, el marcador completo y los cuatro robots en
búsqueda. El log saneado `transport-n4-tolerance-025-final.log`, SHA-256
`3ad8ad2b65b7589f34d52f716990d8675e57cef9474d9679af9407a81612ac43`,
sustenta el resultado terminal. `final-n4-push.png` y `final-n4-done.png` no se
utilizan: una carrera entre la captura y el `cleanup/reset` impide atribuirles
la fase de empuje o la pose final. Esta exclusión evita que una imagen ambigua
reemplace la serie temporal correlacionada.

La primera validación exterior de v11 supuso erróneamente que cada flota
reiniciaría sus ordinales en cero. El contrato corregido admite el offset fresco
del gestor, pero exige namespaces canónicos, bloques contiguos y distintos,
mapas iguales y asignación monotónica. Aquel arnés aprobó 38/38; el arnés vigente
aprobó 49/49 e incluye
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

El reemplazo de la carga de aceptación necesita una barrera adicional. Gazebo puede
entrelazar las respuestas de borrado, aparición y posicionamiento; una llamada
exitosa no garantiza que el `ModelStates` siguiente pertenezca a la misma vida
del modelo. La sonda usa generaciones, exige una ausencia fresca y luego dos
presencias frescas consecutivas en la pose correcta. Solo repite el reemplazo
—como máximo tres veces— cuando existe evidencia autoritativa de desaparición o
de modelo ausente. Errores permanentes y cierre del proceso fallan de inmediato.
Los diagnósticos conservan como máximo 16 eventos saneados, conteo total,
truncamiento y categorías permitidas; no retienen respuestas crudas.
Antes de borrar, crear, posicionar o clasificar un fallo como recuperable, la
sonda comprueba tanto `stop_requested` como `rospy.is_shutdown()`. Si cualquiera
está activo, termina sin abrir otra generación ni repetir servicios de Gazebo.

El control de transporte aplica la misma transacción de comandos en las tres
fases activas: `SEARCH`, `APPROACH` y `PUSH` construyen y validan el lote entero
antes del primer `Twist`. De este modo, un segundo robot con salida `Inf` no deja
escapar el comando válido del primero. Una excepción del ciclo termina en
`FAILED` con el `task_id` y la época originales, aun cuando no alcance el
epílogo normal de estado. El acumulador temporal utilizado para reunir comandos
de aproximación se retira en `finally`, también si la evitación lanza una
excepción.

La geometría de Gazebo se acepta ahora como una instantánea indivisible. Si las
listas `name` y `pose` tienen longitudes diferentes, se vacían pose de carga,
obstáculos y marcador, se invalida la frescura y se publica el fallo
correlacionado. Antes, `zip` podía conservar la pose vieja de la carga mientras
refrescaba el timestamp del mensaje truncado. La odometría de transporte valida
además los cuatro componentes crudos del cuaternión; un `Inf` que por casualidad
produzca un yaw finito ya no atraviesa el gate.

I-127 demostró que esa comprobación al inicio aún no bastaba. Un ciclo inducido
de 1,0 s, mayor que el timeout de 0,75 s, alcanzó a publicar 0,12 m/s en
`SEARCH`. `4450c13` revalida `ModelStates` y la odometría de toda la flota en el
gate final, antes del primer comando no nulo. El mismo `command_lock` permanece
tomado hasta el último `Twist` y el commit de `ObstacleAvoidance`, de modo que el
lote no se divide si el timeout cruza entre dos robots. Las rutas de publicación
directa aplican la misma frontera; los ceros de parada pueden publicarse aun con
datos vencidos para conservar el fallo seguro.

El preflight visible inicial del cambio cinemático, anterior a I-135, identificó
`D3D12 (NVIDIA GeForce RTX 3080)`, 58,206 FPS de cámara, 62,512 eventos de
posrenderizado por segundo, RTF 2,996 y un viewport 990×351. El reporte local
`/tmp/robotswarm-corrected-ghost-gui-report.json`, con permisos `0600`, tiene
SHA-256
`98a4651069f1ea8199d26d278da0b7a4df273b54b93ffaa0a9b65bfd12e1f5d5`.
Las capturas Windows/WSLg realizadas con `CopyFromScreen` quedaron congeladas o
recortadas después de redimensionar la ventana y fueron rechazadas; no se usan
como evidencia del encuadre. Las capturas limpias posteriores de la búsqueda
N=4 y de las cuatro esquinas cierran el encuadre localmente; todavía deben
sanearse/versionarse y repetirse después del despliegue.

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

## Secuencia de liberación pendiente

La validación local final añadió I-143: el secuenciador de rutas ya no retiene
los lotes posteriores cuando un robot está dentro de la banda segura de
histéresis, aunque todavía no cumpla la tolerancia estricta de convergencia.
El candidato exacto aprobó S/N=10 con error máximo 0,0936 m, RTF 2,9912 y cero
colisiones. Los umbrales finales de precisión y seguridad no cambiaron.

La primera repetición sobre `1d497d4` aprobó triángulo, A y S, pero rechazó
cuadrado, V y diamante al alcanzar sus presupuestos históricos antes de la
ventana activa. I-144 eleva solo esos tres límites a 90 s y admite hasta
0,15 rad de yaw de asentamiento sin movimiento comandado. Las tres repeticiones
locales aprobaron en 62,64–86,34 s con error ≤0,0960 m, RTF ≥2,90 y cero
colisiones. El plan sigue abierto hasta integrar el parche mínimo y repetir la
matriz sobre su SHA.

I-144 se integró como `ea25434`. Dos dispatches GPU se detuvieron antes del
lease por timeout DNS. I-145 añadió tres intentos acotados solamente para
errores de transporte; se integró mediante la PR #109 y el despliegue
`30055847809` activó `2445a37`. I-146 se integró mediante la PR #110 y el
despliegue `30060062277` activó `e3dc7ad`. Las seis filas posteriores llegaron
a video HLS.

La misma matriz abrió I-147: S/N=10 mantuvo seguridad y RTF, pero un miembro
liberado dejó de progresar fuera de la histéresis y retuvo dos robots de lotes
posteriores. La PR #111 publicó un detector de 0,01 m por 20 s como `917b06b`;
API y dos Chrome visibles aprobaron, pero S/N=10 volvió a quedar retenida
porque una deriva microscópica reiniciaba el reloj. El segundo delta local
exigió 0,10 m por ventana, omitió el siguiente lote positivo, publicó cero,
reutilizó el replan vivo limitado a dos intentos y expuso telemetría del lote.
Se integró mediante la PR #112 como `2193c3d`; CI, backend y GPU aprobaron,
pero dos S/N=10 productivas volvieron a agotar el presupuesto con seis lotes y
un replan, aunque conservaron RTF ≥2,9908 y cero colisiones.

El diagnóstico mostró que el problema restante no era el detector, sino la
serialización: el lote siguiente esperaba la llegada casi completa del
anterior aun después de quedar libre el cruce. El tercer candidato reconstruye
los corredores restantes desde las poses vivas y adelanta el lote siguiente
solo cuando ya no hay intersección con el despeje del plan; todos los lotes
liberados continúan moviéndose. Tres S/N=10 visibles aprobaron 75 s activos con
error ≤0,0963 m, RTF ≥2,9887 y cero colisiones. La suite local aprobó 631/631
pruebas ROS y 254/254 contratos.
El gate API usa letra A/N=3 para acreditar solapamiento real sin pausas
artificiales.

La PR #113 integró esa corrección como `ec4980b`; CI, backend y GPU aprobaron
en un intento. Su primera S/N=10 productiva liberó nueve robots y mantuvo
progreso sin replans ni colisiones, pero el presupuesto histórico terminó a
252,816 s simulados, antes de despejar el último corredor. I-148 corrige solo
el supuesto del arnés: 120 s de pared cubren 324 s al RTF conservador 2,7.
Precisión, ventana activa, RTF, aceleración y seguridad permanecen iguales.

1. Publicar la calibración I-148 y su documentación en un único PR correctivo. Usar
   un solo ciclo normal de CI y no hacer reruns de jobs aprobados para duplicar
   evidencia.
2. Después del merge, comprobar los servicios públicos y despachar el workflow
   GPU una sola vez para el SHA exacto. Verificar imagen instalada, NVIDIA, RTF,
   unidad versionada y ausencia de un release parcial.
3. Ejecutar las seis formaciones corregidas en sesiones frescas. Exigir roster
   completo, ventana activa, precisión, movimiento natural, cero colisiones,
   RTF ≥2,90, video visible y limpieza por caso.
4. Repetir transporte N=1 sobre el SHA nuevo. La comprobación previa contra
   `1448a31` ya aprobó la preservación del informe, ROS, HLS y cleanup. Después,
   desde React, ejecutar N=4 con
   tolerancia 0,25 m y confirmar búsqueda de toda la flota, aviso, encuentro,
   contribución útil, marcador sincronizado, interacción y fullscreen.
5. Emparejar la sonda de carga de 0,75 kg y GRF con el preflight NVIDIA sobre el
   mismo master Gazebo. Exigir render ≥45 FPS, HLS ≥27 FPS y RTF ≥2,90, sin
   reducir masa o fricción para obtener el aprobado.
6. No elevar cuentas por SQL ni crear usuarios para completar el recorrido
   Admin. Solo se repetirá esa superficie si el operador proporciona una
   credencial administrativa existente y acepta la revocación de sus tokens;
   el recorrido User de `1448a31` permanece válido.
7. Comprobar al final cero tareas, robots, sesiones, leases, contenedores,
   perfiles y procesos temporales. Incorporar únicamente evidencia saneada y
   separada por SHA; conservar el rollback hasta completar la observación.

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
