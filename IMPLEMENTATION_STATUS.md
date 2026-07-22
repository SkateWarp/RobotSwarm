# Estado de implementación de RobotSwarm

**Fecha del corte:** 2026-07-21
**Alcance:** estado de producción en `fbef23e` y cambios locales posteriores de
instrumentación, frontend y orden causal `Accepted → Running` del worker. No
constituye todavía el acta de cierre.

La versión anterior de este archivo describía como trabajo futuro pendiente el
orquestador de tareas, los tres comportamientos, los archivos de lanzamiento,
los mundos y la configuración de RViz. Esos componentes ya existen. Este
documento distingue entre lo implementado en el repositorio, lo demostrado en
una simulación visible y lo que aún requiere aceptación final en producción.

El registro detallado de comisionamiento en español, incluidos los intentos
fallidos y la evidencia visual, está disponible en
[docs/informe-comisionamiento-final.md](docs/informe-comisionamiento-final.md).

## Alineación entre marcos tecnológicos

| Capa | Responsabilidad actual | Estado del repositorio |
| --- | --- | --- |
| Frontend React | Controles de sesión, flota y tareas pertenecientes al usuario; estado en vivo; reproductor privado de escena HLS | `fbef23e` publicado por Cloudflare; la normalización local de fechas UTC sin sufijo y su regresión aún no se han desplegado |
| Backend .NET | Autenticación, comprobaciones de propiedad, cola de sesiones, comandos del worker, resultados de tareas, concesiones del visor y concesiones de drenaje de despliegue | `fbef23e` desplegado y sano |
| Worker GPU .NET | Ciclo de vida aislado de sesiones Docker, puente de comandos ROS, latidos, selección inmutable de imagen y supervisión del publicador del visor | `fbef23e` activo; unidad versionada con AF_NETLINK y GPU NVIDIA comprobada |
| ROS Noetic y Gazebo | Flota dinámica de TurtleBot3 Burger, control de seguridad, orquestación de tareas y tres comportamientos escalables | Imagen de `fbef23e` activa; subresultado físico N=2 y prueba de aceptación cargada N=4 ejercitados en simulación visible |
| MediaMTX y auxiliar del visor | Pantalla X privada y `gzclient` por concesión, publicación H.264/RTSP y origen HLS interno de baja latencia | HLS privado operativo; dos sesiones independientes y una prueba de aceptación cargada se observaron desde Chrome visible |

Las capas utilizan un único ciclo de vida correlacionado en vez de controles
puntuales separados: el frontend invoca al backend autenticado, el backend
emite comandos persistentes para el worker, el worker traduce esos comandos al
espacio de nombres ROS privado y ROS informa el estado de la flota y las tareas
a través del concentrador del worker. El navegador no recibe acceso
administrativo a Docker, ROS, Gazebo, VNC ni MediaMTX.

El PR #102 integró el endurecimiento final como `1182dec`; el PR #103 añadió el
orden causal de cuentas/sesiones y los nuevos arneses como `f14776b`; y el PR
#104 corrigió el analizador del drenaje para Python 3.8. El `main` resultante,
`fbef23eaae2b1b1d5be51ad3fa03e0298239289a`, es el SHA observado en frontend,
backend, worker e imagen ROS durante las pruebas públicas del 21 de julio. Esto
deja a `538ba066` como antecedente de I-063, no como versión vigente.

Sobre `fbef23e` aprobaron la aceptación API multiusuario, dos ventanas visibles,
el recorrido Admin, las cuatro anchuras adaptables y la prueba de aceptación
cargada N=4. El árbol de trabajo contiene correcciones posteriores para hacer
la evidencia más estricta, interpretar como UTC las fechas .NET sin zona en el
navegador y preservar el orden causal `Accepted → Running` del worker. Estas
correcciones
todavía requieren un único PR, su CI y la comprobación posterior al despliegue;
por esa razón el lanzamiento permanece abierto aunque la base productiva esté
alineada.

## Componentes implementados

### Paquete ROS y simulación

El paquete ubicado en `swarm_ws/src/robot_swarm_bridge` contiene:

- mensajes y servicios generados para controlar flotas y tareas;
- `scripts/core/task_orchestrator.py`, incluida la validación de comandos, la
  correlación por identificador de tarea, pausa/reanudación/cancelación, parada
  de emergencia, reporte de estado y un latido de control que falla de forma
  cerrada;
- gestión dinámica de flotas e identificadores de ejecución normalizados desde
  `tb3_0` hasta `tb3_9`;
- evasión de obstáculos y entre robots aplicada a los comandos finales de
  movimiento;
- archivos de lanzamiento para los escenarios principal del enjambre,
  seguimiento del líder, formación y transporte;
- `empty_arena.world`, `swarm_arena.world`, el modelo de carga de transporte y
  una configuración de RViz; y
- sondas unitarias y de aceptación en vivo para algoritmos, física, limpieza,
  renderizado de GUI, distancia de seguridad, notificación de búsqueda y
  transporte de carga.

Las sondas en vivo emplean identificadores correlacionados de tarea y
eliminación. Detienen la tarea exacta antes de modificar el estado compartido de
Gazebo, rechazan la eliminación parcial de la flota, comprueban una lista de
flota vacía y la ausencia de modelos TurtleBot o monitores de velocidad
residuales, y propagan los fallos de limpieza al resultado final distinto de
cero. Esto es instrumentación de aceptación: las mismas condiciones todavía deben observarse
en la revisión GPU desplegada.

### Comportamientos escalables

Los tres comportamientos solicitados están presentes y calculan sus asignaciones
a partir de la flota activa, en vez de exigir una cantidad fija de robots:

1. **Seguimiento del líder.** Los seguidores muestrean la trayectoria recorrida
   por el líder con desplazamientos de distancia y generan una cadena continua
   cuya longitud se adapta al tamaño de la flota.
2. **Figuras y letras.** La geometría y los trazos de los glifos se remuestrean
   según la cantidad de robots activos; luego se asignan a los robots mediante
   una correspondencia de costo mínimo con histéresis para reducir cruces.
3. **Transporte colaborativo.** Todos los robots buscan mientras el objeto es
   desconocido. El primero que lo encuentra publica la posición correlacionada
   del objeto; los robots restantes la confirman y se reúnen. Los robots pueden
   empujar la carga directamente o a través de una cadena acotada de compañeros,
   con preferencia por el contacto con el objeto. La evidencia de finalización
   se correlaciona con la tarea y debe representar la contribución de toda la
   flota solicitada.

El controlador de transporte utiliza de forma predeterminada el planificador
GRF de implementación independiente y conserva el planificador anterior como
alternativa explícita. La evasión de colisiones permanece activa durante la
búsqueda, reunión, formación, seguimiento y transporte; el contacto con la
carga se trata por separado para que la capa de seguridad no rechace por error
el empuje previsto.

El destino solicitado por la interfaz también se coloca como un modelo visual
sin colisión en Gazebo. El controlador publica la posición en una cola latched,
sin realizar una llamada síncrona mientras mantiene los bloqueos de la tarea, y
confirma después el resultado observado mediante `model_states`; un fallo de
este apoyo visual no detiene a los robots. El modelo combina una huella y una
caja translúcida, por lo que aparece en el visor HLS aunque RViz no esté abierto.
La interfaz permite ajustar entre 0,15 m y 0,75 m el margen de llegada; un valor
menor exige un recorrido mayor hasta el mismo destino. Este ajuste no cambia la
masa ni la fricción de la carga cargada de aceptación (0,75 kg y `mu=0,25`),
porque ese perfil conserva la prueba de que un solo Burger no sustituye a la
flota.

### Plano de control web

La ruta de control orientada a producción incluye:

- sesiones de simulación autenticadas y pertenecientes a una cuenta;
- planificación FIFO sobre workers registrados, con una cantidad acotada de
  robots por sesión;
- operaciones para redimensionar la flota e iniciar, pausar, reanudar y cancelar
  tareas, además de la parada de emergencia;
- confirmación persistente de comandos del worker y latidos de sesión;
- progreso y resultados terminales correlacionados de las tareas, además de
  tiempos de espera para tareas finitas que dejan de progresar; y
- una concesión de drenaje autenticada por el worker, vinculada a la revisión Git
  exacta que utiliza el flujo de trabajo de despliegue GPU.

Las mutaciones de cuentas y sesiones ahora comparten un orden de bloqueo
explícito. Un bloqueo asesor global sobre el conjunto de administradores protege
la regla del último administrador habilitado; los flujos que cambian correos
toman después un bloqueo global sobre el conjunto de correos, seguido por
bloqueos de cuentas en orden de account-ID. Estos recursos ordenan las
mutaciones de rol, correo, contraseña, deshabilitación y sesión. La cuenta
autenticada se vuelve a leer únicamente después de adquirir el bloqueo
compartido, y su indicador de habilitación, rol y declaración
`account_version` aún deben coincidir.
En PostgreSQL, esa lectura utiliza `SELECT ... FOR SHARE`: si una transacción
`SERIALIZABLE` fijó una instantánea antigua mientras esperaba, PostgreSQL genera
`40001` y la ruta realiza un nuevo intento real en vez de aceptar credenciales
obsoletas. El caso reproducido de instantánea obsoleta ahora devuelve HTTP 401 y
no deja ninguna sesión. Las solicitudes administrativas que modifican otra
cuenta no dependen solamente de la decisión de autorización previa al manejador.
Cuando la operación afecta la pertenencia al conjunto de administradores,
primero toma el bloqueo global de administradores; después adquiere el bloqueo
compartido de la cuenta actora y el exclusivo de la cuenta objetivo, en orden
de account-ID. La cuenta actora vuelve a validarse mientras esos bloqueos están
activos. Una solicitud cuyo actor Admin fue revocado durante la espera ahora
devuelve HTTP 401 y no modifica la cuenta objetivo.

La reconciliación de sesiones terminales forma parte de la transacción del
latido del worker. Bloquea las sesiones terminales en un orden estable antes
de cargar sus comandos y confirma la transacción antes de publicar por SignalR.
Por tanto, la limpieza de cuentas y el latido comparten una sola secuencia
causal de comandos, en lugar de competir por insertar el mismo `StopSession`.
El contrato público OpenAPI/Swagger también declara la posible respuesta HTTP
409 que devuelve el `DELETE` de una sesión después de agotar sus reintentos
acotados de serialización.

El seguimiento del líder es continuo de manera intencional y no se considera
fallido solo porque siga activo sin alcanzar un valor finito de finalización.
Las tareas finitas de formación y transporte se supervisan para comprobar su
aceptación y progreso.

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
- **Operación de sesión:** el frontend muestra la lista en ejecución informada
  por el worker, sus roles, espacios de nombres, estados y última actualización.
  También reintenta el primer fallo de SignalR sin requerir recargar la página y
  pide confirmación antes de detener y liberar una sesión completa.
- **Visor:** `Cerrar visor` revoca la concesión propia, libera entradas y emite el
  comando persistente `StopViewer`, que detiene solo ese publicador sin apagar
  ROS, Gazebo, el contenedor o la red de la sesión.
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
secciones 14/14. Un recorrido público con Chrome normal aprobó Historial para el
rol `User`, el menú reducido, 403 en dos puntos de acceso `Admin` y la
redirección de las cuatro rutas restringidas, con limpieza completa. Sobre el
árbol local completo aprobaron worker 121/121, frontend 141/141 en 28 conjuntos
de pruebas, el análisis estático de todos los archivos frontend modificados y
la compilación de producción. Falta publicar el único PR final y repetir la
aceptación sobre su SHA desplegado. Esta última frase describe aquel corte
anterior a los PR #102–#104; ya no es el pendiente actual.

El cierre que después quedó integrado por los PR #102 y #103 conserva esas
cifras como antecedentes. El conjunto completo de pruebas del backend sin
conexión PostgreSQL aprobó 250 pruebas y omitió por diseño las 8 de activación
opcional (258 descubiertas); el filtro no-PostgreSQL confirmó 250/250 y las
focales de cuentas 23/23. Al habilitar PostgreSQL 17.10 aprobaron 8/8; la
compilación del backend terminó con cero errores y cuatro advertencias
heredadas. Worker aprobó 124/124, ROS 427/427 y frontend 149/149 en 28 conjuntos
de pruebas, además del análisis estático de 75 archivos y la compilación de
producción. Los siete arneses sin conexión aprobaron entonces 193/193 contratos,
desglosados como 16+38+13+44+37+30+15. También aprobaron `py_compile` sobre 14
módulos, la sintaxis Bash y `git diff --check`. Son resultados locales del
candidato, no evidencia de la aceptación pública posterior. El recuento 193/193
se mantiene como dato histórico de I-091 y no como total del árbol de trabajo actual, que
añadió contratos de clic confiable, MSE, canales de marcadores, límites del
observador y limpieza.

El delta I-088 endurece tres límites de aceptación. La prueba rápida iniciada
desde la interfaz ya no considera suficiente que la API declare cuatro
buscadores: cada robot debe recorrer al menos 0,015 m durante `SEARCH`. En ROS,
`ObstacleAvoidance` detecta el flanco filtrado en el mismo ciclo de seguridad y
`CollaborativeTransport` sella UUID y secuencia de fuente, tarea, fase, secuencia
de control y tiempos. El flujo v2, acotado a 128 eventos, forma parte del mismo
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
`navigationGTSConfig.js`. I-090 trasladó al backend la validación de `Create`,
`PUT` y `PATCH`, la política de contraseña 8–16 y el correo canónico con
exclusión Admin→correo→cuentas, columna generada e índice único. I-091 eliminó la
divergencia entre `Trim()` y `btrim()` mediante el mismo conjunto explícito de
espacio, TAB, LF, VT, FF y CR en C# y PostgreSQL; la migración falla sin cambios
ante datos históricos no soportados o duplicados. La comprobación productiva
previa de solo lectura observó `1182dec`, estado `Healthy`, nueve cuentas y cero
anomalías agregadas, sin registrar PII. No demuestra despliegue del candidato.

### Visor privado en el navegador

La ruta principal de entrega implementada es:

```text
pantalla X privada + gzclient
        -> FFmpeg H.264
        -> ingesta RTSP autenticada
        -> HLS de baja latencia de MediaMTX (red Compose privada)
        -> proxy autenticado del backend
        -> hls.js en el navegador del usuario propietario
```

Cada concesión recibe una ruta canónica, un token privado de publicación, un
token de lectura separado, una fecha de caducidad y un directorio privado de
pantalla/ejecución. El navegador envía su token de lectura en cada solicitud de
lista de reproducción y segmento multimedia. El backend lo valida y utiliza una
credencial CDN protegida diferente hacia MediaMTX; la credencial del navegador
no cruza ese límite privado. El puerto HLS 8888 de MediaMTX no se publica en el
equipo anfitrión. WHEP se mantiene como transporte secundario opcional;
ICE/TURN público ya no es un requisito previo para aceptar la ruta HLS.

El modelo Burger estándar no posee un sensor de cámara compatible, por lo que
la capacidad actual verificable del visor es `Scene`. Las solicitudes
`RobotCamera` se rechazan. La publicación y el proxy HLS son controles de acceso
protegidos. La corrección AF_NETLINK forma parte de la unidad versionada
`fbef23e`; no se utilizó ninguna anulación de diagnóstico en la corrida aceptada
con dos navegadores. Dos usuarios recibieron flujos independientes a aproximadamente
30 FPS decodificados, y detener una sesión dejó activos el flujo y la tarea de
la otra.

La prueba cargada tarda más que la concesión de producción de cinco minutos.
Por ello, la pérdida HLS al final del ensayo no invalida las mediciones tomadas
durante la carga y `PUSH`: la prueba conserva el muestreo correlacionado y acepta
después el rechazo esperado del token vencido. Una captura de producción reveló
además que el contador podía mostrar unos 244 minutos, porque JavaScript
interpretaba como hora local una fecha .NET sin `Z`. El analizador compartido
que la normaliza a UTC está implementado y probado localmente, pero aún no está
desplegado.

La primera prueba rápida endurecida de transporte encontró además una carrera
real del worker: el sondeador alcanzó a persistir `Running` dos segundos antes
de que la ruta de ejecución del comando informara `Accepted`. El backend rechazó
correctamente la regresión posterior. El arreglo local mantiene `Accepted`
después de una
publicación ROS satisfactoria y suspende la recuperación y el sondeo, antes y
después de la lectura, mientras el mismo `StartTask` siga vivo. Las regresiones
concurrentes cubren el descubrimiento, una marca de progreso previa y los fallos
de publicación o reporte; el worker aprobó 129/129. El intento productivo se
conserva como rechazado y debe repetirse sobre el SHA final.

El PR #100 también añadió un cierre explícito del visor acotado al propietario.
La revocación es idempotente y queda cercada ante el reemplazo de la concesión.
El worker conserva un marcador de cierre de corta duración para que un inicio
retrasado no pueda reactivar un visor ya cerrado. Salir de la pantalla completa
nativa libera todas las entradas retenidas. `Escape` se mantiene local al
navegador, incluso ante repetición automática y cuando se perdió su liberación
de tecla anterior, de
modo que no pueda convertirse en una tecla de Gazebo al volver a abrir la
pantalla completa. El botón de interacción solo arma el canal; no sintetiza una
entrada de Gazebo, y deshabilitarlo emite una liberación global. Los arneses
cargado y de matriz también aplican su umbral compartido
`MINIMUM_BROWSER_VIDEO_FPS` (actualmente
27.0) antes de aceptar una captura tomada durante `PUSH`. La corrida de dos
navegadores ya verificó interacción, pantalla completa y parada independiente
sobre `fbef23e`; la repetición final se hará después de publicar el analizador
UTC y el endurecimiento instrumental actual.

### Despliegue y endurecimiento del equipo anfitrión

- El despliegue del frontend continúa a cargo de la integración Git de
  Cloudflare.
- El despliegue del backend continúa a cargo del flujo de trabajo protegido del
  backend sobre la VM de la LAN.
- El despliegue del worker GPU utiliza un flujo de trabajo protegido, separado y
  de activación manual. Este adquiere y valida automáticamente una concesión de
  drenaje del backend; una casilla marcada por un operador no se acepta como
  evidencia de que el worker esté vacío.
- El lanzamiento GPU utiliza el SHA actual exacto de `main` que aprobó CI y la
  imagen ROS exacta identificada por contenido.
- VNC y websockify son servicios de mantenimiento vinculados a la interfaz de
  bucle local. No constituyen el visor multiusuario del navegador.
- PostgreSQL se mantiene privado, mientras el backend se publica únicamente en
  la dirección LAN configurada que utiliza Nginx Proxy Manager.

## Evidencia obtenida hasta el momento

Las afirmaciones siguientes corresponden a pruebas ya realizadas. Algunas se
ejecutaron sobre `fbef23e` y otras son antecedentes locales; ninguna sustituye
la aceptación del próximo SHA que incorporará el delta aún sin publicar:

- El registro de trabajo documenta una matriz visible de Gazebo con formaciones,
  letras, seguimiento del líder y transporte colaborativo en puntos
  representativos del intervalo de 1–10 robots. El repositorio conserva la
  evidencia estructurada completa de los casos de transporte y búsqueda N=10;
  la matriz constituye una observación histórica hasta que los casos
  representativos se repitan sobre la revisión final desplegada.
- La corrida repetida de transporte con diez robots alcanzó `DONE/completed`;
  los diez robots se reunieron y contribuyeron mediante dos raíces en contacto
  con la carga y cadenas de compañeros, sin deltas de colisión registrados.
- Durante esa corrida la carga avanzó 0.922 m con eficiencia direccional de
  0.9997; el factor de tiempo real medido de la física fue 2.9505 bajo carga.
- Una corrida separada de búsqueda distante registró movimiento de 10/10
  robots, una notificación del descubridor, 10/10 confirmaciones, respuestas de
  los otros 9 robots y cero deltas de colisión.
- La repetición aislada más reciente de transporte N=10 registró 977 muestras
  de búsqueda con los diez robots en movimiento, un descubridor más nueve
  avisos y confirmaciones, empuje útil de los diez robots, 59.14% de
  contribución de la flota completa, 0.5044 m de avance, RTF 2.9672, 49.960 FPS
  visibles en la RTX 3080 y ningún contacto inesperado.
- La prueba cargada N=4 se aceptó sobre el despliegue exacto `fbef23e`. Con una
  carga de 0,75 kg, los ensayos de uno, dos y cuatro robots avanzaron
  0,0070/0,0354/1,0836 m, con ganancia 154,8× y RTF
  2,9951/2,9975/2,9941. En GRF, los cuatro robots buscaron, recibieron el aviso,
  se reunieron y empujaron; hubo dos raíces y dos compañeros, 1.618 muestras
  útiles de flota y 0,5001 m de avance con eficiencia 0,9946 y RTF 2,9942.
  La sonda visible concurrente identificó D3D12/NVIDIA RTX 3080, 58,469 FPS de
  cámara, 62,489 eventos de posrenderizado por segundo y RTF 2,996. HLS entregó
  30,8 FPS antes y 30,0 FPS a ambos lados de la captura en `PUSH`, sin fotogramas
  descartados en el muestreo de cinco segundos. La limpieza cerró navegador,
  concesión, publicador, sesión, contenedor, red, procesos y espacios temporales.
  El [reporte versionado de carga N=4](docs/assets/commissioning-2026-07/final-fbef23e/carga-n4-reporte.json),
  que corresponde únicamente a `fbef23e`, tiene SHA-256
  `8ed754cd46172fb904bce79e5729c26c0dabd9de0659a092cc0415a41a4f110a`.
- Los intentos previos de la misma prueba permanecen clasificados como
  rechazados.
  Uno no inició reproducción HLS pese a tener RTSP/H.264 disponible; otro
  mezcló dos marcadores JSON emitidos por funciones de retorno concurrentes; un
  tercero terminó la sonda antes del marcador GRF; y el siguiente trató la caducidad
  normal de la concesión de cinco minutos como si fuera un fallo final de video.
  Esos resultados sirvieron para corregir el instrumento, no para sumar éxitos.
- La aceptación API de producción aprobó con dos cuentas, listas de flota
  N=3/N=7, aislamiento, tareas y limpieza. Dos ventanas Chrome visibles reprodujeron
  flujos independientes a unos 30 FPS; la tarea A terminó, la B continuó al
  detener A, y se ejercitaron interacción y pantalla completa. El recorrido
  Admin visitó Plantillas, Historial, Control, Robots, Grupos y Usuarios,
  comprobó los puntos de acceso administrativos y eliminó su grupo temporal.
- El recorrido adaptable de solo lectura sobre `fbef23e` aprobó en 360, 768,
  1366 y 1920 px: el panel quedó visible y dentro del área visible, sin
  desbordamiento horizontal. El
  [reporte adaptable versionado](docs/assets/commissioning-2026-07/final-fbef23e/responsive-reporte.json),
  que corresponde únicamente a `fbef23e`, tiene SHA-256
  `4e587e4d20d91602daf962f4f43e74b92017a4b3c537907517e06e5914739a74`.
- El endurecimiento local posterior separa el protocolo oficial y los
  marcadores vivos en canales distintos, limita líneas y evidencia, usa una
  escritura atómica por marcador y prueba la desaparición de cada observador y
  grupo de procesos. La sonda gráfica liga hash, PID y tick al `gzclient` vivo;
  la limpieza activa usa `pidfd`; el monitor N=4 conserva el estado real de
  `wait`; y la prueba del clic vive fuera del mundo JavaScript de React. También
  evita APIs ausentes en Python 3.8 (`Popen(umask=)` y `str.removesuffix`) y
  agrega diagnóstico explícito de MediaSource, SourceBuffer, AVC/H.264, estado
  del video y solicitudes HLS. Son cambios
  locales: no deben confundirse con el SHA productivo ni con CI ya ejecutado.
- La pasada local del árbol actual aprobó backend 250/250 con 8 casos PostgreSQL
  de activación opcional omitidos, frontend 159/159, ROS 429/429, worker 129/129
  y 237/237 contratos de los siete arneses. También aprobaron la compilación del
  frontend/backend, la imagen ROS, la migración idempotente, Compose y el
  publicador. No se utilizó GitHub Actions para estas comprobaciones; el CI del
  PR aún será autoritativo. Estos
  resultados corresponden exclusivamente al árbol local: no se atribuyen a
  `fbef23e` ni al futuro SHA de despliegue.
- Después del endurecimiento se comprobaron los puertos de mantenimiento de
  producción: VNC y websockify escuchan en la interfaz de bucle local. Los
  controles de acceso del visor permanecieron desactivados durante la línea
  base inicial y después se habilitaron juntos para la ventana de
  comisionamiento supervisada.
- Las páginas de administración y el ciclo `StopViewer` están presentes en el
  paquete público y fueron recorridos sobre `fbef23e`. Las capturas aún viven en
  directorios temporales; antes del acta final deben sanearse, copiarse a
  `docs/assets/commissioning-2026-07/` y añadirse al manifiesto de hashes.

Las mediciones sin procesar, capturas de pantalla, análisis de incidentes y la
distinción entre corridas negativas y aceptadas se registran en el
[informe de comisionamiento](docs/informe-comisionamiento-final.md).

## Trabajo pendiente antes de declarar completo el lanzamiento

1. Consolidar el analizador UTC, el orden `Accepted → Running` del worker, el
   endurecimiento de los arneses y el branding PNG en una sola rama nacida de `origin/main`,
   ejecutar la pasada local final y publicar un único PR para consumir un solo
   ciclo normal de CI.
2. Después de la integración, comprobar que Cloudflare y los despliegues de
   backend y worker anuncien exactamente el nuevo SHA. No se atribuirán a ese
   lanzamiento las pruebas realizadas antes de publicarlo.
3. Ejecutar sobre ese SHA la matriz visible representativa de formaciones,
   letras, seguimiento y transporte N=1/2/3/4/10. El N=2 físico ya dispone de
   evidencia productiva parcial, pero todavía debe formar parte de la matriz
   consolidada; el N=4 cargado se repetirá como prueba final.
4. Repetir la prueba rápida de transporte iniciada con un clic confiable desde
   React y confirmar límites del observador, aviso, movimiento de toda la flota,
   contribución útil, HLS y limpieza cerrada.
5. Verificar en producción que una fecha sin `Z` muestre unos cinco minutos y
   no unas cuatro horas adicionales. Repetir luego las cuatro áreas visibles y
   una corrida de dos usuarios para comprobar que el cambio no afectó HLS,
   interacción, pantalla completa ni aislamiento.
6. Sanear y versionar las capturas reales de Control, Historial, Plantillas,
   Robots, Grupos, Usuarios, diseño adaptable y carga N=4. Sustituir en el
   informe los marcadores temporales por enlaces y hashes del manifiesto final.
7. Retirar roles, tokens, cuentas, sesiones y perfiles temporales; verificar
   cero recursos residuales y conservar la etiqueta de reversión hasta que el
   nuevo lanzamiento permanezca sano.

Hasta completar estos pasos, «implementado» no debe interpretarse como
«aceptación final de producción aprobada».

## Limitaciones conocidas

- ROS Noetic y Gazebo Classic son dependencias de compatibilidad y ya han
  alcanzado el fin de su vida útil; la migración a ROS 2 y a una versión actual
  de Gazebo constituye un trabajo separado.
- HLS evita UDP/ICE/TURN público, pero añade latencia y envía el contenido
  multimedia a través del proxy del backend.
- Para el modelo Burger actual, el navegador solo permite visualizar la escena.
- La capacidad de sesiones simultáneas debe aceptarse sobre la GPU real de
  producción; el intervalo algorítmico de 1–10 robots no promete diez sesiones
  concurrentes.
- La matriz en vivo toma muestras representativas de cantidades de robots y
  figuras. Esto no implica que todas las combinaciones de figura, trayectoria,
  mundo y parámetros se hayan comprobado exhaustivamente.
- Los robots persistentes y los grupos administrativos son metadatos de
  inventario. No seleccionan las identidades de ejecución exactas dentro de una
  sesión de simulación; la lista del worker es la fuente autoritativa de las
  instancias ROS/Gazebo en vivo.
- `fbef23e` es la base desplegada y aceptada de forma parcial. Las correcciones
  locales posteriores —en especial el analizador UTC, el orden de estados del
  worker y los arneses más estrictos— no están activas en producción y todavía
  necesitan CI y aceptación sobre su propio SHA.
- El PNG histórico es rectangular. Se verificó en las superficies web, pero el
  manifiesto PWA aún necesita derivados cuadrados 192×192 y 512×512 para evitar
  encuadre o recorte al instalar, sin sustituir ni alterar el original.

## Documentación actual

- [Plan de control del enjambre](docs/swarm-control-plan.md)
- [Despliegue del worker GPU](docs/gpu-worker-deployment.md)
- [Publicador del visor de escena](docs/viewer-publisher.md)
- [Arneses de aceptación de producción](scripts/acceptance/README.md)
- [Informe final de comisionamiento en español](docs/informe-comisionamiento-final.md)
