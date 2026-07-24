# Estado de implementación de RobotSwarm

**Fecha del corte:** 2026-07-24

**Alcance:** producción observada en
`ec4980b502dd99f7d8598387d6ff1da4a1229e61`, integrada mediante la PR #113,
y calibración local I-148 todavía sin publicar. Frontend, backend y worker GPU
ejecutan ese corte; CI, backend y GPU aprobaron en los runs `30068986450`,
`30069263190` y `30069309217`. La revisión anterior al trabajo,
`1448a31`, y el archivo
`/home/anyelo/Documents/Codex/backups/2026-07-15-github-robotswarm_backup-20260721.tar.gz`
se conservan para reversión. Este documento no constituye todavía el acta de
cierre.

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
| Frontend React | Controles de sesión, flota y tareas pertenecientes al usuario; estado en vivo; reproductor privado de escena HLS | `ec4980b` publicado por Cloudflare; HLS productivo observado a 31,2 FPS |
| Backend .NET | Autenticación, comprobaciones de propiedad, cola de sesiones, comandos del worker, resultados de tareas, concesiones del visor y concesiones de drenaje de despliegue | `ec4980b` desplegado y `Healthy`; aislamiento y solapamiento API de 6,902 s comprobados en el corte anterior |
| Worker GPU .NET | Ciclo de vida aislado de sesiones Docker, puente de comandos ROS, latidos, selección inmutable de imagen y supervisión del publicador del visor | Release exacto `ec4980b…-30069309217-1` activo; RTX 3080 y reinicios cero comprobados |
| ROS Noetic y Gazebo | Flota dinámica de TurtleBot3 Burger, control de seguridad, orquestación de tareas y tres comportamientos escalables | `ec4980b` solapa corredores; S/N=10 liberó 9/10, pero reveló que el presupuesto histórico era menor que una ruta sana |
| MediaMTX y auxiliar del visor | Pantalla X privada y `gzclient` por concesión, publicación H.264/RTSP y origen HLS interno de baja latencia | Seis inicios HLS postdeploy llegaron a video; los casos aceptados midieron aproximadamente 30 FPS |

I-146 quedó publicada y comprobada sobre `e3dc7ad`: las seis filas abrieron
video HLS y las cinco formaciones aceptadas midieron aproximadamente 30 FPS en
Chrome, 57,5–58,5 FPS en Gazebo y RTF ≥2,9908. La fila S/N=10 fue rechazada
después por ROS, no por el visor. I-147 conserva ese rechazo: ocho robots
avanzaron, dos lotes posteriores permanecieron detenidos y no hubo colisiones.
El primer detector publicado en `917b06b` también fue refutado: 0,01 m por
20 s permitía progreso demasiado lento. El segundo candidato, publicado en
`2193c3d`, exigió 0,10 m/20 s y añadió telemetría, pero dos corridas demostraron
que seguía esperando lotes casi completos. El tercer candidato conserva el
fallback a 0,20 m/20 s y libera de forma concurrente únicamente los corredores
vivos que ya no se cruzan. Tres repeticiones visibles aprobaron con error
≤0,0963 m, RTF ≥2,9887 y cero colisiones.

Las capas utilizan un único ciclo de vida correlacionado en vez de controles
puntuales separados: el frontend invoca al backend autenticado, el backend
emite comandos persistentes para el worker, el worker traduce esos comandos al
espacio de nombres ROS privado y ROS informa el estado de la flota y las tareas
a través del concentrador del worker. El navegador no recibe acceso
administrativo a Docker, ROS, Gazebo, VNC ni MediaMTX.

Los PR #102–#105 llevaron la base hasta `9f49e17`. La PR #106 integró el
marcador visual cinemático, la tolerancia de llegada, el cierre de las carreras
de carga y los controles web restantes. Su squash produjo `1448a31`; CI,
Cloudflare, backend y el despliegue GPU exacto aprobaron. `9f49e17` queda desde
entonces como rollback histórico.

Sobre `1448a31` aprobaron la API multiusuario N=3/N=7, dos Chrome visibles con
visores privados, interacción y pantalla completa, las cuatro anchuras
adaptables, seguimiento N=3/N=6/N=10 y transporte N=2/N=3/N=4/N=10. El
recorrido User y las denegaciones 403 de las rutas Admin también aprobaron. No
se repitió el recorrido Admin completo porque no se dispone de una credencial
administrativa autorizada; no se promovió una cuenta mediante SQL.

La misma aceptación postdeploy halló que las seis formaciones móviles podían
terminar antes del estado activo y que el caso N=1 perdía el informe gráfico al
vencer la concesión del visor. I-136–I-138 trasladan el solver fuera del callback
de odometría, alinean la evasión con la envolvente del Burger y correlacionan
posición/yaw; permiten hasta dos replans en cero únicamente si la escena sigue
estable, las poses son finitas y no existe contacto. I-139 conserva el informe
atestiguado antes de esperar ROS y hace que renovación y limpieza mantengan la
propiedad del lease incluso ante fallos intermedios. I-140 hace observable y
fail-closed cualquier publisher que no acepte la parada local. I-141 distingue
la planificación estacionaria de un heartbeat perdido y calibra la correlación
de pose al asentamiento normal de Gazebo sin omitir la revalidación geométrica.
I-142 elimina tres falsos negativos del comisionamiento web sin debilitar los
gates de concurrencia, video o fullscreen. I-143 separa la liberación segura
de un corredor de la convergencia estricta del slot para evitar un bloqueo
entre lotes. El corte exacto
ya fue revisado de manera independiente, construido como imagen inmutable y
probado en Gazebo visible con N=3 y N=10. La repetición productiva N=1 también
aprobó con el arnés corregido. Este conjunto se publicará en un único PR
correctivo para limitar el gasto de GitHub Actions.

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
masa ni la fricción de la carga de aceptación (0,75 kg y `mu=0.25`), porque ese
perfil conserva la prueba de que un solo Burger no sustituye a la flota. Las
demostraciones rápidas ya disponen de un perfil de práctica de 0,25 kg y
`mu=0.05`.

En formación móvil, el centro permanece inmóvil hasta que todos los robots
alcanzan y estabilizan sus posiciones. El controlador conserva rígida la figura
o letra, valida la huella completa de una órbita circular y, si es necesario,
reduce el radio dentro de límites explícitos. También planifica las rutas de
entrada por lotes para evitar cruces y vuelve a comprobar modelos vivos justo
antes de publicar un lote completo de velocidades. El solver costoso se ejecuta
en un único worker coalescente, nunca dentro del callback de odometría, y cada
tarea requiere una muestra posterior a su inicio. Si posición o yaw cambian
durante el cálculo, el controlador conserva `Twist=0` y permite como máximo dos
replans desde la pose viva cuando la geometría permanece idéntica y libre de
contacto. Un obstáculo dinámico, una pose corrupta, contacto o churn persistente
descartan el plan y fallan cerrados.

La misma frontera se aplica a los datos numéricos. Las poses de odometría con
`x`, `y` o yaw `NaN`/`Inf` se rechazan de forma atómica y no reemplazan la última
muestra válida. Cada ciclo vuelve a validar la flota completa y los
`model_states`, incluso después de entrar en `MOVING`. Si una excepción
inesperada atraviesa el control vivo, el estado pasa a `FAILED`, se cancelan la
asignación y la generación del solver pendientes y se solicita velocidad cero
para toda la flota. Si un publisher no confirma esa parada, el estado enumera
los robots afectados en lugar de afirmar una entrega total.

Los commits `511e47c` y `377a0e3` cerraron cuatro fronteras halladas después de
esa revisión. El modo adaptativo revalida el objetivo ya deformado; lineal y
waypoints conservan un plan geométrico vivo; formación, seguimiento y transporte
validan la flota completa de comandos antes de publicar; y seguimiento actualiza
pose y timestamp de odometría como una sola muestra finita. Las excepciones de
control producen un estado `FAILED` ligado a la tarea, no una parada ambigua.
Transporte aplica además la barrera de lote en `SEARCH`, `APPROACH` y `PUSH`,
invalida por completo un `ModelStates` truncado, comprueba el cuaternión crudo de
odometría y limpia en `finally` el acumulador temporal de comandos. El build ROS
identificado más abajo sigue descartado porque es anterior a estos dos commits.

La revisión continuó en `568979d` y `4450c13`. Seguimiento todavía podía aceptar
una trayectoria si `moving_box` cambiaba mientras `_path_plan_worker` trabajaba;
ahora revalida vuelta y ruta sobre la escena más reciente y correlaciona el
escenario por segunda vez antes del commit. Transporte validaba frescura al
inicio, pero un ciclo inducido de 1,0 s superó el timeout de 0,75 s y publicó
0,12 m/s en `SEARCH`. El gate final comprueba ahora `ModelStates` y la odometría
de toda la flota antes de cualquier lote no nulo, y conserva el lock hasta el
último `Twist` y el commit de evitación. Los comandos de parada siguen permitidos
con datos vencidos. Estas incidencias están cerradas en código, pero la rama aún
no se considera candidata final. La revisión posterior reprodujo I-128–I-134:
un plan de seguimiento anclado antes de mover al líder de `(0; 0)` a `(3; 3)`,
ventanas de frescura que dejaron salir 0,22 m/s y 0,1 m/s, un `ModelStates`
truncado que ocultó `moving_box`, cuaterniones crudos con `Inf`, ejes no
holonómicos y nombres duplicados ambiguos. `07da8f4` cerró ese grupo histórico con doble
correlación de la cadena, gates finales de frescura, snapshots persistentes y
unívocos, filtro por mundo activo, validación cruda de cuaterniones y rechazo de
ejes incompatibles con el Burger. Ese conjunto, incluido I-135, quedó integrado
en `1448a31`. El delta local vigente es I-136–I-143 y todavía requiere PR, CI,
despliegue y aceptación visible sobre su propio SHA.

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
- **Rutas heredadas de tareas:** `/apps/configs/task` redirige a
  `/apps/GTS/task-templates` para administradores y
  `/apps/dashboard/tasks` redirige a `/apps/GTS/taskLogs` para usuarios. De este
  modo ya no se montan las pantallas que consultaban `/api/TaskActivity` y
  `/api/TaskForm/widget`, endpoints que no pertenecen al plano de control real.
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
tenía una corrida física en el corte histórico; N=2 aprobó después la
aceptación física postdeploy sobre `1448a31`.

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
que la normaliza a UTC quedó integrado por el PR #105 y forma parte de
`9f49e17` y de `1448a31`. El recorrido postdeploy comprobó nuevamente el bundle;
la captura específica del contador todavía no se ha seleccionado para el acta.

La primera prueba rápida endurecida de transporte encontró además una carrera
real del worker: el sondeador alcanzó a persistir `Running` dos segundos antes
de que la ruta de ejecución del comando informara `Accepted`. El backend rechazó
correctamente la regresión posterior. El arreglo integrado en `9f49e17`
mantiene `Accepted`
después de una
publicación ROS satisfactoria y suspende la recuperación y el sondeo, antes y
después de la lectura, mientras el mismo `StartTask` siga vivo. Las regresiones
concurrentes cubren el descubrimiento, una marca de progreso previa y los fallos
de publicación o reporte; el worker aprobó 129/129. El intento productivo se
conserva como rechazado; la secuencia se volverá a observar sobre el próximo
SHA para que la aceptación final pertenezca a un solo artefacto.

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
navegadores volvió a verificar interacción, pantalla completa, cierre y parada
independiente sobre `1448a31`. El cierre de una concesión no detuvo la tarea ROS
de la otra cuenta.

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

Las afirmaciones siguientes corresponden a pruebas ya realizadas. Se distingue
la evidencia histórica de `fbef23e`/`9f49e17`, la aceptación postdeploy de
`1448a31` y el corte local de I-136–I-143. Ningún resultado local se atribuye al
próximo SHA antes de desplegarlo:

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
- La aceptación postdeploy de `1448a31` aprobó con dos cuentas, listas de flota
  N=3/N=7, aislamiento, tareas y limpieza. Dos ventanas Chrome visibles
  reprodujeron flujos independientes a unos 30 FPS; la tarea A terminó, la B
  continuó al detener A, y se ejercitaron interacción y pantalla completa. El
  recorrido User y las denegaciones Admin aprobaron; no se ejecutó un recorrido
  Admin completo sin una credencial autorizada.
- El recorrido adaptable de solo lectura aprobó nuevamente sobre `1448a31` en 360, 768,
  1366 y 1920 px: el panel quedó visible y dentro del área visible, sin
  desbordamiento horizontal. Como antecedente, el
  [reporte adaptable versionado](docs/assets/commissioning-2026-07/final-fbef23e/responsive-reporte.json),
  que corresponde únicamente al corte anterior `fbef23e`, tiene SHA-256
  `4e587e4d20d91602daf962f4f43e74b92017a4b3c537907517e06e5914739a74`.
- La matriz postdeploy de `1448a31` aprobó seguimiento N=3/N=6/N=10 y
  transporte N=2/N=3/N=4/N=10. N=10 transportó 0,5036 m, terminó a 0,4964 m,
  mantuvo RTF 2,9672, registró 10/10 robots útiles, búsqueda y aviso completos y
  cero colisiones. Las seis formaciones terminaron antes del gate activo y N=1
  perdió su informe después del TTL; ambos resultados se rechazaron y abrieron
  I-136–I-139.
- El endurecimiento integrado en `9f49e17` separa el protocolo oficial y los
  marcadores vivos en canales distintos, limita líneas y evidencia, usa una
  escritura atómica por marcador y prueba la desaparición de cada observador y
  grupo de procesos. La sonda gráfica liga hash, PID y tick al `gzclient` vivo;
  la limpieza activa usa `pidfd`; el monitor N=4 conserva el estado real de
  `wait`; y la prueba del clic vive fuera del mundo JavaScript de React. También
  evita APIs ausentes en Python 3.8 (`Popen(umask=)` y `str.removesuffix`) y
  agrega diagnóstico explícito de MediaSource, SourceBuffer, AVC/H.264, estado
  del video y solicitudes HLS. La rama correctiva posterior conserva esas
  garantías y suma los arreglos físicos descritos en este corte.
- El corte local vigente de I-136–I-143 aprobó 625/625 pruebas ROS y 253/253
  contratos de aceptación. Dentro del total ROS, las suites focales terminaron
  formación 103/103, ciclo de vida 235/235 y seguimiento 47/47. También aprobaron
  la compilación Python 3.8 y `git diff --check`. La revisión independiente
  cerró sin hallazgos P0 ni P1. Las regresiones del lease cubren la transición
  `closing` → hueco DOM → botón, fallos de HLS posteriores a la renovación,
  binding temporalmente desconocido y limpieza fail-closed. Estas cifras
  describen el árbol local, no un despliegue.
- La imagen local posterior a I-141, ID
  `sha256:3394046cede94cd3855cf8b834546176f48453369b93b0879ad13f75d39b5f48`,
  aprobó la formación S N=10 visible durante 75,0004 s activos: 10/10
  asignaciones, error independiente máximo 0,0952 m, aceleración filtrada
  máxima 0,7039 m/s², separación mínima 0,4025 m, RTF 2,9851 y cero
  colisiones. La secuencia negativa y el resultado final se conservan en la
  [evidencia saneada de I-141](docs/assets/commissioning-2026-07/corrective-i141/README.md).
- El recorrido público final de I-142 aprobó dos Chrome visibles, visores
  privados, entrada real, fullscreen, tareas concurrentes, cierre/reapertura
  del visor B y continuidad de B después de detener A. Los streams midieron
  aproximadamente 30 FPS y cero drops; responsive aprobó cuatro anchuras. La
  [secuencia saneada](docs/assets/commissioning-2026-07/corrective-i142/README.md)
  conserva los intentos rechazados y los hashes de los tres aprobados.
- La reproducción final encontró I-143 antes de publicar: un robot dentro de
  la histéresis segura de 0,14 m podía retener los corredores siguientes por no
  haber cruzado aún la tolerancia estricta de 0,09 m. La imagen exacta
  `sha256:4caa2ea97dc55e3f0e4929568e255fcbea0d6969318522e34f889a6df72691e9`
  aprobó después S/N=10 con error máximo 0,0936 m, RTF 2,9912 y cero
  colisiones. La suite completa aprobó 625/625 en 117,379 s. La
  [evidencia I-143](docs/assets/commissioning-2026-07/corrective-i143/README.md)
  conserva el diagnóstico antes/después.
- La primera matriz visible posterior a PR #107 aprobó 3/6 formaciones y fue
  rechazada. I-144 amplía únicamente los presupuestos de cuadrado/V/diamante a
  90 s y calibra el yaw de asentamiento a 0,15 rad. La imagen local exacta
  aprobó las tres filas en 62,64/78,43/86,34 s, con errores
  0,0945/0,0937/0,0960 m, RTF ≥2,90 y cero colisiones. La
  [evidencia I-144](docs/assets/commissioning-2026-07/corrective-i144/README.md)
  distingue el 3/6 productivo, el segfault ambiental y los aprobados locales.
  El freeze del delta aprobó 626/626 pruebas ROS y 253/253 contratos.
- I-145 registra los runs GPU `30054706834` y `30054818947`: ambos agotaron
  5 s al resolver el backend antes de adquirir el lease. El release `1d497d4`
  siguió activo. La adquisición ahora reintenta únicamente errores de
  transporte hasta tres veces, con pausa de 5 s; los HTTP distintos de 200
  continúan fallando de inmediato. La
  [evidencia I-145](docs/assets/commissioning-2026-07/corrective-i145/README.md)
  conserva diagnóstico y contención.
- La PR #109 publicó I-145 y el run `30055847809` dejó activo el release
  exacto `2445a37`. La PR #110 publicó I-146 como `e3dc7ad`; CI, backend y GPU
  aprobaron. Seis filas postdeploy alcanzaron video HLS y los casos aceptados
  midieron aproximadamente 30 FPS. Frontend aprobó 164/164, build y 37/37
  focales; el arnés aprobó 74/74. La
  [evidencia I-146](docs/assets/commissioning-2026-07/corrective-i146/README.md)
  distingue el diagnóstico previo de la comprobación postdeploy.
- I-147 conserva resultados que no deben mezclarse: la primera S/N=10 de
  `e3dc7ad` quedó en `forming`, sin colisiones, con dos robots todavía
  retenidos; una repetición fresca sí aprobó 75,0317 s activos, error 0,0951 m
  y RTF 2,9846. El primer candidato `917b06b` volvió a fallar porque 0,01 m
  cada 20 s admitía una deriva microscópica; su reporte conserva RTF 2,9855 y
  cero colisiones. El segundo candidato `2193c3d` volvió a fallar dos veces
  con seis lotes, un replan, RTF ≥2,9908 y cero colisiones. El tercer candidato
  reconstruye los corredores restantes y solapa lotes cuando dejan de
  cruzarse; tres S/N=10 locales aprobaron 75 s activos con error ≤0,0963 m,
  RTF ≥2,9887 y cero colisiones. ROS aprobó 631/631 y los contratos 254/254.
  El gate API con
  letra A aprobó 6,902 s de solapamiento y dos Chrome visibles aprobaron a
  30,089/30,069 FPS con cero drops. La
  [evidencia I-147](docs/assets/commissioning-2026-07/corrective-i147/README.md)
  conserva hashes, diagnóstico y el primer intento local descartado.
- La PR #113 publicó el tercer candidato como `ec4980b`. Su primera S/N=10
  productiva llegó al quinto de seis lotes con nueve robots liberados, progreso
  sano, RTF 2,9605, HLS 31,2 FPS y cero colisiones. Terminó a 252,816 s
  simulados porque el arnés todavía suponía el borde histórico de 229,5 s.
  I-148 eleva solo la envolvente S/N=10 a 120 s de pared (324 s al piso 2,7);
  precisión, ventana activa, aceleración y seguridad no cambian.
- La imagen exacta del delta, sin montajes de fuentes, tiene ID
  `sha256:6f1af927d149c3be17115d50c6f2785a5fa4e91cc67ff2ff95bac86ee9844cb5`.
  En ventanas Gazebo no headless, el triángulo N=3 y la S N=10 conservaron el
  estado `moving` durante 75 s y finalizaron con error independiente máximo de
  0,0921/0,0974 m; el comportamiento informó 0,0925/0,0981 m. El RTF del
  algoritmo fue 2,9965/2,9875 y no hubo colisiones. Las sondas concurrentes
  midieron 58,493/57,507 FPS y RTF 2,996/2,984 sobre D3D12/NVIDIA. El primer
  intento `llvmpipe` a 6,03 FPS se rechazó y se repitió después de entregar
  correctamente `/dev/dxg` y las bibliotecas WSL D3D12 al contenedor.
- El transporte productivo N=1 de `1448a31`, repetido con I-139 en el arnés,
  terminó `DONE`, desplazó la caja 0,5013 m, mantuvo RTF 2,9962, utilidad 1/1 y
  cero colisiones. La sonda activa rindió 58,711 FPS y el HLS 30,164 FPS; la
  limpieza retiró sesión, lease, contenedor, red, perfil y procesos. La corrida
  no agotó el lease, por lo que prueba la preservación temprana del reporte,
  no una renovación física; esa rama permanece cubierta por contratos.
- El freeze provisional anterior a I-117–I-125, integrado hasta `e18926a`, aprobó ROS
  461/461, frontend 164/164 en 30 suites, backend 253/253 con 8 casos
  PostgreSQL opt-in omitidos por diseño y los siete arneses de aceptación
  241/241 (17+49+13+61+44+41+16). El cierre de formación aprobó además 6/6
  pruebas focales, 50/50 de formación/rutas/live y 1/1 de evitación. El cierre
  de apagado cargado aprobó 81/81 pruebas loaded, 170/170 ROS/mundo, 16/16
  frontend focales y 53/53 backend focales. Worker aprobó 129/129, PostgreSQL 17
  opt-in 8/8, el lint focal, el build de producción del frontend, el publicador
  del visor y las pruebas de despliegue/rollback GPU quedaron en verde. El
  primer build frontend terminó en `EACCES` por residuos de `build/` con otro
  propietario; al corregir únicamente la propiedad del directorio ignorado, la
  misma compilación aprobó, por lo que se clasificó como fallo ambiental. Un
  build intermedio etiquetado `robotswarm-ros:local-final-candidate` completó
  catkin al 100 %: ID
  `sha256:db8ffda30d79e5e22e1bbfe66978faedb118bb9c43b3e25dedd161807833be14`,
  4.231.139.487 bytes, creado el 2026-07-22T03:10:23Z. Una revisión posterior
  encontró fronteras P1 todavía abiertas, por lo que ese ID quedó descartado
  como candidato final.
- Después, `511e47c` aprobó 99/99 pruebas focales de formación y 469/469 en su
  ejecución ROS aislada. `377a0e3` aprobó 206/206 de seguimiento+lifecycle y
  473/473 en la ejecución global aislada. También aprobaron la compatibilidad
  sintáctica con Python 3.8 y `git diff --check`. Estas cifras cierran las
  regresiones de I-117–I-125, pero todavía no son una
  suite combinada final de todas las capas. En ese punto aún faltaban la imagen
  ROS y los builds posteriores, cerrados más abajo; CI, despliegue y aceptación
  visible permanecen pendientes. No se utilizó GitHub Actions para estas comprobaciones: el CI del
  único PR seguirá siendo la fuente autoritativa. Los resultados corresponden
  al árbol local y no se atribuyen a `9f49e17` ni al futuro SHA de despliegue.
- I-126 se verificó primero en el worktree aislado de `bd85755` y quedó
  integrada como `568979d`: aprobó 40/40 pruebas de seguimiento y 483/483
  globales, además de Python 3.8 y `git diff --check`. I-127 quedó integrada
  como `4450c13`; su corte aislado final aprobó 172/172 pruebas de lifecycle y
  487/487 globales, además de `py_compile` con Python 3.8 y
  `git diff --check`. La regresión hace vencer el reloj entre el primer y el
  segundo robot y comprueba una decisión atómica, sin pulso parcial. Estos
  recuentos permanecen separados hasta la repetición exacta descrita abajo.
- La primera iteración de I-128–I-132 aprobó 95/95 pruebas de
  seguimiento+formación. La revisión independiente añadió I-133 e I-134 antes
  de congelar el paquete. El commit integrado `07da8f4` aprobó después 6/6
  regresiones focales finales y 497/497 de la suite global aislada en 109,067 s,
  además de `py_compile` con Python 3.8.10 para los tres controladores y sus tres
  archivos de prueba, y `git diff --check`. No se presenta el 95/95 como corte
  final; el freeze exacto se registra en el punto siguiente.
- La repetición exacta sobre el árbol principal `07da8f4`, con la documentación
  y el ajuste de mundo todavía sin commit, aprobó 497/497 pruebas ROS en
  109,460 s y terminó con RC=0. Este corte precede al ajuste de cámara y se
  conserva para trazabilidad. También aprobaron 241/241 contratos de
  aceptación (17+49+13+61+44+41+16), 8/8 pruebas de física del mundo, Python
  3.8.10, `git diff --check`, tres workflows YAML, las variantes Compose
  producción/backend/hero/raíz y el escaneo de secretos. La única coincidencia
  fue la URL deliberadamente inválida del fixture negativo en
  `WorkerOptionsValidatorTests.cs:44`; no contiene un secreto.
- La primera imagen `robotswarm-ros:local-final-safe`, ID
  `sha256:02fbd5c2302d3af0eb9e543af04bb4507292786b7155b39b54e9d294b27f4864`,
  completó catkin y permitió las pruebas funcionales N=4. La inspección de sus
  capturas reveló que la cámara predeterminada recortaba el marcador cerca de
  `y=-4`; por ello quedó supersedida solamente respecto del encuadre visual, no
  por un fallo del algoritmo ni de la sincronización del marcador.
- La imagen reconstruida después de corregir la cámara conserva la etiqueta
  `robotswarm-ros:local-final-safe`, completó catkin al 100 % y tiene ID
  `sha256:e17579ed83e0c37a9ff9b03817652aeb935573b307801ddc5863d29f2a92ae0d`,
  tamaño 4.231.381.706 bytes y fecha
  `2026-07-22T04:53:51.679721236Z`. Fue el artefacto local exacto previo a PR
  #106; se conserva como antecedente y quedó supersedido por el despliegue
  `1448a31` y por la imagen correctiva `6f1af927…4cb5`.
- La suite ROS completa se repitió después del ajuste de cámara y de este
  rebuild: aprobó 497/497 pruebas en 109,592 s, con `OK` y RC=0. Esta es la
  repetición local exacta histórica de PR #106; sustituyó en su etapa al corte
  de 109,460 s. El freeze vigente posterior es 576/576 y no borra su valor de
  trazabilidad pre-cámara.
- Sobre la imagen anterior al ajuste de cámara se ejecutaron dos aceptaciones
  locales visibles de transporte N=4, ambas con RC=0. El perfil legado, que
  omite el parámetro y conserva tolerancia 0,50 m, desplazó el objeto 0,5022 m,
  terminó a 0,4978 m de la meta, mantuvo RTF 2,9965, registró 4/4 robots útiles
  y cero colisiones. La segunda corrida inyectó tolerancia 0,25 m solamente en
  memoria dentro del arnés para reproducir el valor de la interfaz, sin
  modificar el repositorio: avanzó 0,7535 m, terminó a 0,2465 m, mantuvo RTF
  2,9963 y registró 4/4 robots útiles, fracción simultánea 0,8707, velocidad
  máxima 0,1664 m/s, aceleración máxima en ventana 0,6281 m/s² y cero
  colisiones. `tb3_1` encontró la carga y avisó a `tb3_0`, `tb3_2` y `tb3_3`;
  hubo picos de cuatro buscadores y cuatro robots en reunión, y el marcador se
  publicó y sincronizó en `x=-3.5`, `y=-4.0`.
- La evidencia temporal de esas corridas está en
  `/tmp/robotswarm-local-final-visible-20260722T0444Z/`. Las capturas
  `approach`, `search`, `done` y `zoomout2` tienen respectivamente SHA-256
  `fa356ca8b084ee62e093f5993b1134a5b65ac2a857e492096e1906a0677c19d6`,
  `d27774ed3560056919728b83dce8386a28187ba4f0e9d7234ac8959e706aff7b`,
  `044573e7ca9a43d347947e51e2db6560e364006b737f4242f7c49c445ca049b4` y
  `9e88ce8444111379ce740c2c7ae4fb01e20c516dcc585f5da4c4c12fe6e1e6c7`.
  La sonda simultánea identificó D3D12/NVIDIA RTX 3080, 57,182 FPS de cámara,
  58,795 eventos de posrenderizado por segundo y RTF 2,997, con viewport
  990×334.
- I-135 registró el recorte, cuya causa era la pose de cámara
  `0 -12 10 0 0.7 1.5708`. Se amplió el encuadre a
  `0 -14.4 12 0 0.72 1.5708` y se agregó una guarda al test del mundo. La imagen
  reconstruida aprobó una sonda visible exacta con D3D12/NVIDIA RTX 3080,
  57,279 FPS de cámara, 58,791 eventos de posrenderizado por segundo, RTF 2,997
  y viewport 990×334. El modelo `target_marker` se movió mediante
  `SetModelState` a las cuatro esquinas `(4,4)`, `(-4,4)`, `(4,-4)` y
  `(-4,-4)`; huella y caja quedaron completas y dentro de los muros en las
  cuatro capturas. Aprobaron además 8/8 pruebas del mundo y
  `git diff --check`, por lo que I-135 queda cerrada localmente.
- La evidencia temporal de I-135 está en
  `/tmp/robotswarm-local-final-camera-20260722T0454Z/`. Los SHA-256 de las
  capturas NE, NW, SE y SW son, respectivamente,
  `75c53cf725a69c20568d00a529754d463bb79957c2e10e904d4b46e2c22ee839`,
  `84750bdbb15eddb1babaeffedf10205de84a816fd6e474e13a28f35a887152ca`,
  `ade73f6e43125f0945a60606d3db7cb6e10538abc19b2d3fb18e1ead1b848e14` y
  `86f32de8ad0628c0fae1d97b53a1ae3ed9dcf86f73fa753ec5c89a886ea48f51`.
  Esta comprobación es local y no productiva; no sustituye el recorrido visible
  posterior al despliegue.
- La repetición física visible exacta sobre la imagen reconstruida `e17579ed…ae0d`
  y con tolerancia 0,25 m inyectada solo en memoria también terminó `PASS`,
  RC=0, sin borrar la flota durante el caso. El objeto avanzó 0,7523 m y quedó
  a 0,2477 m de la meta; RTF fue 2,9962, los 4/4 robots resultaron útiles y la
  fracción de contribución simultánea fue 0,9355. Hubo picos de cuatro robots
  tanto en búsqueda como en reunión; `tb3_1` avisó a `tb3_0`, `tb3_2` y
  `tb3_3`; no hubo colisiones. La velocidad máxima fue 0,1649 m/s y la
  aceleración máxima en ventana, 0,7883 m/s², inferior al límite 1,0 m/s².
  Las transiciones ocurrieron, en tiempo de pared, a 0,278 s (`SEARCH`),
  39,191 s (`APPROACH`), 107,725 s (`PUSH`) y 131,991 s (`DONE`); el marcador
  quedó sincronizado en `x=-3.5`, `y=-4.0`.
- Dentro de
  `/tmp/robotswarm-local-final-camera-20260722T0454Z/`, la única captura de esa
  corrida aceptada como evidencia visual de fase es `final-n4-search.png`,
  SHA-256
  `d036d9814b893441f2c57be52a4ab58415339f85bfeba6055177820417211f11`:
  muestra la carga, el marcador completo y los cuatro robots buscando. El log
  saneado `transport-n4-tolerance-025-final.log` tiene SHA-256
  `3ad8ad2b65b7589f34d52f716990d8675e57cef9474d9679af9407a81612ac43`.
  Las capturas `final-n4-push.png` y `final-n4-done.png` se excluyen: existe una
  carrera entre la captura y el `cleanup/reset`, y por tanto no prueban ni la
  fase `PUSH` ni la pose final. Esas propiedades se sostienen con el log
  correlacionado, no con las dos imágenes descartadas.
- El preflight local visible que validó primero el cambio cinemático, antes de
  localizar I-135, identificó
  `D3D12 (NVIDIA GeForce RTX 3080)`, midió 58,206 FPS de cámara, 62,512 eventos
  de posrenderizado por segundo y RTF 2,996, con viewport 990×351. El reporte
  `/tmp/robotswarm-corrected-ghost-gui-report.json` tiene permisos `0600` y
  SHA-256
  `98a4651069f1ea8199d26d278da0b7a4df273b54b93ffaa0a9b65bfd12e1f5d5`.
  Esta medición acredita el render local acelerado y el movimiento del modelo,
  pero no el encuadre completo ni una aceptación postdeploy.
- Las capturas de escritorio Windows/WSLg obtenidas inicialmente con
  `CopyFromScreen` quedaron congeladas o recortadas y se rechazaron. Las sondas
  posteriores produjeron capturas limpias mediante el cliente vivo. La
  comparación antes/después del marcador y la búsqueda N=4 quedaron saneadas,
  versionadas y verificadas por SHA-256 bajo
  `docs/assets/commissioning-2026-07/final-1448a31/`. Otras superficies web y
  la carga N=4 correctiva siguen pendientes de evidencia final.
- Después del endurecimiento se comprobaron los puertos de mantenimiento de
  producción: VNC y websockify escuchan en la interfaz de bucle local. Los
  controles de acceso del visor permanecieron desactivados durante la línea
  base inicial y después se habilitaron juntos para la ventana de
  comisionamiento supervisada.
- Las páginas User y el ciclo `StopViewer` fueron recorridos nuevamente sobre
  `1448a31`. Las rutas Admin rechazaron correctamente al usuario normal; el
  recorrido Admin completo y sus capturas quedan pendientes de una credencial
  autorizada.

Las mediciones sin procesar, capturas de pantalla, análisis de incidentes y la
distinción entre corridas negativas y aceptadas se registran en el
[informe de comisionamiento](docs/informe-comisionamiento-final.md).

## Trabajo pendiente antes de declarar completo el lanzamiento

1. Publicar un único PR correctivo y consumir un solo ciclo normal de CI.
2. Integrar únicamente con CI verde y desplegar una sola vez la imagen del SHA
   de merge exacto, conservando la etiqueta de rollback.
3. Repetir sobre ese SHA las seis formaciones, transporte N=1, el smoke React
   N=4 con tolerancia 0,25 m y la carga N=4 de 0,75 kg con `mu=mu2=0.25`.
4. Confirmar HLS, interacción, marcador, RTF, seguridad, cierre del visor y
   limpieza final. El recorrido Admin completo solo se hará con una credencial
   autorizada; no se alterarán roles directamente en la base de datos.
5. Retirar cuentas, sesiones, perfiles y recursos temporales creados por las
   pruebas y verificar que no queden leases, contenedores ni procesos
   residuales.

Hasta completar estos pasos, «implementado» no debe interpretarse como
«aceptación final de producción aprobada».

## Limitaciones conocidas

- ROS Noetic y Gazebo Classic son dependencias de compatibilidad y ya han
  alcanzado el fin de su vida útil; la migración a ROS 2 y a una versión actual
  de Gazebo constituye un trabajo separado.
- HLS evita UDP/ICE/TURN público, pero añade latencia y envía el contenido
  multimedia a través del proxy del backend.
- Para el modelo Burger actual, la capacidad de video es `Scene`, no
  `RobotCamera`; el visor `Scene` sí permite interacción autorizada con el
  escritorio privado de Gazebo.
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
- `1448a31` es la base desplegada. El marcador y la tolerancia ya están activos;
  las correcciones locales I-136–I-143 todavía necesitan CI, despliegue y
  aceptación sobre su propio SHA.
- El grafo ROS se considera privado y confiable. La lista pendiente de ACK queda
  acotada, pero un proceso ya comprometido dentro del mismo contenedor podría
  publicar identificadores siempre distintos mientras el socket inverso está
  bloqueado y acumular callbacks en su lane. Es un riesgo P2 de disponibilidad
  interno, sin ruta desde el navegador; una defensa adicional sería coalescer
  globalmente el ACK en un único job.
- El PNG histórico es rectangular. Se verificó en las superficies web, pero el
  manifiesto PWA aún necesita derivados cuadrados 192×192 y 512×512 para evitar
  encuadre o recorte al instalar, sin sustituir ni alterar el original.

## Documentación actual

- [Plan de control del enjambre](docs/swarm-control-plan.md)
- [Despliegue del worker GPU](docs/gpu-worker-deployment.md)
- [Publicador del visor de escena](docs/viewer-publisher.md)
- [Arneses de aceptación de producción](scripts/acceptance/README.md)
- [Informe final de comisionamiento en español](docs/informe-comisionamiento-final.md)
