# Aceptación multiusuario de producción

Estos siete arneses comprueban el flujo público de RobotSwarm sin reutilizar una
sesión histórica. `robotswarm-prod-e2e.py` valida la API, el aislamiento HLS y
la limpieza de dos cuentas. `robotswarm-visible-e2e.py` repite el recorrido en
dos ventanas normales de Chrome, con GPU habilitada, interacción real y
capturas sanitizadas. `robotswarm-sections-e2e.py` recorre las pantallas según
el rol y `robotswarm-responsive-e2e.py` captura Control a 360, 768, 1366 y
1920 píxeles CSS sin crear sesiones ni tareas. La matriz ROS, el ensayo N=4 con
carga y el smoke de transporte desde la interfaz completan los siete recorridos.
Los arneses visibles no admiten modo headless y fijan el origen de las
credenciales a `https://rs.zerav.la`.

La ejecución offline vigente aprobó 253/253 contratos: API de producción 17,
carga N=4 49, responsive 13, matriz ROS 73, secciones 16, transporte desde la
interfaz 44 y visor multiusuario 41. Este conteo corresponde a las pruebas
unitarias de los siete instrumentos; no equivale a haber aprobado sus recorridos
contra un release desplegado. La compuerta N=4 aprobó sobre la base `fbef23e`,
pero debe repetirse sobre el SHA final. De la matriz normal quedan por repetir
las seis formaciones corregidas; los cinco tamaños de transporte ya tienen un
resultado sobre `1448a31`, incluido N=1 con I-139 en el arnés.
Los resultados de suites de aplicación, CI y postdeploy deben citarse con el SHA
y el reporte de la ejecución que los produjo, no inferirse a partir de estos 253
contratos.

## Archivos locales requeridos

Las credenciales se leen, sin ejecutar el archivo como código, desde
`/tmp/robotswarm-e2e-credentials.env`. Debe pertenecer al usuario, tener modo
`0600` y contener exactamente los marcadores, correos y contraseñas de las dos
cuentas de prueba mediante las claves `TEST_A_ID`, `TEST_A_EMAIL`,
`TEST_A_PASSWORD`, `TEST_B_ID`, `TEST_B_EMAIL` y `TEST_B_PASSWORD`.

El enlace entre ambos ensayos utiliza `/tmp/robotswarm-e2e-binding.key`, un
archivo regular de 32 bytes, también `0600`. Es una clave aleatoria independiente
de las contraseñas y nunca se incorpora al reporte. Puede generarse una sola vez
para la pareja de ejecuciones con:

```bash
umask 077
openssl rand -out /tmp/robotswarm-e2e-binding.key 32
```

El operador debe comprobar primero en los hosts que backend y worker GPU
corresponden al mismo SHA completo. Ese valor se pasa explícitamente a los dos
programas; no se acepta una revisión abreviada.

## Orden de ejecución

La planificación comprueba permisos y esquema sin acceder a la red:

```bash
python3 scripts/acceptance/robotswarm-prod-e2e.py --plan
```

Después de autorizar la prueba de producción, se ejecuta la parte API y se
conserva su salida en un directorio del propietario:

```bash
python3 scripts/acceptance/robotswarm-prod-e2e.py \
  --execute-production \
  --deployment-commit <sha-completo> \
  --output /tmp/robotswarm-acceptance/api.json
```

El reporte aprobado tiene una vigencia de treinta minutos para iniciar la parte
visual. En WSL, `--profile-root` debe apuntar a un directorio temporal de Windows
que pertenezca al operador:

```bash
python3 scripts/acceptance/robotswarm-visible-e2e.py \
  --api-report /tmp/robotswarm-acceptance/api.json \
  --deployment-commit <sha-completo> \
  --profile-root "/mnt/c/Users/<usuario>/AppData/Local/Temp" \
  --output-dir /tmp/robotswarm-acceptance/visual
```

La prueba visual exige dos rosters distintos —tres y siete robots por defecto—,
HLS privado en ambas cuentas, FPS numéricos, clic, arrastre, rueda, teclado,
pantalla completa, tareas concurrentes y continuidad de B después de detener A.
Todos los JSON y PNG se crean con modo `0600`; los perfiles de Chrome, sesiones
y controles interactivos se liberan incluso ante interrupción. Un resultado no
se considera aprobatorio si la limpieza falla.

Antes de solicitar un visor, el recorrido multiusuario y la compuerta N=4
inspeccionan en Chrome las Media Source Extensions (MSE), `SourceBuffer` y el
soporte H.264 que utiliza `hls.js`; esta comprobación no crea un lease. Así se
distingue una incapacidad del navegador de un fallo posterior de provisión o
reproducción HLS. Si el primer frame no llega en la compuerta N=4, el diagnóstico
acotado conserva únicamente señales como estado MSE, número de recursos HLS,
estado visible de la interfaz y dimensiones del elemento `video`, sin retener
URL firmada, token ni identificador de lease.

Los controles HTML se someten primero a hit-test y a un clic CDP físico. El
resultado registra si Chrome emitió un evento `isTrusted`; si un control
ordinario no lo recibe, se permite un único `element.click()` como fallback y se
deja constancia explícita mediante `fallbackUsed`. Las acciones que requieren
activación de usuario —inicio de tarea y pantalla completa— exigen el clic
trusted y fallan de forma cerrada: nunca utilizan ese fallback. Los gestos sobre
Gazebo (ratón, rueda y teclado) continúan enviándose como eventos de entrada
reales y no como llamadas DOM.

Al salir de fullscreen, el visor debe emitir una sola liberación global de
entradas retenidas. `Escape` pertenece al navegador en esa transición y no debe
llegar a Gazebo; el frontend consume el auto-repeat mediante `event.repeat` y
permite un nuevo Escape después de volver a fullscreen, aunque el `keyup`
anterior no se haya recibido. La regresión frontend correspondiente fija esta
condición de forma explícita.

Los programas muestran únicamente mensajes y reportes saneados. No deben
copiarse a la documentación el archivo de credenciales, la clave de enlace, los
perfiles del navegador ni las respuestas HTTP crudas.

Si el POST de una sesión termina con un resultado de red incierto, el arnés no
supone que hubo rollback. Durante `--cleanup-timeout` vuelve a listar la cuenta
dedicada, recupera todas sus sesiones activas —el preflight ya exigió que no
existieran ocupantes previos— y las detiene. Una falta total de observabilidad
o una parada no confirmada hace
fallar el cleanup; el proceso devuelve código 3 cuando el ensayo ya había
fallado y tampoco pudo demostrar la limpieza.

Como precondición del recorrido, un principal JWT viejo no puede abrir ni
controlar una sesión después de modificar la cuenta. El backend adquiere el
advisory lock compartido y vuelve a comprobar estado, rol y `account_version`;
el `SELECT ... FOR SHARE` hace que un snapshot `SERIALIZABLE` fijado antes de la
espera termine en `40001` y se repita desde una transacción nueva. La
reproducción controlada termina en 401 y cero sesiones. El `DELETE` de sesión
declara también en Swagger/OpenAPI su posible 409 cuando agota los reintentos de
serialización; un instrumento no debe tratar ese estado como un 404 ni como una
limpieza confirmada.

Los timestamps de .NET se validan antes de comparar intervalos o caducidades.
Los parsers admiten de uno a siete dígitos fraccionarios, normalizan a los seis
microsegundos que acepta Python 3.8 y conservan `Z` o el desplazamiento UTC. En
las columnas históricas `timestamp without time zone`, la ausencia de sufijo se
interpreta de forma explícita como UTC. Esto evita tanto el error de precisión
de `datetime.fromisoformat` en Python 3.8 como comparaciones entre fechas aware y
naive, sin aceptar texto o zonas horarias arbitrarias.

La misma regla alcanza las operaciones Admin sobre otra cuenta. Si una petición
ya pasó el middleware pero su actor queda revocado mientras espera, el handler
toma el lock global cuando corresponde, ordena por ID el lock compartido del
actor y el exclusivo del objetivo, y vuelve a validar al actor después de
adquirirlos. El resultado exigido es 401 con la cuenta objetivo intacta; una
respuesta exitosa obtenida solo por haber autorizado antes de la espera invalida
la prueba.

Create, PUT y PATCH aplican además el mismo validador del backend; la política no
depende del formulario React. La contraseña admite de 8 a 16 caracteres y exige
minúscula, dígito y un símbolo de `!@#$%^&*`. El correo tiene longitud máxima 254
y una forma ASCII cerrada. Su canonicalización elimina únicamente espacio,
tabulador, LF, VT, FF y CR en los extremos y convierte a minúsculas. La misma
regla se materializa en la columna generada `NormalizedEmail`, su índice único y
el `CHECK` de PostgreSQL. Las mutaciones respetan el orden conjunto Admin →
correo → cuentas y una carrera de unicidad devuelve el error controlado de
correo en uso.

La migración aborta antes del índice si detecta formatos históricos no soportados
o duplicados canónicos; no elige una cuenta ganadora. El
[preflight agregado de solo lectura](../../docs/assets/commissioning-2026-07/account-preflight-readonly-20260720.txt)
no incluye PII. Corresponde a la base existente, no a una aplicación postdeploy
de la migración.

## Recorrido de las demás secciones

`robotswarm-sections-e2e.py` es independiente de la prueba ROS y abre una sola
ventana normal de Chrome. Comprueba Historial con una cuenta User; con una
cuenta Admin recorre además Plantillas, Robots, Grupos y Usuarios. También abre
y cancela los diálogos disponibles. La ejecución Admin crea un grupo sin robots
con un nombre efímero y lo elimina en `finally`; un fallo de esa limpieza hace
fallar la aceptación.

Una auditoría encontró que la primera versión del arnés conservaba rótulos
anteriores. Los marcadores vigentes son exactamente Plantillas, Historial,
Control, Robots, Grupos y Usuarios. El contrato no mantiene una segunda copia a
ciegas: lee `SwarmFrontend/src/app/fuse-configs/navigationGTSConfig.js` y comprueba
en esa configuración real el nombre, la ruta y el rol de las seis entradas.

Cada rol se prueba por separado. Por defecto se reutiliza de forma local la
cuenta A de `/tmp/robotswarm-e2e-credentials.env`, regular, propiedad del
operador y con modo `0600`. También puede pasarse `--credentials` con un archivo
dedicado del mismo modo:

```text
TEST_EMAIL=<correo-de-la-cuenta>
TEST_PASSWORD=<contraseña-de-la-cuenta>
TEST_ROLE=User
```

La bandera de producción es obligatoria. Para User y Admin se cambia tanto el
rol esperado como el contenido del archivo anterior:

```bash
python3 scripts/acceptance/robotswarm-sections-e2e.py \
  --execute-production \
  --expected-role User \
  --deployment-commit <sha-completo> \
  --profile-root "/mnt/c/Users/<usuario>/AppData/Local/Temp" \
  --output-dir /tmp/robotswarm-acceptance/sections-user

python3 scripts/acceptance/robotswarm-sections-e2e.py \
  --execute-production \
  --expected-role Admin \
  --deployment-commit <sha-completo> \
  --profile-root "/mnt/c/Users/<usuario>/AppData/Local/Temp" \
  --output-dir /tmp/robotswarm-acceptance/sections-admin
```

El usuario normal debe ver un menú reducido, obtener HTTP 403 desde endpoints
administrativos reales y ser redirigido fuera de las cuatro rutas restringidas.
El rol del perfil debe coincidir con el claim del JWT. Para el recorrido Admin,
cualquier elevación temporal se hace fuera del arnés, revocando refresh tokens;
al terminar se restaura User, se revocan otra vez y se repite la denegación.
El grupo efímero solo se elimina si el diálogo contiene exactamente su nombre.

Las capturas y el JSON quedan en `0600`; antes de cada captura se ocultan
correos, celdas personales, nombres y avatares mostrados por la cuenta, UUID,
direcciones IP y nombres de worker. El perfil efímero se elimina al salir,
incluso después de una interrupción.

## Matriz adaptable de Control

`robotswarm-responsive-e2e.py` reutiliza la cuenta A y las protecciones del
arnés visible, pero solo autentica, visita Control y toma capturas. No crea ni
detiene sesiones, no inicia tareas y no abre el visor. Una única ventana normal
de Chrome recibe mediante CDP viewports de 360, 768, 1366 y 1920 píxeles CSS,
siempre con factor de escala 1 y sin desactivar la GPU. En cada ancho exige que
el viewport de diseño y la captura sean exactos, que el panel de sesión esté
dentro del ancho realmente visible y que la página no tenga desbordamiento
horizontal. Chrome puede descontar del ancho visible el espacio de su barra de
desplazamiento vertical; esa diferencia se registra, pero no altera el ancho CSS
solicitado.

El directorio de salida debe pertenecer al operador y tener modo `0700`; el
reporte y las cuatro PNG se escriben con `0600`. Tanto Chrome como la raíz de
perfiles y la salida son argumentos explícitos:

```bash
install -d -m 0700 /tmp/robotswarm-acceptance/responsive

python3 scripts/acceptance/robotswarm-responsive-e2e.py \
  --execute-production \
  --deployment-commit <sha-completo> \
  --chrome "/mnt/c/Program Files/Google/Chrome/Application/chrome.exe" \
  --profile-root "/mnt/c/Users/<usuario>/AppData/Local/Temp" \
  --output-dir /tmp/robotswarm-acceptance/responsive
```

El programa siempre intenta retirar la emulación, cerrar exclusivamente el
Chrome que lanzó, liberar su puerto CDP y borrar su perfil. Si no puede demostrar
esa limpieza, el resultado no es aprobatorio.

## Matriz ROS visible por escenario

`robotswarm-ros-matrix-e2e.py` ejecuta los catorce casos declarados por
`robotswarm_live_acceptance.py`, en el mismo orden: seis formaciones, tres
recorridos de seguimiento y cinco transportes GRF. Utiliza únicamente la
cuenta A y una ventana normal de Chrome. Cada caso recibe una sesión nueva con
el número de TurtleBot3 que le corresponde; no se reutilizan contenedores entre
casos.

Antes de ejecutar ROS, el arnés exige una sola sesión activa en la cuenta y un
solo contenedor administrado. Comprueba que la imagen esté fijada por digest,
que su etiqueta OCI corresponda al SHA solicitado y que `ImageVersion` tenga el
formato `<sha>+<digest-corto>` generado por el despliegue. También contrasta las
dos vistas de Docker: el contenedor debe figurar conectado exclusivamente a la
red interna etiquetada y esa red debe registrar como único miembro el mismo ID
completo de contenedor. Después solicita el visor real, asocia el runtime
privado del lease al proceso publicador de esa sesión y sólo entonces espera el
primer frame. La localización y la decodificación comparten un único plazo
monotónico. El arnés rechaza el fallback WHEP.

Chrome se ejecuta como una ventana normal, sin `--headless` ni
`--disable-gpu`. La prueba selecciona únicamente el target aleatorio que creó.
Si Windows minimiza esa ventana o deja la pestaña en segundo plano, la restaura
y la trae al frente mediante los dominios reales de Browser y Page; no emula
el estado de visibilidad. Un frame sólo es válido cuando
`document.visibilityState` es `visible`, además de tener dimensiones y estado
de decodificación válidos.

El comando del contenedor siempre selecciona un único caso y conserva
`--min-rtf 2.90`, pero deliberadamente no utiliza `--delete-after`. El runner
detiene y verifica la tarea al emitir `RESULT_JSON` y `SUMMARY_JSON`, pero
mantiene los modelos del enjambre hasta la limpieza de la sesión. Una lectura de
`/gazebo/model_states` exige el conjunto exacto `tb3_0`…`tb3_N-1` antes y después
del escenario; al terminar, la interfaz autenticada debe indicar también `N`
robots activos. La liberación posterior de la sesión elimina finalmente el
enjambre, el contenedor y la red.

El escenario N=1 conserva un tope total de 325 s de pared y, además, presupuestos
independientes de 245 s para `SEARCH`, 20 s para `APPROACH` y 55 s para `PUSH`.
Los cinco segundos restantes cubren transiciones y variación de muestreo. El
resultado publica la razón, la fase y su duración cuando se agota un límite; un
empuje que progresa pero vence a tiempo sigue siendo un fallo. Estos valores se
derivaron de una repetición local visible que necesitó aproximadamente 209,2 s
para buscar y 6,1 s para aproximarse. No se acortó el gate de 0,50 m de avance.

N=3 utiliza un presupuesto distinto, medido sobre su propia geometría: 115 s
para `SEARCH`, 125 s para `APPROACH` y 45 s para `PUSH`, con tope general de
290 s. Las fases suman 285 s y dejan los mismos cinco segundos para bordes y
observación. El límite anterior de 190 s terminó una corrida cuando los tres ya
empujaban —18,5605 s de `PUSH` y 0,3422 m—; una segunda corrida terminó
`APPROACH` a los 100,2932 s mientras el último robot completaba su giro. Ambos
resultados se rechazaron y no se redujo ningún gate físico para la repetición.

El transporte adicional es `transport_grf_n2`. Su gate exige el roster exacto en
`SEARCH` → `APPROACH` → `PUSH` → `DONE`, búsqueda y aviso correlacionados para
ambos robots, rendezvous completo, exactamente dos raíces sobre la carga, cero
compañeros y dos empujadores útiles. La adición cierra una brecha del catálogo y
de sus contratos. El recorrido ROS físico N=2 ya aprobó la lógica del algoritmo:
los dos robots buscaron, el hallazgo se anunció al compañero, ambos completaron
el rendezvous y empujaron como raíces; la carga avanzó más de 0,50 m, el RTF fue
aproximadamente 2,996 y no se registraron colisiones. En esa corrida el HLS rondó
30 FPS y el `gzclient` primario 49,6 FPS. Sin embargo, la sonda gráfica secundaria
concurrente todavía no dispone de un reporte integral aprobatorio. Por ello esta
evidencia acepta el algoritmo N=2, pero no permite marcar como aprobada la matriz
productiva completa.

En los seis casos de formación, la matriz añade de forma explícita
`--formation-active-seconds 75.0`. Esta variante inicia la forma en modo
`moving`, espera una asignación que cubra exactamente toda la flota, exige el
estado correlacionado `moving`, ausencia de error y odometría incompleta, y un
error máximo de posición de 0,12 m. Desde ese punto mantiene la tarea en
`running` durante al menos 75 segundos de pared, recibe mensajes frescos y
vigila parada de emergencia y contactos. Las distancias, límites, velocidad,
aceleración y RTF se vuelven a evaluar con el resto de las métricas al cerrar la
ventana. Luego el runner detiene la misma tarea por su `task_id`. Fuera de esta
opción, `robotswarm_live_acceptance.py` conserva el comportamiento normal de
formación estática y exige que termine en `formed`.

Los tres casos de seguimiento reciben asimismo
`--follow-active-seconds 75.0`. El runner no termina al observar la primera
vuelta válida: exige simultáneamente la vuelta paramétrica requerida y la
ventana activa completa, mantiene el mismo `task_id` en estado `running` y
vuelve a evaluar líder, cadena, separaciones, movimiento, seguridad y RTF al
cerrarla. Ambas ventanas de 75 segundos superan el timeout de 45 segundos de la
sonda gráfica y permiten demostrar solapamiento real sin prolongar
artificialmente los casos de transporte.

Antes de enviar el `docker exec` del runner, el operador prepara dentro del
contenedor un estado y un lock privados, regulares, propiedad del mismo euid y
con modo `0600`. Un supervisor actualiza ese estado bajo `flock`: vuelve a
comprobar estado y cancelación justo antes del `spawn`, registra el hijo solo
después de validar su euid, línea de comandos, script y tarea, y conserva un
tombstone hasta finalizar. Por ello una cancelación anterior al arranque, en la
ventana de arranque o posterior no permite que aparezca un runner tardío. Al
abortar se mata el grupo validado, se solicita `stop_task` correlacionado y se
acepta únicamente un estado terminal de esa tarea o, si nunca llegó a arrancar,
al menos tres publicaciones nuevas de reposo durante medio segundo. Un mensaje
`idle` antiguo o latched no es evidencia suficiente. Solo entonces se eliminan
estado y lock; cualquier duda deja el caso y su limpieza en fallo cerrado.

Para cada escenario se extraen exactamente una línea `RESULT_JSON` y una
`SUMMARY_JSON`. Ambos documentos saneados, sus hashes y el código de salida se
guardan antes de aplicar los criterios de aprobación; por ello un fallo
funcional conserva su diagnóstico sin convertirse en un resultado exitoso.

Los contactos se atribuyen por episodio, no solo por el contador agregado al
final. Observar una instantánea a 10 Hz era insuficiente y registrar el flanco en
el `task_lock` del orquestador todavía dependía del orden de callbacks. La fuente
autoritativa es ahora la misma evaluación de seguridad: `ObstacleAvoidance`
detecta la transición filtrada de falso a verdadero y `CollaborativeTransport`
entrega en ese ciclo la identidad y fase de la tarea, un UUID de fuente, la
secuencia de origen, la secuencia de control y los tiempos de simulación y pared.

El productor incorpora el stream versión 2, con un máximo de 128 eventos, dentro
del mismo `/transport/status`, incluido el mensaje terminal. El orquestador
verifica UUID, ventana de tarea, watermark, continuidad de origen/control,
capacidad, robot, tipo, fase y tiempos; después copia únicamente eventos nuevos
a `/swarm/status.collision_events`. Repetir un status idéntico no duplica el
contador. El `Bool` `/collision_state` sigue mostrando el estado instantáneo,
pero no incrementa el contador cuando la tarea activa es transporte.

El arnés live fija su baseline y conserva tanto los campos de origen
(`source_id`, `source_sequence`, `source_control_sequence`, `source_sim_time` y
`source_wall_time`) como los tiempos posteriores de observación. Un hueco,
reinicio, regresión, secuencia reutilizada, metadato inválido o agotamiento de
capacidad invalida el caso de forma cerrada.

La señal filtrada ya excluye los contactos declarados con la carga y con el
predecesor de una cadena. Por eso cualquier episodio que permanezca en el
contador es inesperado en todas las fases, incluidas `PUSH` y `DONE`. El atraque
requerido se prueba por otra vía: muestras de contacto con carga/predecesor,
geometría ground-truth y participación GRF. Esas mediciones no pueden absolver
retroactivamente un contacto de seguridad.

Las mediciones se separan por fase para no atribuirles un alcance que no tienen.
El `render-report.json` se recoge antes del escenario y queda identificado como
`viewerStartupScene`: demuestra capacidad inicial D3D12/NVIDIA, al menos 45 FPS
en la escena de arranque y un RTF inicial de 2,90, pero no mide el movimiento del
algoritmo.

El reporte inicial se copia y valida inmediatamente después de la sonda HLS,
antes de esperar la tarea ROS. La atestación liga SHA, PID, tick de inicio y
display al `gzclient` vivo. Si el TTL vence durante un caso largo, el arnés
sondea de forma acotada la transición `closing` → hueco del DOM → botón de
apertura; renueva únicamente después de comprobar que el runtime anterior ya no
existe. El estado del lease nuevo se conserva desde la apertura hasta `finally`,
incluso si fallan su localización o HLS. Un binding todavía desconocido impide
declarar la limpieza completa.

La evidencia bajo carga es independiente. El arnés copia desde la imagen
inmutable el `gazebo_gui_preflight.py` oficial y su plugin, y los ejecuta en un
segundo `gzclient` visible dentro de un sandbox acotado. Este proceso usa el
mismo display X11 autenticado, el mismo master ROS/Gazebo privado, el gateway y
los assets versionados del visor. Se lanza después de iniciar el único runner
ROS del caso y de comprobar por un suscriptor ROS independiente que la tarea
sigue `running` y el comportamiento está activo; para formación, esa prueba
requiere además modo y estado `moving`, flota completa y error acotado. El probe
debe completar dos segundos de calentamiento y cinco de muestreo antes de que
ese runner termine. Los intervalos se registran con `time.monotonic`; si el
comportamiento deja de estar activo, el caso falla en vez de atribuirle
falsamente una medición concurrente. El reporte
`activeScenarioGuiProbe` exige D3D12/NVIDIA, FPS medio y post-render de al menos
45, y RTF físico de al menos 2,90 durante esa superposición.

En paralelo con ambos procesos, Chrome mide cinco segundos del HLS mediante
`requestVideoFrameCallback` y `getVideoPlaybackQuality`. Se exige avance del
tiempo multimedia, presentación y decodificación de al menos
`MINIMUM_BROWSER_VIDEO_FPS` —actualmente 27,0— frente al objetivo nominal de
30 FPS, y una proporción de frames descartados no mayor a
0,10. Esta fase demuestra transporte y presentación de video bajo la carga del
escenario; no afirma por sí sola que cada pixel represente movimiento. El
movimiento y la adaptación del enjambre proceden de las métricas correlacionadas
del runner ROS, cuyo RTF mínimo sigue siendo 2,90. Finalmente se toma una PNG
saneada del navegador.

La ejecución productiva requiere todos los argumentos sensibles de forma
explícita:

```bash
install -d -m 0700 /tmp/robotswarm-acceptance

python3 scripts/acceptance/robotswarm-ros-matrix-e2e.py \
  --execute-production \
  --deployment-commit <sha-completo> \
  --credentials /tmp/robotswarm-e2e-credentials.env \
  --chrome "/mnt/c/Program Files/Google/Chrome/Application/chrome.exe" \
  --profile-root "/mnt/c/Users/<usuario>/AppData/Local/Temp" \
  --output /tmp/robotswarm-acceptance/ros-matrix.json
```

Sin `--scenario` se ejecutan los catorce casos. Para una comprobación parcial se
puede repetir la opción, sin duplicar nombres:

```bash
python3 scripts/acceptance/robotswarm-ros-matrix-e2e.py \
  --execute-production \
  --deployment-commit <sha-completo> \
  --credentials /tmp/robotswarm-e2e-credentials.env \
  --chrome "/mnt/c/Program Files/Google/Chrome/Application/chrome.exe" \
  --profile-root "/mnt/c/Users/<usuario>/AppData/Local/Temp" \
  --output /tmp/robotswarm-acceptance/ros-parcial.json \
  --scenario formation_triangle_n3 \
  --scenario transport_grf_n4
```

Para aislar el resultado histórico de 11/13 —obtenido cuando el catálogo tenía
catorce casos— sin consumir el tiempo de los otros doce casos, se pueden repetir
solamente los transportes de uno y tres robots:

```bash
python3 scripts/acceptance/robotswarm-ros-matrix-e2e.py \
  --execute-production \
  --deployment-commit <sha-completo> \
  --credentials /tmp/robotswarm-e2e-credentials.env \
  --chrome "/mnt/c/Program Files/Google/Chrome/Application/chrome.exe" \
  --profile-root "/mnt/c/Users/<usuario>/AppData/Local/Temp" \
  --output /tmp/robotswarm-acceptance/ros-transport-n1-n3.json \
  --scenario transport_grf_n1 \
  --scenario transport_grf_n3
```

Este comando sigue siendo productivo: crea dos sesiones reales, una por caso,
y por ello no debe ejecutarse como parte de las pruebas unitarias ni de CI.
La comprobación local N=1 del candidato recorrió `SEARCH` → `APPROACH` → `PUSH`
→ `DONE`, avanzó 0,5005 m a RTF 2,9964, mantuvo utilidad 1/1 al 100 % y registró
cero colisiones. La comprobación local N=3 también llegó a `DONE`: `tb3_0`
notificó a los otros dos robots, 3/3 quedaron conectados y acoplados, se
observaron 274 lotes de control/GRF y alrededor del 99 % de contribución útil
por robot. El avance fue aproximadamente 0,5005 m, la eficiencia 0,9984, el RTF
aproximadamente 2,996 y no hubo colisiones ni contactos inesperados. N=3 volvió
a aprobar sobre `1448a31`. La primera corrida N=1 aprobó ROS, física y HLS en
ese despliegue, pero su caso formal se rechazó porque el informe gráfico
desapareció antes de la lectura tardía. La repetición con I-139 en el arnés
terminó formalmente `DONE`: avanzó 0,5013 m, mantuvo RTF 2,9962, utilidad 1/1,
cero colisiones, 58,711 FPS en la sonda activa y 30,164 FPS HLS. El reporte se
preservó antes de esperar ROS y la limpieza fue completa. Esa corrida terminó
sin necesitar renovar el lease; la renovación y sus fallos intermedios siguen
cubiertos por los contratos, no por una afirmación física inventada.

El reporte agregado se escribe atómicamente con modo `0600` y no contiene
UUID, correos, contraseñas, tokens ni identificadores de lease, contenedor o
red. Cada caso conserva hashes SHA-256 independientes del resultado ROS, el
resumen, los reportes de arranque y carga, el script y plugin oficiales, las
métricas de video y la captura, además de un hash conjunto. Los artefactos
quedan en un directorio hermano con modo `0700` y archivos `0600`. La salida de
cada proceso hijo se drena simultáneamente por
`stdout` y `stderr` con un límite conjunto de 16 MiB; si lo rebasa, se termina el
grupo de procesos y el bloque de limpieza de la sesión sigue ejecutándose.

En el bloque `finally` de cada caso se cierra primero el visor, se detiene la
sesión y se confirma que desaparecieron su contenedor, su red privada, el
publicador y todos los directorios runtime del lease observados o recuperados.
Si el binding de una renovación no puede recuperarse mediante el lookup seguro
acotado, la limpieza falla cerrada aunque la API haya aceptado el cierre. Si
alguna comprobación falla, no
se inicia el escenario siguiente y el proceso devuelve 3. Una interrupción por
Ctrl-C concede tiempo al runner ROS para limpiar, repite la liberación desde el
control plane y devuelve 130 después de cerrar Chrome y borrar el perfil.

`transport_grf_n4` sigue siendo uno de los catorce escenarios funcionales con la
caja normal. La prueba de carga física N=4 es independiente y se describe a
continuación; no debe ejecutarse de forma concurrente con la matriz.

## Transporte N=4 con carga física

`robotswarm-loaded-n4-e2e.py` cubre la compuerta de carga que no forma parte de
la matriz normal. Usa únicamente la cuenta A, crea mediante el control plane
público una sesión privada nueva con cuatro Burger y abre su HLS real en una
ventana normal de Chrome. La sesión debe
estar vacía al comenzar y se mantiene bajo el mismo bloqueo de operador que la
matriz, de modo que ambas pruebas no pueden competir por la cuenta o la GPU.

El arnés verifica primero el contenedor, la revisión OCI, la imagen inmutable y
la red interna. La apertura se divide deliberadamente en tres hitos ordenados:
petición del visor → localización y validación del runtime del lease → primer
frame HLS decodificado. De este modo, una espera de video no oculta si el
publicador nunca creó el lease correcto. Luego se vincula el proceso `gzclient`
del publicador con ese lease, el display privado y los masters ROS/Gazebo del
mismo contenedor.

El script `gazebo_gui_preflight.py` y su plugin se copian desde la imagen
desplegada a un workspace efímero, privado y ejecutable bajo `/tmp`; no se toman
del checkout del operador ni se ejecutan desde `/run/user`, que puede estar
montado con `noexec`. El sandbox los incorpora de forma individual y de solo
lectura, mientras mantiene el reporte y la autoridad X11 dentro del runtime del
lease. El workspace se elimina de manera explícita durante la limpieza.

El `gzclient` de producción vive en el host GPU y, a diferencia de un cliente
dentro del contenedor, no publica `/gazebo_gui` en ese grafo ROS. El supervisor
entrega `--external-viewer-verified` al probe únicamente después de completar
la correlación anterior y aprobar la sonda visible. Ejecutar el probe a mano sin
esa prueba conserva el modo fail-closed: debe existir `/gazebo_gui`. La opción
no es una vía para omitir el visor, sino la afirmación interna de un supervisor
que ya demostró el proceso externo exacto.

La prueba de carga se inicia antes del preflight, pero este no comienza durante
los ensayos aislados de capacidad. Primero el probe oficial reemplaza la caja y
completa esas mediciones; después, un monitor ROS independiente exige que la
tarea real `transport_grf_n4` esté en ejecución. En cada muestra contrasta el
`task_id`, la fase, el progreso, la masa fresca de 0,75 kg y los conjuntos
exactos `tb3_0`…`tb3_3` tanto en el roster como en los modelos de Gazebo. Solo
entonces lanza el preflight en el mismo display y contra los mismos masters.
Sus 2 segundos de calentamiento y 5 segundos de muestreo deben quedar rodeados
por estados del mismo task. Cada muestra mantiene un orden no decreciente
`SEARCH` → `APPROACH` → `PUSH`; cualquier retorno a una fase anterior se rechaza,
al igual que un retroceso significativo del progreso, y ningún hueco entre
muestras puede superar un segundo. También se exige NVIDIA/D3D12, al menos 45
FPS y RTF 2,90. Una segunda serie de masa rodea y cubre toda la ventana. Los instantes se
toman con `time.monotonic`; así no se infiere el solapamiento a partir de
resultados obtenidos en momentos distintos.

Dentro de ese mismo intervalo, Chrome mide durante 5 segundos el HLS visible
con `requestVideoFrameCallback` y `getVideoPlaybackQuality`. Se exige que el
video avance, presente y decodifique al menos
`MATRIX.MINIMUM_BROWSER_VIDEO_FPS` —actualmente 27,0— frente al objetivo de 30,
que el área útil permanezca visible y que la proporción de frames descartados
no supere 0,10. Por tanto, la tarea GRF N=4 real, la carga física, el preflight
de Gazebo y la reproducción del navegador quedan demostrados de forma
concurrente y con el mismo `task_id`.

El CLI exige para `--preflight-timeout` un mínimo de 55 segundos: 45 para el
probe gráfico activo, 5 para la muestra HLS concurrente y 5 de margen de cierre.
El valor predeterminado es 60 segundos. Un presupuesto inferior se rechaza antes
de crear la sesión y no puede acortar silenciosamente ninguna de esas ventanas.

El probe desplegado recibe explícitamente los siguientes límites de capacidad:

```text
python3 robotswarm_payload_load_live.py \
  --fleet-count 4 \
  --command-speed 0.16 \
  --push-duration 12.0 \
  --single-max-progress 0.05 \
  --root-only-max-progress 0.06 \
  --fleet-min-progress 0.20 \
  --min-robot-progress 0.05 \
  --minimum-gain 4.0 \
  --min-rtf 2.90 \
  --verify-grf-n4
```

Al sustituir la caja, Gazebo puede publicar el modelo en `ModelStates` un ciclo
antes de que `SetModelState` lo resuelva. La colocación reintenta durante cinco
segundos solamente la respuesta exacta de modelo inexistente, cada 50 ms;
cualquier otro error continúa siendo terminal. De esta manera se absorbe la
carrera de arranque sin convertir una avería del servicio en un éxito tardío.

Al hijo GRF se le fijan también 0,75 segundos y 5 muestras de empuje útil, una
fracción útil mínima de 0,50, eficiencia hacia la meta de 0,50, recorridos de
búsqueda de 0,05 m, rendezvous de 0,10 m y 0,03 m para las raíces junto a la
caja. El validador no acepta límites relajados aunque el JSON del probe diga
`passed`; compara los valores publicados con este contrato y vuelve a calcular
la ganancia de cuatro robots a partir del progreso medido. En cada ensayo de
1, 2 y 4 robots exige además velocidad 0,16 m/s, aproximadamente 12 segundos
simulados, duración de pared positiva y el roster exacto. Recalcula
`RTF=tiempo_simulado/tiempo_de_pared`, lo contrasta con el valor publicado y
aplica 2,90 al resultado independiente.

Los 12,0 s simulados terminan antes de los ocho comandos cero de seguridad. El
probe toma ambos extremos temporales en ese punto y después detiene los robots;
el gate tolera como máximo 12,25 s. Una regresión añade 0,50 s simulados durante
la parada y comprueba que no aparezcan en `push_duration_sim_s`. Para GRF, el
supervisor valida por separado dos umbrales: recalcula el eco dinámico desde la
distancia al comienzo de `PUSH`, la tolerancia de llegada y un epsilon fijo de
0,001 m, pero exige siempre al menos 0,50 m de avance físico. Así puede aceptar
un eco coherente de 0,4999 m sin aceptar un desplazamiento real menor a medio
metro.

El protocolo debe contener exactamente un `RESULT_JSON`, un `SUMMARY_JSON`, un
`LOAD_RESULT_JSON` y un `POST_LOAD_CLEANUP_JSON`, en ese orden. Además de aceptar
los límites físicos de 1, 2 y 4 robots, el arnés vuelve a comprobar que los
cuatro robots buscaron, que el hallazgo se anunció a todo el grupo, que los
cuatro completaron el rendezvous y que todos aportaron empuje útil durante la
fase GRF. Durante `PUSH`, el monitor exige en `/swarm/status` el mismo `task_id`
del resultado final, cuatro identificadores `tb3_*`,
`all_pushers_confirmed=true`, cuatro contribuidores útiles y masa de 0,75 kg.
Se comprueba el HLS interactivo y se toma una captura del navegador entre dos
mensajes del mismo task que mantienen el estado `running/PUSH`, el roster y los
modelos exactos. Cada mensaje cita la secuencia y antigüedad de su muestra de
masa; la secuencia debe avanzar después de la captura y el progreso no puede
retroceder de forma significativa. La imagen final por sí sola no se usa como prueba del estado del
algoritmo. La aceptación termina
verificando por ROS que la tarea está terminal, el roster y los modelos
`tb3_*` están vacíos y la caja normal de 0,25 kg fue restaurada por el cleanup
oficial.

La ejecución se autoriza por separado, después de confirmar que backend,
frontend, worker e imagen ROS corresponden al SHA completo solicitado:

```bash
install -d -m 0700 /tmp/robotswarm-acceptance

python3 scripts/acceptance/robotswarm-loaded-n4-e2e.py \
  --execute-production \
  --deployment-commit <sha-completo> \
  --credentials /tmp/robotswarm-e2e-credentials.env \
  --chrome "/mnt/c/Program Files/Google/Chrome/Application/chrome.exe" \
  --profile-root "/mnt/c/Users/<usuario>/AppData/Local/Temp" \
  --output /tmp/robotswarm-acceptance/loaded-n4.json
```

En el estado actual de comisionamiento todavía no existe un reporte aprobatorio
de este comando sobre el release final. Los resultados locales que siguen son
antecedentes de ingeniería y no deben presentarse como aceptación N=4
postdeploy.

En el diagnóstico local del candidato, el límite antiguo de 180 s cortó una
primera corrida con 0,2335 m, 752 muestras y 4/4 robots útiles. Por ello los
presupuestos actuales son 60 s para `SEARCH`, 100 s para `APPROACH` y 190 s
para `PUSH`, dentro de un tope de 355 s, sin modificar el mínimo de 0,50 m. V10
llegó físicamente a `DONE`, pero el gate detectó que la duración de capacidad
incluía aproximadamente 0,52 s de la parada y rechazó el resultado integral.

La repetición corregida v11 aprobó localmente. La capacidad fue
0,0070/0,0340/1,0424 m para uno/dos/cuatro robots, con duraciones simuladas
12,048/12,024/12,000 s, RTF 2,9969/2,9962/2,9975 y ganancia exterior
148,9143×. GRF terminó `SEARCH` en 39,2083 s, entró en `PUSH` a los 115,4389 s
y llegó a `DONE` a los 252,6 s. `tb3_1` notificó a los otros tres robots; 4/4
buscaron, completaron el rendezvous y empujaron mediante dos raíces y dos
compañeros. Las 1598 muestras fueron útiles para toda la flota; el avance fue
0,5002 m, la eficiencia 0,9946 y el RTF exterior 2,9756. Ground-truth registró un
atraque declarado de raíz y el contador de seguridad filtrado permaneció en
cero. La sonda visible durante
`PUSH` midió 58,816/58,831 FPS, RTF 2,996 y viewport 1618×869 en la RTX 3080.

El primer intento de validar externamente v11 rechazó los namespaces frescos
`tb3_0`, `tb3_1`–`tb3_2` y `tb3_3`–`tb3_6` porque suponía que cada subensayo
volvería a comenzar en cero. La regla actual admite cualquier ordinal inicial,
pero exige nombres `tb3_<n>` canónicos, bloques contiguos, mapas de progreso y
conexión iguales, ausencia de reutilización y continuidad monotónica. Una
revisión P2 añadió el gate que exige, antes y después de la captura de `PUSH`,
FPS decodificados finitos y no inferiores a
`MATRIX.MINIMUM_BROWSER_VIDEO_FPS`. El caso de 26,9 frente a 27,0 debe fallar.
La regresión forma parte de los 49/49 contratos vigentes; la evidencia v11 y
los contratos aprobaron en sus alcances respectivos. Esta aprobación es local;
el mismo comando sigue siendo obligatorio sobre el SHA desplegado.

Durante el ensayo no se debe interactuar con la ventana automatizada. El JSON,
los cuatro documentos del protocolo, los documentos sanitizados de
solapamiento y correlación, el reporte oficial de Gazebo y las capturas de
antes, durante `PUSH` y después se escriben con permisos privados y hashes
SHA-256. No se conservan salidas crudas, UUID, credenciales, identificadores de
contenedor, red o lease. El hijo de carga mantiene el protocolo oficial en
`stdout` y reserva una segunda tubería, recibida como `stderr` por el arnés,
exclusivamente para los marcadores live. Ese canal acepta solo líneas completas
con los prefijos `LOADED_PAYLOAD_READY_JSON`, `LOADED_MASS_SAMPLE_JSON`,
`LOADED_GRF_ACTIVE_JSON` o `LOADED_PUSH_LIVE_JSON`; cualquier línea adicional,
incompleta o con otro prefijo termina el hijo en modo fail-closed. El diagnóstico
ordinario de `stderr` se descarta dentro del contenedor y los fallos se clasifican
a partir del protocolo estructurado.

Ambas tuberías se drenan simultáneamente con un límite conjunto de 16 MiB para
el probe; las órdenes Docker aplican la misma cota. El preflight y su `gzclient`
usan cuotas independientes de 1 MiB, y el primero recibe un `TMPDIR` efímero,
propio y `0700` bajo `/tmp`. Al alcanzar un límite se termina el grupo de procesos
correspondiente sin esperar a acumular toda la salida en memoria.

El `finally` detiene primero los hijos acotados, retira los archivos temporales
y el `gzclient` del preflight, cierra el visor, detiene la sesión y exige
ausencia de tarea, roster, proceso del probe, contenedor, red, publicador,
runtime, Chrome, puerto CDP y perfil. Una limpieza no demostrada devuelve
código 3; una interrupción devuelve 130 y no convierte evidencia parcial en
aprobación.

## Smoke visible del control web de transporte

`robotswarm-transport-ui-e2e.py` comprueba el recorrido que no cubre la matriz
ROS ejecutada directamente dentro del contenedor: una persona configura e
inicia `CollaborativeTransport` desde la interfaz pública. Es una prueba
productiva, manual y versionada; no forma parte de GitHub Actions. Usa la
cuenta A y el mismo bloqueo de host que la matriz y el ensayo con carga, por lo
que estos tres arneses no pueden ocupar simultáneamente la sesión o la GPU.

El caso crea una sesión nueva con cuatro TurtleBot3 Burger, exige que su
historial de tareas esté vacío y contrasta el contenedor administrado con la
revisión solicitada. La imagen debe estar fijada por ID inmutable. Además, el
contenedor tiene que ser el único miembro de una sola red Docker interna y
privada. El roster no se deduce de `tb3_0`…`tb3_3`: se lee antes y después de
la tarea en `/api/sessions/{id}/robots`, y se exige igualdad exacta de cuatro
miembros, ordinales sin huecos, namespaces coherentes y estados `Ready` o
`Active`. Después, el arnés solicita el visor, vincula primero el runtime privado
con la sesión y solo entonces espera el primer frame HLS decodificado. Este orden
request → bind → frame impide atribuir un video tardío a otro lease y permite
conservar un diagnóstico acotado si el frame no llega. La ventana de Chrome es
normal; no se admiten `--headless`, `--disable-gpu` ni el fallback WHEP. El
identificador del lease se conserva solo en memoria privada para correlacionar
el publicador, el runtime y su estado autenticado; no se copia al reporte.

La tarjeta «Transporte colaborativo» se selecciona en el navegador y se
escriben `X=-3`, `Y=-4` mediante los dos `input[type=number]` reales de React.
Este desplazamiento corto parte de la caja ubicada en torno a `(-3,-3)` y
permite observar búsqueda, reunión y empuje sin sustituir el algoritmo por un
probe ROS. Antes de iniciar se comprueban validación, estado habilitado,
ausencia de modales y `document.elementFromPoint` en el centro del botón. El
arnés emite exactamente un clic físico sobre «Iniciar tarea». Un listener en
captura exige que sea un único evento primario y confiable del navegador, y
que el primer POST ocurra después de ese gesto.

Un observador instalado previamente en la pestaña consulta cada 100 ms el
inventario autenticado, detecta regresiones y guarda por separado la primera
muestra crítica de cada fase para que un ensayo largo no la expulse del buffer.
En paralelo, un probe acotado de `XMLHttpRequest` observa únicamente el header
`Idempotency-Key`: nunca exporta su valor y conserva solo una huella SHA-256
truncada para demostrar igualdad. Se aceptan de cero a tres respuestas `409`
previas, pero cada una debe ser exactamente
`serialization_conflict/retryable=true`, reutilizar la misma huella y estar
seguida por un único `202`. El `202` debe contener un solo comando `StartTask` y
un solo `TaskRun` correlacionados. Una relectura tardía rechaza clics o requests
que aparezcan después de la supuesta conciliación.

Los identificadores de la respuesta, del `TaskRun`, del worker y del contenedor
se comparan privadamente junto con sus timestamps; el JSON público conserva
solo los resultados booleanos de esas relaciones. Un suscriptor ROS de solo
lectura se inicia dentro de ese mismo contenedor antes del clic. Este suscriptor
exige el autómata sin regresiones `SEARCH` → descubrimiento y `APPROACH` →
`PUSH` → `DONE`, y preserva el aviso, los conteos y el progreso hasta finalizar.
Durante `PUSH` debe observar al menos tres estados consecutivos, durante 0,40 s
o más, con `current_useful_pusher_count` y
`current_useful_pusher_ids` iguales al roster autoritativo, además de un aumento
de progreso mínimo de 0,001.

La salida de este observador ROS se consume de forma incremental, sin esperar a
que termine el proceso ni acumular un prefijo sin salto de línea. Cada línea se
limita a 16 KiB y el grafo completo de objetos JSON retenidos como evidencia a
8 MiB; exceder cualquiera de las dos cotas invalida el recorrido. Al cerrarlo se
solicita primero la salida cooperativa mediante su marcador privado y, si el
grupo continúa vivo, se escala de forma acotada `SIGINT` → `SIGTERM` → `SIGKILL`.
La limpieza solo aprueba cuando desaparece el grupo y terminan ambos lectores.

El smoke anterior podía aceptar `SEARCH` a partir del contador de buscadores sin
demostrar que sus modelos se hubieran desplazado. El mismo suscriptor integra
ahora la distancia recorrida por cada modelo mientras la fase es `SEARCH`. El
smoke exige que cada uno de los cuatro alcance al menos 0,015 m y marca el
movimiento como demostrado directamente. Si cualquiera queda por debajo del
umbral, el recorrido falla de forma cerrada: una aprobación separada del ensayo
con carga no sustituye esta observación correlacionada con el clic y el
`TaskRun`. La API también debe haber registrado a los cuatro robots como
buscadores activos.

Durante `Running`, Chrome mide cinco segundos de HLS: el tiempo multimedia
debe avanzar, la presentación y decodificación deben sostener al menos 27 FPS,
y la proporción de frames descartados no puede superar 0,10. Se guardan una
captura saneada del panel y un recorte de Gazebo tanto en `SEARCH` como en
`PUSH`; antes y después de cada captura se verifica que la fase y el `TaskRun`
siguen siendo los mismos. Los dos frames de Gazebo deben ser distintos: la
distancia de hash queda como indicador auxiliar y al menos 0,0001 del área
debe cambiar más de 32 niveles por canal para descartar pequeñas variaciones del
codificador. Una imagen repetida no prueba movimiento. La captura final
corresponde al resultado `DONE`.

El resultado persistido se vuelve a leer dos veces. Debe demostrar un único
evento `payload_found` correlacionado, un finder perteneciente al roster, tres
compañeros distintos notificados, `all_pushers_confirmed=true` y exactamente
cuatro contribuidores útiles que coinciden con el roster autoritativo. Un
estado fallido, un error, una razón de fallo o cualquier contador explícito de
colisión/contacto inesperado distinto de cero invalida el caso.

La ejecución requiere autorización y entradas privadas explícitas:

```bash
install -d -m 0700 /tmp/robotswarm-acceptance

python3 scripts/acceptance/robotswarm-transport-ui-e2e.py \
  --execute-production \
  --deployment-commit <sha-completo> \
  --credentials /tmp/robotswarm-e2e-credentials.env \
  --chrome "/mnt/c/Program Files/Google/Chrome/Application/chrome.exe" \
  --profile-root "/mnt/c/Users/<usuario>/AppData/Local/Temp" \
  --output /tmp/robotswarm-acceptance/transport-ui.json
```

El archivo de credenciales debe ser regular, pertenecer al operador y tener
modo `0600`; el directorio del reporte y de las evidencias usa `0700`. El JSON
solo conserva conteos y verificaciones: omite correos, contraseñas, tokens,
UUID e identidades de worker, lease, contenedor y red. Al terminar, incluso
después de un fallo o Ctrl-C, el `finally` detiene el suscriptor privado, cierra
el visor y exige que el lease quede inactivo, revocado y con `StopViewer` en
estado terminal `Completed`. Después detiene la sesión y comprueba la
desaparición del publicador, runtime privado, contenedor y red. Por último
cierra exclusivamente el Chrome creado por el arnés y comprueba proceso,
puerto CDP y perfil. Si el arranque de Chrome falla después de crear el perfil,
solo borra el directorio cuyo marcador local todavía coincide con esta
ejecución. Una limpieza incompleta devuelve 3; una interrupción devuelve 130.

El contrato offline no toca infraestructura productiva. GitHub Actions solo lo
compila y ejecuta como prueba unitaria dentro del job existente; nunca pasa
`--execute-production`. Para repetirlo sin gastar minutos de Actions se usa
localmente:

```bash
python3 -m unittest -v \
  scripts.acceptance.test_transport_ui_acceptance_contract
```
