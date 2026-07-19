# Informe técnico de comisionamiento final de RobotSwarm

**Proyecto:** Plataforma web para el control de enjambres TurtleBot3 Burger

**Fecha de inicio de la intervención:** 19 de julio de 2026

**Entorno estudiado:** frontend en Cloudflare Workers, backend en una máquina virtual de la red local y cómputo ROS/Gazebo en Windows Subsystem for Linux (WSL) con una NVIDIA GeForce RTX 3080

## Resumen

Este documento registra el proceso de cierre técnico del proyecto RobotSwarm. Mientras el despliegue y la aceptación pública permanezcan pendientes, no debe interpretarse como un acta final. El trabajo no se limita a enumerar cambios: también conserva los síntomas observados, los intentos que no funcionaron, la forma de localizar cada causa y la evidencia utilizada para aceptar o rechazar una solución. El propósito es que otro estudiante pueda repetir el proceso y distinguir entre una prueba unitaria, una simulación visible y una validación real desde el navegador.

Al iniciar esta etapa, el sistema de control ya permitía crear sesiones aisladas, cambiar el número de robots y ejecutar las tareas de formación, seguimiento de líder y transporte colaborativo. Sin embargo, el visor privado todavía no estaba habilitado en producción, el despliegue del trabajador GPU requería una confirmación manual y existía evidencia contradictoria para la prueba de transporte con diez robots. Por ese motivo, el proyecto no se consideró terminado.

## 1. Objetivos

1. Repetir la prueba visible de transporte colaborativo con diez robots y conservar un resultado verificable.
2. Garantizar que todos los robots busquen el objeto, compartan su posición y colaboren en el empuje.
3. Añadir supervisión del resultado de las tareas y un drenaje seguro de sesiones antes de desplegar el trabajador GPU.
4. Habilitar un visor privado por sesión mediante H.264 y MediaMTX, utilizando el transporte web que resulte viable en la red disponible.
5. Comprobar el aislamiento con usuarios concurrentes y evitar el uso de un escritorio VNC compartido como visor público.
6. Reducir la exposición de servicios administrativos y revisar la configuración sensible de la máquina virtual.
7. Desplegar los cambios por CI/CD y verificar frontend, backend, ROS, GPU y servicios públicos.

## 2. Metodología

Se utiliza una metodología incremental con los siguientes criterios:

- cada corrección parte de una revisión conocida y dispone de un punto de reversión;
- primero se reproduce el problema y luego se modifica el código;
- las pruebas automáticas no sustituyen la observación de Gazebo cuando se evalúan movimiento, FPS o factor de tiempo real;
- las afirmaciones sobre concurrencia requieren por lo menos dos sesiones simultáneas;
- una tarea se acepta únicamente si existe evidencia de su terminación, no solo un `heartbeat` del proceso;
- los secretos nunca se copian a este informe ni a la salida de una prueba.

La revisión inicial de producción fue `3fcc80abb2736f850a32cd2d897b438b1e44108e`. Antes de editar el repositorio se creó la etiqueta remota:

```text
rollback/pre-final-commissioning-3fcc80a
```

## 3. Estado inicial («antes»)

| Elemento | Observación inicial | Estado |
| --- | --- | --- |
| Frontend | `https://rs.zerav.la` respondía HTTP 200 | Operativo |
| Backend | `https://robot.zerav.la/health` respondía `Healthy` | Operativo |
| Trabajador GPU | Servicio activo, registrado y sin reinicios | Operativo |
| Capacidad | Tres sesiones concurrentes y diez robots por sesión | Limitada por configuración |
| Gazebo visible | 54.015 FPS medios, 55.185 eventos de renderizado por segundo y RTF 2.996 en la medición conservada | Aceptado previamente |
| Visor privado | Backend y trabajador tenían la publicación deshabilitada | No comisionado |
| Ruta pública del visor | El nombre `stream.rs.zerav.la` no resolvía y WHEP/ICE solo escuchaban en loopback | No disponible |
| VNC heredado | Escritorio compartido servido por websockify | No cumple aislamiento por usuario |
| Transporte N=10 | El documento indicaba aprobación, pero el registro crudo más reciente indicaba fallo | Evidencia contradictoria |
| Despliegue GPU | Protegido y verificable, pero manual | Parcial |

### 3.1 Evidencia visual inicial

Las capturas de esta sección se toman antes de habilitar el visor. Se conservan en `docs/assets/commissioning-2026-07/`.

![Frontend antes del comisionamiento](assets/commissioning-2026-07/figura-01-frontend-antes.png)

**Figura 1.** Estado inicial del frontend público.

![Visor no comisionado](assets/commissioning-2026-07/figura-02-visor-no-comisionado-antes.png)

**Figura 2.** Espacio de control autenticado antes de crear una sesión. El visor está reservado por cuenta, pero aún no existe una ruta de video operativa.

![Respuesta del VNC heredado](assets/commissioning-2026-07/figura-03-vnc-compartido-antes.png)

**Figura 3.** Una solicitud directa desde la LAN a `http://10.0.0.126:6080` alcanzó el websockify heredado. La respuesta HTTP 405 no elimina el riesgo: confirma que el servicio administrativo aceptaba tráfico externo antes del cambio.

![Fallo DNS del visor](assets/commissioning-2026-07/figura-04-ruta-stream-antes.png)

**Figura 4.** Estado inicial de `stream.rs.zerav.la`; el navegador no podía resolver el nombre.

## 4. Bitácora de incidencias

### I-001. El flujo denominado «ROS deployment» fallaba antes de ejecutar ROS

**Síntoma.** GitHub mostraba el flujo de despliegue GPU en rojo y el nombre del flujo hacía pensar que el error pertenecía a ROS.

**Localización.** La inspección de los pasos reveló que la ejecución se detuvo en `Test GPU worker`: 94 de 95 pruebas aprobaron. Todos los pasos de ROS, construcción de imagen, prueba NVIDIA y despliegue quedaron omitidos.

**Causa.** Un proceso hijo conservaba abierto el descriptor de `stderr` después de morir su padre. El trabajador intentaba leer ese flujo antes de terminar el grupo de procesos, por lo que la recuperación se retrasaba. Además, la prueba esperaba que el intérprete `dash` ejecutara siempre una trampa de `SIGTERM`, comportamiento que no es uniforme en WSL.

**Solución aplicada.** Se cambió el orden: primero se termina el grupo de procesos y después se leen los diagnósticos. La prueba ahora verifica el PID real del proceso huérfano. La corrección aprobó 30 repeticiones de la prueba específica, las 95 pruebas del trabajador y el despliegue GPU completo.

### I-002. La herramienta de control visual de Windows no pudo inicializarse

**Síntoma.** El cliente de automatización terminó antes de abrir una aplicación.

**Mensaje observado.** `sandboxCwd is not a local file URI` para la ruta WSL del repositorio.

**Diagnóstico.** El fallo ocurre en la validación de metadatos del entorno, antes de enviar acciones de teclado o ratón. Por tanto, no corresponde a Chrome, Gazebo ni al sitio web.

**Tratamiento.** Se conserva el error en esta bitácora y se utilizan capturas reproducibles mediante automatización del navegador y herramientas de captura del host. No se presenta una imagen generada como si fuera evidencia real.

### I-003. GitHub CLI no estaba instalado en WSL

**Síntoma.** `gh --version` respondió `command not found`.

**Localización y solución.** Se consultó la versión oficial más reciente, se descargó el binario Linux AMD64 en un directorio temporal y se comprobó su SHA-256 antes de ejecutarlo. La autenticación se obtuvo del almacén de credenciales ya configurado, sin imprimir el token. Git continuó usando la credencial de Windows como vía de respaldo.

### I-004. La primera captura del frontend mostró solamente la pantalla de carga

**Síntoma.** La imagen se tomó antes de que React terminara de montar la aplicación.

**Causa.** El modo de captura finalizó después de recibir el documento HTML, sin esperar las solicitudes y temporizadores de la aplicación de una sola página.

**Solución aplicada.** Se añadió un presupuesto de tiempo virtual de diez segundos y se exigió que el compositor terminara sus etapas antes de guardar la imagen. La Figura 1 muestra el resultado corregido.

### I-005. La captura autenticada agotó el tiempo de espera aunque la página sí había cargado

**Síntoma.** La automatización esperaba el texto `Private simulation viewer` y terminó por tiempo, pero la URL ya correspondía al espacio de control.

**Causa.** La interfaz transforma los encabezados a mayúsculas mediante estilos; la comparación distinguía mayúsculas y minúsculas.

**Solución aplicada.** Se inspeccionó el texto real del documento y se cambió la condición por una verificación independiente de capitalización. La sesión autenticada no se descartó y se obtuvo la Figura 2.

### I-006. Una redirección de Windows creó accidentalmente un archivo llamado `NUL`

**Síntoma.** `git status` mostró un archivo no rastreado que no pertenecía al proyecto.

**Causa.** La expresión `2>NUL`, válida dentro de `cmd.exe`, fue interpretada primero por Bash desde el directorio del repositorio. En lugar de descartar la salida, Bash creó un archivo llamado `NUL` con un diagnóstico de Windows.

**Solución aplicada.** Se comprobó que el archivo contenía solamente la salida generada por esa orden. `apply_patch` no pudo leerlo porque el texto utilizaba una codificación no UTF-8, por lo que se eliminó exclusivamente ese archivo no rastreado. Las llamadas posteriores a programas de Windows se ejecutaron desde `/mnt/c` y sin redirecciones ambiguas.

### I-007. No fue posible crear usuarios de prueba con la cuenta de arranque

**Síntoma.** La autenticación administrativa respondió HTTP 400 y los intentos siguientes recibieron HTTP 401.

**Localización.** La inspección mostró que las variables `BootstrapAdmin__Email` y `BootstrapAdmin__Password` existían en el contenedor, pero sus valores tenían longitud cero. Este comportamiento es correcto después del aprovisionamiento: producción no conserva una contraseña administrativa de arranque.

**Solución aplicada.** Se generaron dos contraseñas aleatorias fuera del repositorio, se calcularon hashes BCrypt compatibles con el backend y se insertaron dos cuentas de usuario temporales directamente en PostgreSQL. El archivo local de credenciales se creó con modo `0600`; su contenido no se imprimió. Las cuentas se eliminarán junto con sus sesiones al terminar la aceptación.

### I-008. La captura directa de X11 produjo una imagen negra de Gazebo

**Síntoma.** `ffmpeg -f x11grab` generó una imagen completamente negra aunque `gzclient` era visible en WSLg.

**Localización.** Se comprobó que Gazebo seguía renderizando y que el fallo se limitaba a la captura de la ventana raíz. WSLg compone algunas superficies aceleradas de una forma que no queda representada correctamente al leer esa superficie mediante `x11grab`.

**Solución aplicada.** Se localizó el identificador X11 de la ventana titulada `Gazebo` con `xwininfo`, se extrajo esa ventana mediante `xwd` y luego se convirtió el archivo XWD a PNG. Las Figuras 5–7, 9 y 10 provienen de esa ruta y fueron inspeccionadas antes de incorporarlas al informe.

### I-009. La primera escena de búsqueda distante no permitía completar el empuje N=10

**Síntoma.** La fase `SEARCH` cumplió la comunicación entre robots, pero la ejecución terminó al iniciar `APPROACH`.

**Datos observados.** Se obtuvieron 345 muestras de búsqueda; los diez robots recibieron movimiento, `tb3_7` emitió un único aviso y los otros nueve lo confirmaron. Sin embargo, la combinación objeto `(3.5, 3.5)` y meta `(-0.8, -3.0)` no dejaba espacio suficiente para organizar dos cadenas de cinco robots. Además, `tb3_0` quedó inicialmente junto a un cilindro y registró un contacto.

**Criterio adoptado.** No se modificó el algoritmo para ocultar una entrada geométricamente inviable. El fallo se registró como prueba negativa en la bitácora de trabajo, aunque su JSON crudo no forma parte del paquete retenido. Se repitió el ensayo con un objeto distante, una meta factible y separación segura respecto de los obstáculos. La segunda ejecución aprobó sin colisiones y sí conserva sus datos estructurados.

### I-010. El entorno local no pudo instalar la misma versión de NumPy utilizada por CI

**Síntoma.** La creación de un entorno virtual local rechazó `numpy==1.26.4` indicando que no existía una distribución compatible.

**Causa.** La distribución WSL de trabajo utiliza Python 3.8.10; NumPy 1.26 requiere una versión posterior de Python. GitHub Actions ejecuta la suite en un Ubuntu más reciente, por lo que el archivo de CI no era el origen del error.

**Tratamiento.** La validación local se ejecutó con NumPy 1.24.4 y PyYAML 5.3.1, ya instalados y compatibles con Python 3.8. La verificación exacta con NumPy 1.26.4 se deja a la ejecución reproducible de GitHub Actions; ambas rutas deben aprobar antes del despliegue.

### I-011. MediaMTX iniciaba correctamente, pero el proxy HLS no podía obtener la primera lista

**Síntoma.** La comprobación inicial solo verificaba que el contenedor MediaMTX 1.18.2 permaneciera activo. Al publicar después una señal H.264 real, la solicitud de `index.m3u8` respondió HTTP 302 y añadió `cookieCheck=1`.

**Cómo se localizó.** Se creó una red Docker temporal, se publicó un patrón de video con FFmpeg y se registraron las solicitudes de autenticación recibidas por MediaMTX. El token de publicación llegó correctamente, pero no apareció ninguna lectura: la redirección ocurrió antes. La revisión del código fuente etiquetado `v1.18.2` confirmó que las versiones recientes crean una sesión HLS mediante cookie o parámetro de consulta. Esto era incompatible con el proxy deliberadamente estricto, que no sigue redirecciones, no conserva cookies y no reenvía parámetros desconocidos.

**Solución aplicada.** Se configuró `hlsCDNSecret` como una credencial interna diferente del lease del usuario. El backend sigue validando el lease y la ruta exacta en la base de datos, pero usa esa credencial solamente en el salto privado hacia MediaMTX. En modo CDN, MediaMTX no necesita la cookie del navegador. La aplicación falla cerrada si el secreto falta o no cumple el formato base64url esperado. El secreto no aparece en URL, lista, respuesta ni registro de prueba.

### I-012. La primera prueba HLS completa reveló dos valores válidos en YAML, pero inválidos al crear el muxer

**Síntoma.** Después de resolver la redirección, la solicitud respondió primero `maximum reader count reached` y luego `Low-Latency HLS requires at least 7 segments`.

**Diagnóstico.** Ambos errores solo aparecen cuando existe un publicador y MediaMTX intenta crear el muxer; por eso no fueron detectados por una comprobación que se limitaba al arranque del contenedor. La ruta admitía un lector, pero LL-HLS utiliza un lector para el muxer y otro para su sesión CDN. Además, la configuración indicaba cinco segmentos aunque la variante de baja latencia exige siete como mínimo.

**Solución aplicada.** `maxReaders` pasó de 1 a 3: dos plazas son el mínimo de LL-HLS y la tercera permite el solapamiento breve de un fallback WHEP mientras cierra el muxer. `hlsSegmentCount` pasó de 5 a 7. La validación de CI ahora publica video H.264, solicita la lista principal, sigue con la lista de video y descarga una parte MP4 mediante el mismo binario fijado de MediaMTX. La repetición local produjo HTTP 200 sin `Location` ni `Set-Cookie` y conservó nombres canónicos `.m3u8` y `.mp4` compatibles con el proxy.

La sonda conserva además `authMethod: http`, tal como se ejecuta en producción. Para no depender de una base de datos durante CI, levanta un fixture HTTP mínimo en la dirección esperada por MediaMTX. Este fixture acepta solamente la combinación correcta de acción (`publish` o `read`), ruta canónica y token de prueba; también registra los rechazos sin guardar el valor de los tokens. La prueba exige una publicación y una lectura RTSP autorizadas, rechaza un token utilizado para la acción equivocada y una lectura dirigida a otra sesión, y finalmente confirma que las listas y partes HLS usan exclusivamente `hlsCDNSecret` en el salto privado.

### I-013. Un comando obsoleto podía sobrevivir a una cancelación rápida

**Síntoma potencial.** Al detener y volver a iniciar una sesión en poco tiempo, dos instancias del trabajador podían leer el mismo comando pendiente. La comprobación del estado y el cambio a `Acknowledged` ocurrían en operaciones distintas. Una respuesta HTTP exitosa tampoco demostraba por sí sola que ROS ya hubiese terminado la tarea cancelada.

**Cómo se localizó.** Se escribieron pruebas con dos contextos relacionales independientes para reproducir la carrera real de base de datos. Las pruebas de memoria no eran suficientes porque no aplican del mismo modo los tokens de concurrencia. También se simuló una cancelación mientras el puente ROS continuaba reportando una tarea activa.

**Solución aplicada.** Los estados del comando y de la tarea se marcaron como tokens de concurrencia. El trabajador debe ganar una transición atómica antes de ejecutar el manejador; si otro proceso la ganó, descarta el sobre obsoleto. Para cancelar, el trabajador envía la orden a ROS y consulta el estado correlacionado hasta observar el terminal esperado. El monitor del backend ignora resultados que pertenezcan a otra tarea o a una revisión anterior.

### I-014. Los contadores agregados podían aceptar una colaboración incompleta

**Síntoma potencial.** Un resultado podía declarar diez robots mediante un contador y, sin embargo, repetir un identificador, omitir un compañero o acreditar como útil a un robot fuera de la sesión.

**Localización.** Se construyeron resultados sintéticos con el mismo total numérico, pero con conjuntos de identificadores diferentes. La política anterior no comparaba el roster autorizado con cada conjunto de evidencia.

**Solución aplicada.** La aceptación de transporte ahora utiliza igualdad de conjuntos: el descubridor debe pertenecer al roster, los notificados deben ser exactamente todos los compañeros y los contribuidores útiles deben coincidir con todos los robots asignados. El requisito se protege mediante `Tasks__RequireCollaborativeTransportEvidence`; su activación se planificó en dos etapas para no rechazar tareas mientras un trabajador antiguo siguiera registrado.

### I-015. Un `heartbeat` podía competir con el drenaje del trabajador GPU

**Síntoma potencial.** El despliegue necesitaba impedir nuevas sesiones, pero un registro o latido simultáneo podía volver a presentar al trabajador como disponible. Separar el bloqueo en GitHub de la decisión del scheduler no eliminaba esta ventana.

**Cómo se localizó.** Se ejecutaron dos contextos de base de datos contra la misma fila: uno adquirió el drenaje y el otro intentó actualizar el trabajador. Después se forzó al scheduler a consultar un estado desactualizado.

**Solución aplicada.** El lease de drenaje tiene identificador, revisión objetivo, caducidad y token de concurrencia. El scheduler excluye cualquier trabajador con un lease vigente aunque su campo de estado todavía no refleje `Draining`. Los registros y latidos reintentan las colisiones sin borrar el lease. El workflow adquiere el lease autenticado, espera cero sesiones tanto en backend como en el reporte del trabajador, lo verifica otra vez antes de desplegar y solo reanuda la planificación tras una puesta en servicio correcta.

### I-016. Los dos despliegues podían modificar producción con revisiones distintas

**Riesgo encontrado.** Backend y trabajador GPU utilizaban grupos de concurrencia distintos. Además, el backend decidía si debía desplegar comparando únicamente `HEAD^`; en un `push` con varios commits podía omitir archivos relevantes. El rollback de MediaMTX dependía de un nombre de imagen mutable.

**Tratamiento.** Ambos workflows comparten el grupo `production-robotswarm`. El backend despliega toda ejecución exitosa de CI sobre `main`, y los dos flujos vuelven a consultar el SHA actual inmediatamente antes de su primera modificación. MediaMTX quedó fijado por digest y el rollback conserva el identificador inmutable de la imagen y la configuración que estaban activos. Las consultas de drenaje y salud tienen límites explícitos de conexión y duración, de modo que una red interrumpida no deja el job bloqueado indefinidamente.

### I-017. La topología del proxy y la publicación multimedia no estaba expresada como una condición verificable

**Observación.** Nginx Proxy Manager llega al backend desde `10.0.0.2`, mientras que el backend y MediaMTX se publican en `10.0.0.126`. Si estas variables quedan vacías, en loopback o contienen otra interfaz, la aplicación puede atribuir la limitación de peticiones al proxy completo o publicar RTSP en un socket distinto del previsto.

**Cómo se comprobó.** La dirección del proxy se obtuvo de las conexiones activas del host, y las direcciones de publicación se contrastaron con los sockets Docker. No se dedujeron a partir del nombre DNS público.

**Solución aplicada.** El workflow exige direcciones IPv4 unicast y no loopback para el proxy y el backend; cuando se habilita la publicación, aplica la misma regla a MediaMTX y compara el resultado real de `docker port media_prod 8554/tcp` con `10.0.0.126:8554`. El backend solo confía en el proxy declarado para interpretar encabezados reenviados.

### I-018. Una instalación local reutilizada produjo un fallo de frontend que no existía en el repositorio

**Síntoma.** La primera verificación de frontend falló dentro de Babel usando el directorio `node_modules` que ya existía en WSL.

**Diagnóstico.** El árbol instalado no correspondía de forma limpia con `package-lock.json`; por tanto, repetir órdenes sobre él no permitía separar un defecto del cambio de un residuo local.

**Solución aplicada.** Se creó una copia limpia, se utilizó Node 18 y se ejecutó `npm ci` exclusivamente desde el lockfile. En ese entorno aprobaron 4 suites con 21 pruebas, el lint dirigido y la construcción de producción. `npm audit` conserva 99 avisos que ya estaban presentes en la revisión base (17 bajos, 30 moderados, 47 altos y 5 críticos); `hls.js` no añadió un aviso nuevo. Esta deuda no se declara resuelta y se incluye en las limitaciones.

### I-019. El archivo de hashes conservaba nombres de evidencias que ya habían sido renombradas

**Síntoma.** La comprobación completa de `checksums.sha256` informó siete archivos ausentes. Cinco capturas sí estaban en el repositorio, pero con los nombres finales `figura-05` a `figura-10`; otros dos archivos voluminosos se habían excluido deliberadamente.

**Causa.** El manifiesto se generó antes de seleccionar y renombrar la evidencia del informe. La instrucción `--ignore-missing` ocultaba esa diferencia, pero no demostraba que el paquete entregado estuviese íntegro.

**Solución aplicada.** Se preservó `manifest.txt` como inventario histórico y se regeneró el archivo de hashes para referir únicamente los datos retenidos y sus figuras derivadas. La comprobación actual no ignora ausencias:

```bash
cd docs/assets/commissioning-2026-07/datos-transporte-n10
sha256sum -c checksums.sha256
```

El resultado fue 16 de 16 entradas correctas.

### I-020. Una interrupción durante el cuerpo HLS no se traducía a un error controlado

**Síntoma reproducido.** La ruta devolvía 502 cuando no podía conectar con MediaMTX, pero una desconexión después de recibir las cabeceras podía lanzar `IOException` al copiar el cuerpo y cerrar la respuesta de forma abrupta.

**Solución aplicada.** El proxy trata tanto los fallos de solicitud como los de lectura del cuerpo como indisponibilidad del upstream. Se añadió una regresión que interrumpe el contenido después de las cabeceras y comprueba la respuesta 502. Los límites de tamaño y tiempo, la validación exacta de ruta y lease, y los cupos globales y por sesión permanecen activos.

### I-021. La repetición final no encontró `dotnet` en el `PATH` del shell

**Síntoma.** La orden de prueba terminó inmediatamente con `dotnet: command not found`.

**Diagnóstico y tratamiento.** El SDK 8.0.422 sí estaba instalado en `/tmp/robotswarm-dotnet-8.0.422`, pero el shell no interactivo de esa ejecución no heredó su directorio en `PATH`. Se repitieron las suites con la ruta absoluta del mismo SDK: backend aprobó 124/124, trabajador 99/99 y ROS 351/351. Este incidente pertenece al entorno local y no se atribuyó al código ni se corrigió alterando el proyecto.

### I-022. La publicación necesitó utilizar el almacén de credenciales de Windows

**Síntomas.** El `push` desde WSL respondió `could not read Username`; el conector de GitHub pudo leer el PR, pero recibió HTTP 403 al intentar crearlo. Git para Windows sí tenía una credencial válida, aunque su primera ejecución rechazó el directorio WSL por `dubious ownership`.

**Tratamiento.** Se invocó Git para Windows con una excepción `safe.directory` limitada a esa orden. El PR se creó con GitHub CLI obteniendo la credencial del almacén existente y manteniéndola solo en una variable de proceso. No se escribió ni mostró el token. No se cambió la configuración global de Git.

### I-023. El runner remoto ya no incluía FFmpeg

**Síntoma.** En el PR 94, backend, pruebas y migración aprobaron, pero `Validate pinned MediaMTX configuration` terminó justo después de los cinco segundos de arranque del contenedor y antes de iniciar la publicación H.264.

**Cómo se localizó.** El log completo de Actions confirmó MediaMTX 1.18.2 activo y no mostró ninguna conexión del publicador. Entre ese punto y la primera publicación, el único chequeo sin diagnóstico era `ffmpeg -encoders | grep libx264` bajo `set -o pipefail`. Primero se eliminó esa tubería para descartar un `SIGPIPE` por salida temprana y para añadir mensajes a cada prerrequisito. La segunda ejecución produjo entonces el diagnóstico inequívoco: `The CI runner does not provide FFmpeg`.

**Solución aplicada.** CI instala `ffmpeg` explícitamente antes de la sonda y deja de depender del inventario cambiante de `ubuntu-latest`. La versión de MediaMTX se captura y compara; la lista completa de encoders se guarda primero en un archivo temporal y después se inspecciona. Cada prerrequisito devuelve ahora un mensaje propio. La sonda exacta, extraída del YAML corregido, aprobó localmente con publicación/lectura RTSP, dos rechazos de autenticación y entrega LL-HLS completa.

## 5. Registro cronológico de cambios y pruebas

Esta sección se actualiza durante el trabajo. Cada entrada debe incluir revisión, entorno, entrada de la prueba, resultado y referencia a su evidencia.

| Fecha (AST) | Actividad | Resultado | Evidencia |
| --- | --- | --- | --- |
| 2026-07-19 | Creación del punto de reversión | Etiqueta remota creada sobre `3fcc80a` | Git remoto |
| 2026-07-19 | Verificación pública inicial | Frontend HTTP 200; backend `Healthy` | Consulta HTTP |
| 2026-07-19 | Inicio del informe | Documento creado antes de las modificaciones funcionales | Este archivo |
| 2026-07-19 | Transporte visible con diez robots | Aprobado; diez robots asignados y físicamente vinculados | Figuras 5–7 y reporte JSON |
| 2026-07-19 | Búsqueda distante N=10, escenario inicial | La búsqueda aprobó; la geometría posterior de empuje fue rechazada como inviable. El crudo no se retuvo | Incidencia I-009 |
| 2026-07-19 | Búsqueda y aviso N=10, escenario corregido | Aprobado; 10/10 buscaron, 9/9 respondieron al aviso y no hubo colisiones | Figuras 9–10 |
| 2026-07-19 | Endurecimiento del acceso VNC | Puertos 5901 y 6080 limitados a loopback; token retirado de los argumentos del proceso | Figura 8 y verificación de servicios |
| 2026-07-19 | Sonda HLS con H.264 real, primer intento | Rechazada por redirección de sesión, límite de lectores y número de segmentos | Incidencias I-011 e I-012 |
| 2026-07-19 | Sonda HLS con credencial CDN interna | Aprobada localmente: lista principal, lista de video y partes MP4 sin cookies ni redirecciones | Prueba reproducible de CI |
| 2026-07-19 | Pruebas de concurrencia del plano de control | Aprobadas las carreras de claim, cancelación, resultado y drenaje | Incidencias I-013 a I-015 |
| 2026-07-19 | Revisión de despliegues | Lock compartido, revalidación de `main` y rollback inmutable incorporados | Incidencias I-016 e I-017 |
| 2026-07-19 | Suite local completa | Backend 124/124; trabajador 99/99; ROS 351/351; frontend 21/21, lint y build | [Resumen local previo al despliegue](assets/commissioning-2026-07/predeploy-validation-summary.txt) |
| 2026-07-19 | Verificación del paquete de evidencia | 16/16 hashes correctos, sin ignorar archivos ausentes | Incidencia I-019 |
| 2026-07-19 | PR 94, dos primeras ejecuciones de CI | Se hizo visible que `ubuntu-latest` no incluía FFmpeg; dependencia añadida de forma explícita | Incidencia I-023 |

## 6. Resultados previos al despliegue

Los datos de esta sección se capturaron durante la intervención sobre la revisión base `3fcc80a`; `source-sha.txt` conserva esa referencia. Sirven como aceptación visible del algoritmo y como línea base de rendimiento, pero no se presentan como evidencia del artefacto que todavía no había pasado por GitHub CI. Después de desplegar el SHA definitivo se repetirá una muestra representativa desde el frontend público y se documentará separadamente.

### 6.1 Transporte colaborativo con diez robots

La evidencia cruda anterior no respaldaba la afirmación de aprobación. El registro más reciente permanecía en `APPROACH`, no declaraba roles para los diez robots, no contenía muestras GRF y mostraba desplazamiento cero. La nueva ejecución se realizó con `gzclient` visible, renderizado D3D12 en la RTX 3080 y física configurada a un factor 3.0.

El resultado fue `DONE/completed` y cumplió los criterios siguientes:

- 10 de 10 robots asignados y físicamente vinculados a la estructura de empuje;
- 175 lotes GRF coherentes;
- 0.922 m de avance hacia la meta;
- eficiencia de trayectoria 0.9997;
- 64.0 % de lotes con contribución útil de todo el enjambre, frente a un mínimo exigido de 50 %;
- intervalo máximo de empuje útil simultáneo de 4.2 segundos simulados;
- cero episodios de colisión y cero contactos inesperados;
- RTF 2.9505 durante la carga y 2.996 en la prueba gráfica previa.

El resultado contiene tres advertencias que no deben omitirse: ninguno de los robots mantuvo individualmente el ritmo nominal fijo de 0.015 m/s, la flota completa no lo sostuvo durante una ventana simultánea y la referencia nominal de empuje se siguió en menos del 50 % de los lotes. La aceptación se produjo por el criterio adaptativo: hubo contribución útil del roster completo en el 64 % de los lotes, un intervalo simultáneo de 4.2 s y avance casi rectilíneo. Por tanto, este ensayo demuestra coordinación y contribución, pero no una velocidad nominal constante.

La caja verde de 0.25 kg es una carga de práctica que un solo Burger puede mover. Un ensayo histórico separado con la [caja cargada de 0.75 kg](loaded-transport-acceptance.md) produjo 0.0072 m con un robot, 0.0336 m con los dos robots raíz y 1.1796 m cuando dos compañeros empujaron a través de esas raíces. Su salida cruda no quedó incluida en este paquete, por lo que la demostración de reparto físico se repetirá en la revisión final desplegada; no se infiere únicamente de las Figuras 5–7.

Las mediciones estructuradas, el registro del caso anterior y sus hashes se conservan en [`datos-transporte-n10`](assets/commissioning-2026-07/datos-transporte-n10/README.md).
El conjunto completo de capturas y resúmenes previos puede comprobarse desde su directorio con `sha256sum -c checksums.sha256`.

![Aproximación simultánea de diez robots](assets/commissioning-2026-07/figura-05-transporte-n10-aproximacion.png)

**Figura 5.** Los diez robots se aproximan al objeto y a sus posiciones de colaboración.

![Empuje en dos cadenas](assets/commissioning-2026-07/figura-06-transporte-n10-empuje.png)

**Figura 6.** Dos robots mantienen contacto directo con la caja y ocho compañeros forman dos cadenas balanceadas. Los diez aportan fuerza en el mismo intervalo.

![Continuidad del empuje](assets/commissioning-2026-07/figura-07-transporte-n10-empuje-final.png)

**Figura 7.** Instantánea posterior del mismo empuje. La continuidad no se deduce de una sola imagen: se contrasta con la serie temporal y los 175 lotes GRF del resultado estructurado.

### 6.2 Reducción de la superficie administrativa

El servicio VNC se conserva únicamente para administración y diagnóstico del host. Los sockets 5901 y 6080 pasaron de escuchar en la LAN a escuchar solamente en `127.0.0.1`. El túnel de Cloudflare continuó operativo, mientras que su token dejó de aparecer en la línea de comandos del proceso y se trasladó a un archivo de entorno propiedad de `root`, con permisos `0600`.

![VNC restringido a loopback](assets/commissioning-2026-07/figura-08-vnc-loopback-despues.png)

**Figura 8.** Intento desde la LAN a `http://10.0.0.126:6080` después del cambio: el puerto ya no acepta una conexión directa. El acceso administrativo exige el canal autorizado. La [salida `ss` conservada](assets/commissioning-2026-07/vnc-bind-after.txt) confirma que 5901 y 6080 escuchan en loopback; la captura documenta el efecto observado desde otro equipo.

### 6.3 Búsqueda distribuida, descubrimiento y aviso

La búsqueda distante se ensayó por separado para obtener suficientes muestras antes del descubrimiento. Así se evitó aceptar la propiedad «todos buscan» a partir de un caso donde el objeto apareciera demasiado cerca del despliegue inicial.

En la ejecución aceptada se registraron 905 muestras. Los diez robots mantuvieron comandos de movimiento en el 100 % de sus muestras y recorrieron individualmente entre 6.7981 m y 10.1385 m. `tb3_1` encontró el objeto y emitió un único evento `payload_found`; la notificación incluyó a los otros nueve robots, se recibieron diez confirmaciones totales y los nueve compañeros iniciaron la maniobra `APPROACH`. Cuatro de ellos giraron o se alejaron brevemente mientras actuaba el evitador, por lo que no se afirma progreso radial inmediato para cada robot. Después del aviso se observaron 16 muestras de movimiento por compañero y el contador de colisiones no aumentó.

![Búsqueda distribuida con diez robots](assets/commissioning-2026-07/figura-09-busqueda-n10.png)

**Figura 9.** Los diez TurtleBot3 Burger recorren regiones distintas del entorno durante la fase `SEARCH`.

![Convergencia después del aviso](assets/commissioning-2026-07/figura-10-convergencia-tras-aviso-n10.png)

**Figura 10.** Tras el evento del robot descubridor, los nueve compañeros responden y cambian simultáneamente de `SEARCH` a la maniobra `APPROACH`; el evitador determina su giro inicial.

### 6.4 Verificación del plano de control y del visor antes del despliegue

La revisión local cerró con 124 pruebas de backend, 99 del trabajador, 351 del paquete ROS y 21 de frontend. También aprobaron la migración idempotente de la base de datos, el renderizado de Docker Compose, la unicidad de claves YAML, la sintaxis Bash y Python, y `git diff --check`. Se conserva un [resumen curado de comandos y resultados](assets/commissioning-2026-07/predeploy-validation-summary.txt); GitHub Actions será la fuente autoritativa sobre el SHA publicado.

La sonda multimedia utilizó el mismo digest de MediaMTX 1.18.2 previsto para producción. Publicó una señal H.264 por RTSP, leyó el flujo autorizado, rechazó tanto el token de publicación usado como lector como un lease aplicado a otra sesión, y descargó la lista principal, la lista de video, el MP4 de inicialización y una parte LL-HLS. Estos resultados aceptan la implementación para desplegar con los gates apagados; todavía no aceptan la experiencia pública de dos usuarios, que requiere la etapa operacional siguiente.

### 6.5 Resumen de cambios por capa

| Capa | Cambio principal | Forma de verificación previa |
| --- | --- | --- |
| Frontend | Reproductor `hls.js` asociado al lease privado, recuperación controlada y presentación del resultado terminal de la tarea | 21 pruebas, lint dirigido y build de producción |
| Backend | Proxy LL-HLS autenticado, monitor de resultados, política de roster completo, lease de drenaje y protección de carreras de estado | 124 pruebas, incluidas pruebas relacionales y fallos de upstream |
| Trabajador GPU | Publicador por display privado, anuncio de capacidades, claim atómico y cancelación confirmada contra ROS | 99 pruebas y sonda del ejecutable de publicación |
| ROS | Evidencia correlacionada de notificación, roster y contribución útil para transporte | 351 pruebas y simulaciones visibles N=10 |
| Infraestructura | MediaMTX por digest, origen HLS privado, despliegues serializados, revalidación de SHA y rollback inmutable | Compose, YAML, sonda H.264/RTSP/LL-HLS y revisión estática de workflows |

## 7. Limitaciones y trabajo futuro

- ROS Noetic llegó al final de soporte el 31 de mayo de 2025 y Gazebo Classic en enero de 2025. El contenedor fija el entorno actual para reproducibilidad, pero la migración a ROS 2 y Gazebo moderno sigue siendo necesaria.
- El TurtleBot3 Burger utilizado no incorpora una cámara compatible en su modelo actual; el visor entrega la escena privada de Gazebo, no una cámara a bordo del robot.
- LL-HLS evita exponer ICE/UDP y TURN, pero añade latencia y hace pasar el video por el backend. La capacidad y el costo de ancho de banda se medirán durante la prueba concurrente.
- La validación previa del publicador utilizó la ruta de codificación disponible con `libx264`. El encoder efectivo del release GPU (`h264_nvenc` o fallback) se registrará durante el despliegue.
- `npm audit` informa 99 avisos heredados de la revisión base. El cambio de visor no incrementó esa cifra ni añadió un hallazgo para `hls.js`, pero la actualización de dependencias antiguas requiere un trabajo separado y pruebas de regresión de la interfaz.
- Las matrices de figuras, trayectorias y cantidades son muestras representativas; no constituyen una prueba exhaustiva de toda combinación posible de mapa y parámetros.
- Hasta finalizar la aceptación pública siguen pendientes dos navegadores autenticados simultáneos, denegación cruzada, caducidad de lease, parada aislada, repetición de la carga física y medición de FPS/RTF bajo concurrencia.

## 8. Conclusiones

> Las conclusiones se redactarán cuando finalicen el despliegue y las pruebas concurrentes.

## 9. Referencias

1. P. Rezeck, R. M. Assunção y L. Chaimowicz, «Cooperative Object Transportation using Gibbs Random Fields», *IROS*, 2021. [Artículo y preprint](https://arxiv.org/abs/2109.13734); [implementación de referencia](https://github.com/rezeck/grf_transport), revisión consultada `7e89a67e88a49dfadbf1f11b342791fd3ebd4cf0`.
2. MediaMTX, [documentación oficial de HLS](https://mediamtx.org/docs/read/hls) y [autenticación mediante servidor HTTP](https://mediamtx.org/docs/features/authentication).
3. MediaMTX, [uso de `hlsCDNSecret` frente a un origen HLS](https://mediamtx.org/docs/features/scalability) y [release v1.18.2](https://github.com/bluenviron/mediamtx/releases/tag/v1.18.2).
4. Open Robotics, [fin de soporte de ROS Noetic](https://ros.org/blog/noetic-eol/) y [fin de soporte de Gazebo Classic](https://classic.gazebosim.org/).
