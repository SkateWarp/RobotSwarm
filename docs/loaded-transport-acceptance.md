# Aceptación del transporte colaborativo con carga

## 1. Objetivo y alcance

Esta prueba comprueba que cuatro TurtleBot3 Burger pueden encontrar, rodear y
transportar juntos una carga que no debería ser desplazada de forma útil por un
solo robot. No basta con observar movimiento en Gazebo: la aceptación relaciona
la física de la carga, los estados del algoritmo GRF, el visor HLS abierto en un
navegador normal y la limpieza posterior de la sesión.

La caja verde del escenario habitual tiene una masa de 0,25 kg y
`mu=mu2=0.05`.
Sirve para las pruebas rápidas de búsqueda, aviso, encuentro y control, pero un
Burger puede moverla por sí solo. Para estudiar reparto físico de carga se
utiliza el perfil `transport_crate_loaded`, de 0,75 kg y con un coeficiente de
fricción de contacto `mu=mu2=0.25`.

La selección de esos valores parte de una estimación sencilla:

- fuerza de inicio de movimiento de la caja: `0,75 × 0,25 × g = 1,84 N`;
- tracción estimada de un Burger de 1 kg: `1,00 × 0,10 × g = 0,98 N`;
- tracción estimada de cuatro Burger: `4 × 0,98 = 3,92 N`.

Este cálculo solo sirve para escoger un caso de prueba razonable. El criterio de
aceptación procede de las mediciones del simulador y no de la estimación.

## 2. Ejecución supervisada

El gate productivo se inicia desde el equipo con GPU cuando no existe otra
sesión de aceptación. Se emplea Chrome visible, sin `--headless` y sin
`--disable-gpu`, y se crea una sesión nueva de cuatro robots mediante la misma
interfaz pública que utiliza una persona.

```bash
python3 scripts/acceptance/robotswarm-loaded-n4-e2e.py \
  --execute-production \
  --deployment-commit <sha-productivo-completo> \
  --credentials <archivo-privado-0600> \
  --chrome <chrome-visible> \
  --profile-root <directorio-temporal> \
  --output <informe-saneado.json>
```

El archivo de credenciales no forma parte de la evidencia. El informe final se
escribe con permisos `0600`, oculta identificadores privados y conserva hashes
SHA-256 de los documentos que sí pueden auditarse.

La secuencia supervisada es la siguiente:

1. Se comprueba que Chrome dispone de Media Source Extensions (MSE),
   `SourceBuffer` y soporte H.264 compatible con `hls.js` antes de solicitar un
   lease. Así, un navegador incapaz de reproducir el visor no consume una sesión
   productiva para terminar después en un falso diagnóstico de Gazebo.
2. Se crea la sesión N=4, se comprueba su roster y se vincula un único contenedor
   administrado al SHA esperado.
3. Se abre el HLS privado y se exige al menos un fotograma decodificado. El
   `gzclient` primario debe pertenecer al publicador de ese lease y usar los
   mismos `ROS_MASTER_URI`, `GAZEBO_MASTER_URI`, red y display privado que la
   sesión.
4. La sonda oficial instala la caja de 0,75 kg y ejecuta ensayos de capacidad con
   uno, dos y cuatro robots. Cada reemplazo queda cercado por generación,
   ausencia fresca y dos observaciones frescas de la pose final. A continuación
   mantiene esa carga para el GRF N=4.
5. Mientras el GRF cargado sigue activo, se abre un segundo `gzclient` temporal
   en el mismo display y se ejecuta el preflight oficial de GPU, FPS y RTF. El
   navegador mide HLS durante el mismo intervalo.
6. Durante `PUSH`, una captura debe quedar encerrada entre dos estados nuevos de
   la misma tarea, con masa fresca de 0,75 kg, roster completo y HLS por encima
   del mínimo. Solo después se validan los resultados y la limpieza.

La opción interna `--external-viewer-verified` no certifica una ventana por sí
misma. Solo se entrega a la sonda ROS después de que el supervisor haya
correlacionado el proceso real, el lease, el display y los dos masters. Ejecutar
el probe aislado con esa opción no constituye evidencia válida.

## 3. Criterios físicos y algorítmicos

En los ensayos de capacidad se envía la misma orden lineal de 0,16 m/s durante
12 segundos simulados. La prueba aprueba únicamente cuando se cumplen todos los
siguientes puntos:

- un robot desplaza la caja como máximo 0,05 m;
- las dos raíces en contacto directo la desplazan como máximo 0,06 m;
- cuatro robots la desplazan al menos 0,20 m;
- el avance con la flota es al menos cuatro veces el avance individual;
- cada robot avanza como mínimo 0,05 m;
- las dos raíces conservan contacto medido con la carga y los otros dos robots
  conservan contacto con su predecesor; y
- los tres ensayos mantienen RTF no menor que 2,90.

Las duraciones `push_duration_*` abarcan solo el comando positivo. Los ocho
mensajes de velocidad cero de la parada segura se publican después de capturar
los extremos. Por este motivo se admite una tolerancia de 0,25 s simulados, pero
no se suma la parada al intervalo que se utiliza para recalcular el RTF.

El segundo tramo ejecuta el algoritmo normal de transporte con la carga todavía
instalada. Se exige la secuencia no regresiva `SEARCH → APPROACH → PUSH → DONE`,
un único robot descubridor, aviso a sus tres compañeros, búsqueda inicial por
los cuatro, encuentro en las coordenadas comunicadas y contribución útil de toda
la flota. La prioridad es ocupar primero los dos contactos directos con la caja;
los demás robots empujan a través de cadenas contiguas de compañeros. El avance
físico mínimo hacia el objetivo se mantiene en 0,50 m.

El umbral que publica el runner es dinámico: parte de la distancia restante al
entrar en `PUSH`, resta la tolerancia de llegada y se limita por la configuración.
Puede aparecer como 0,4999 m por redondeo. El arnés valida ese eco con un epsilon
de 0,001 m, pero en una comprobación separada continúa exigiendo 0,50 m de avance
físico real. Ese medio metro es un mínimo de aceptación, no un límite de fuerza
ni una orden para detener el ensayo exactamente allí. Desde la liberación de
la PR #106, el formulario público envía `arrival_tolerance=0.25` por defecto y
permite ajustarlo entre 0,15 y 0,75 m. Una meta más distante y un margen menor
permiten observar un recorrido más largo sin alterar las propiedades físicas
de la caja. Es importante separar ambas decisiones: la tolerancia determina
cuándo el controlador considera alcanzado el destino, mientras que la masa y
la fricción determinan el esfuerzo necesario para mover el objeto.

## 4. Correcciones introducidas durante el diagnóstico

### 4.1. Reinstalación de la carga después de cada reset

El segundo diagnóstico productivo observó un marcador inicial de 0,75 kg y,
después, seis muestras de 0,25 kg. El `LOAD_RESULT_JSON` estructurado clasificó
el fallo como `model_placement_failed`: `reset_fleet()` había restablecido la
caja de práctica antes de colocar los robots del siguiente subensayo.

La sonda ahora recibe el XML seleccionado en cada `run_trial()`. Primero reinicia
la flota, después reinstala la misma caja cargada y solo entonces organiza los
empujadores. Para poder comprobar el SHA productivo anterior sin modificarlo, el
supervisor inspecciona la firma del probe desplegado: si esa versión aún no
acepta `payload_xml`, aplica una envoltura de compatibilidad a `reset_fleet()`;
si ya lo acepta, no la aplica. Esto evita tanto perder la carga como instalarla
dos veces al comparar versiones.

El marcador `LOADED_PAYLOAD_READY_JSON` se emite únicamente después de comprobar
en el XML una masa de 0,75 kg. Además, el supervisor exige una muestra posterior
del servicio de propiedades de Gazebo; el marcador por sí solo no abre la
barrera de carga.

### 4.2. Ejecución del preflight en una montura ejecutable

El directorio del lease situado bajo `/run/user/1000` estaba en una montura
`noexec`. Guardar allí el plugin permitía copiarlo, pero no mapearlo como objeto
compartido. Los archivos ejecutables se preparan ahora en un directorio privado
`/tmp/robotswarm-matrix-probe-*`, con modo `0700`. El sandbox de `bwrap` monta el
script y el plugin individualmente, en solo lectura, sobre `/viewer` antes de
realizar `--chdir`; su `/tmp` interno sigue siendo un `tmpfs` aislado.

La limpieza no elimina una ruta arbitraria. Antes de retirar el workspace se
comprueban padre, nombre, propietario, modo, lista cerrada de archivos y tipos
regulares. Si cualquiera de esas condiciones cambia, la aceptación falla de
forma cerrada.

### 4.3. Identidad del `gzclient` y espacios de PID

Un primer intento rechazó incorrectamente el visor primario porque comparó el
PID informado dentro del sandbox con el PID visto desde el host. Linux publica
ambas identidades en `NSpid`. La correlación actual recorre únicamente los
descendientes del publicador ligado a la sesión, exige el ejecutable esperado,
el argumento `/viewer/plugin.so`, el display correcto y acepta el PID reportado
solo si coincide con el PID del host o con una entrada de `NSpid`.

Para detectar procesos temporales abandonados ya no se reutiliza el PID que ve
un namespace. Cada preflight recibe un token aleatorio en
`ROBOTSWARM_ACTIVE_PROBE_TOKEN`; el token se hereda por Python, `bwrap` y
`gzclient`. El supervisor recorre `/proc`, compara también el *start tick* para
evitar reutilización de PID y aplica `SIGTERM` seguido de `SIGKILL` si queda un
proceso propio. La aceptación requiere que la búsqueda final quede vacía.

### 4.4. Compatibilidad con Python 3.8

La máquina de trabajo utiliza Python 3.8. Esta versión no ofrece
`subprocess.Popen(umask=...)` ni `str.removesuffix()`. El hijo que genera archivos
privados se inicia mediante un programa fijo de `sh` que ejecuta `umask 077` y
realiza `exec` con cada argumento separado; no evalúa texto procedente del hijo.
Para retirar `.json` se utiliza una comprobación de sufijo y un corte explícito.
Las dos rutas tienen regresiones focales para evitar que una validación local
funcione solo en un Python más reciente que el host real.

### 4.5. Marcadores JSON concurrentes

El monitor de masa y la sonda oficial comparten el mismo pipe. En el intento
`loaded-n4-hybrid-mse` dos escrituras se entrelazaron y produjeron un
`LOADED_GRF_ACTIVE_JSON` malformado. El intento se rechazó correctamente y no se
interpretó su contenido parcial como estado ROS.

Los marcadores vivos se serializan ahora sin `NaN`, incluyendo prefijo, JSON y
salto de línea en una sola escritura `os.write`. Se rechaza cualquier mensaje
mayor de 4096 bytes, límite adoptado para permanecer dentro de la escritura
atómica del pipe. El lector también detecta prefijos conocidos incrustados en un
JSON roto y conserva solamente longitud, hash y tipo de marcador; no guarda el
texto potencialmente sensible.

La escritura atómica del informe tiene un propósito distinto: cada actualización
se genera en un archivo temporal privado, se sincroniza y se sustituye mediante
`os.replace`. Así, una interrupción no deja un JSON final parcialmente escrito.

### 4.6. Presupuestos de espera, HLS y limpieza

El visor utiliza un solo presupuesto de 240 s desde el clic en «Abrir visor»
hasta el primer fotograma. El tiempo consumido esperando el lease se descuenta
antes de esperar video, en lugar de iniciar un segundo timeout completo. Si el
fotograma no llega, el informe registra un estado acotado de MSE, recursos HLS,
elementos de video y mensajes visibles, sin copiar tokens.

El preflight dispone de 45 s para calentamiento, medición y cierre. El timeout
exterior debe añadir al menos cinco segundos para observar su terminación; de lo
contrario, los argumentos se rechazan antes de crear una sesión. La medición HLS
de cinco segundos debe quedar por completo dentro del proceso cargado, de la
tarea GRF y del preflight.

Un intento que agotó la espera del fotograma eliminó contenedor y red, pero el
directorio del lease tardó más y el informe quedó como `cleanup-failed`. Esto es
un fallo real de la compuerta de limpieza, aunque la causa funcional fuera HLS.
En cambio, cuando el probe termina antes de emitir su postcheck estructurado, la
destrucción comprobada del contenedor inmutable demuestra que su tarea, roster y
proceso ya no existen. El informe conserva ambas ideas por separado:
`officialProbeTaskRosterClean=false` indica que faltó el postcheck del probe, y
`taskRosterClean=true` solo puede obtenerse entonces mediante
`containerAbsent=true`. De esta forma no se presenta un falso fallo de recursos,
pero tampoco se inventa una limpieza oficial que nunca fue observada.

### 4.7. Carrera entre borrado, aparición y posicionamiento de Gazebo

Un reintento posterior no alcanzó todavía la fase física: el modelo podía
desaparecer después de una respuesta exitosa de `delete_model`, reaparecer por
`spawn_model` y recibir un `set_model_state` correspondiente a otra observación.
El problema era temporal y no indicaba que 0,75 kg fueran demasiados. Confiar en
la respuesta individual de un servicio permitía aceptar un modelo antiguo o
solicitar reemplazos mientras Gazebo aún procesaba el anterior.

La sonda asigna una generación a cada intento. Antes de crear exige una muestra
fresca donde la caja no exista; después de crear y posicionar requiere dos
muestras frescas consecutivas con el modelo presente en la pose esperada. Un
resultado de una generación anterior no puede abrir la barrera de la siguiente.
Solo se autoriza otro reemplazo —hasta tres en total— cuando la observación
autoritativa demuestra desaparición o `model missing`. Un error permanente, un
cierre solicitado o la falta estable de convergencia terminan con una categoría
explícita en vez de entrar en un bucle de borrado y creación.

### 4.8. Diagnóstico acotado del fallo de colocación

El primer borrador conservaba todos los mensajes de cada intento. En una falla
prolongada eso podía crecer sin límite, repetir respuestas voluminosas e incluir
campos innecesarios. El informe guarda ahora como máximo 16 entradas saneadas,
el número total observado y un indicador de truncamiento. Las huellas se
calculan únicamente para categorías permitidas; no se copia la respuesta cruda
de Gazebo. El timeout de presencia estable tiene su propia clasificación, de
modo que el supervisor puede distinguir una carrera recuperable de un modelo
que nunca se estabilizó.

### 4.9. Marcador de destino y decisión sobre la física

Para que el espectador conozca la intención antes del empuje, `swarm_arena.world`
contiene una huella magenta y una caja translúcida de 0,4 m. Es un modelo
cinemático, sin gravedad ni colisión: se ve en Gazebo y, por tanto, en HLS, pero
no empuja ni bloquea a los robots. No se declaró `static` porque Gazebo Classic
no actualiza de forma fiable ese visual después de `set_model_state`.
Transporte publica su pose mediante un publicador latched y comprueba después
`/gazebo/model_states`. La telemetría indica si el comando
se publicó, si la posición quedó sincronizada y cuál fue la coordenada
observada. Un fallo del marcador se muestra como advertencia y no cambia el
resultado físico de la tarea.

Se decidió conservar 0,75 kg y `mu=mu2=0.25`. Reducir esos valores haría que la
caja recorriera más distancia con menos esfuerzo, pero también eliminaría el
contraste medido entre uno, dos y cuatro robots. El perfil de práctica de 0,25
kg y `mu=mu2=0.05` permanece disponible para demostraciones rápidas y es el que
usa el transporte normal. Para mostrar un movimiento más largo se utiliza un
destino suficientemente lejano y el margen de llegada de 0,25 m; el perfil
cargado sigue respondiendo, por separado, a la pregunta académica de si la
flota aporta una capacidad que un Burger aislado no posee.

### 4.10. Cierre de recuperación durante el apagado

La revisión final encontró un caso distinto de la carrera normal: si ROS
entraba en shutdown justo después de detectar `model missing`, la clasificación
podía abrir otro ciclo de `delete`/`spawn`/`set_model_state`. Esta actividad no
podía ayudar a una prueba que ya se estaba cerrando y aumentaba el riesgo de
dejar una vida tardía de la caja.

La sonda comprueba ahora `stop_requested` y `rospy.is_shutdown()` antes de cada
servicio, al iniciar el bucle de pose, antes de clasificar una ausencia como
recuperable y entre las etapas de un reemplazo. El apagado termina de inmediato:
no crea otra generación ni toca Gazebo. Las regresiones cubren shutdown antes
del primer servicio, durante una recuperación y después de una etapa ya
completada.

### 4.11. Instantáneas vivas completas y odometría finita

La revisión de `377a0e3` encontró que recorrer `name` y `pose` con `zip` podía
aceptar un `ModelStates` truncado. El callback refrescaba el timestamp del
mensaje, pero podía conservar la pose anterior de la caja; así, el control veía
geometría vieja con una marca de frescura nueva. Ahora se exige igual longitud,
se construye la instantánea completa fuera del estado compartido y solo se
realiza un commit atómico cuando todas las poses son válidas. Un truncamiento
vacía carga, obstáculos y marcador, invalida el timestamp y produce `FAILED`
correlacionado con la tarea activa.

La odometría de transporte comprueba además `x`, `y`, yaw y los cuatro valores
crudos del cuaternión. Este último paso es necesario porque un cuaternión con un
componente `Inf` podía atravesar una conversión trigonométrica y producir por
casualidad un yaw finito. La muestra inválida ya no actualiza pose, velocidad ni
frescura.

### 4.12. Publicación por lote y limpieza ante excepciones

Validar cada robot justo antes de su propio publish todavía permitía que los
primeros se movieran si un elemento posterior del lote era inválido. El
transporte reúne y valida ahora todos los `Twist` antes del primer comando en
`SEARCH`, `APPROACH` y `PUSH`; comprueba finitud, ejes compatibles y límites
físicos del Burger. Un solo fallo detiene a todos y publica `FAILED` con el
`task_id` y la época de control originales. Las excepciones inesperadas del
ciclo siguen el mismo camino, en vez de depender de alcanzar el epílogo normal.

Durante `APPROACH` se utiliza un acumulador temporal para formar el lote. Su
retiro está dentro de `finally`, de modo que una excepción de evitación no deja
un buffer activo que capture comandos de un ciclo posterior. El cierre normal
correlacionado continúa distinguiéndose del fallo y no se etiqueta como
`FAILED`.

### 4.13. Frescura al final del ciclo de control

I-127 reprodujo una ventana posterior: el ciclo validaba `ModelStates` al
entrar, pero un cálculo inducido de 1,0 s excedía el timeout de 0,75 s antes de
publicar. La fase `SEARCH` alcanzó a emitir 0,12 m/s con una instantánea ya
vencida. `4450c13` comprueba dentro de la barrera final la frescura de Gazebo y
de la odometría de toda la flota antes de aceptar cualquier lote no nulo. La
misma exclusión se mantiene hasta el último `Twist` y el commit del estado de
`ObstacleAvoidance`; por ello el lote se publica completo o no comienza, incluso
si el reloj cruza el timeout entre dos robots. Las rutas de publicación directa
usan el mismo gate. Los comandos cero permanecen permitidos con datos vencidos,
porque bloquear una parada degradaría el fallo seguro.

La regresión adelanta el reloj entre el primer y el segundo robot y verifica que
no aparece un pulso parcial. El corte aislado de `4450c13` aprobó 172/172 pruebas
de lifecycle y 487/487 globales, además de `py_compile` con Python 3.8 y
`git diff --check`. Estos resultados cerraron I-127 en código. El freeze y la
imagen posteriores se registran en la sección 5; en ese corte la prueba física
aún no se había ejecutado.

### 4.14. Correlación de odometría durante la planificación de seguimiento

La revisión posterior abrió I-128, fuera del algoritmo de carga pero dentro del
mismo candidato de liberación. El planificador asíncrono de seguimiento ya
correlacionaba los obstáculos, no las posiciones y yaw de la cadena usados para
calcular la ruta. En la reproducción se bloqueó el solver, se desplazó al líder
mediante odometría válida de `(0; 0)` a `(3; 3)` y luego se liberó el cálculo. El
plan antiguo se confirmó con `anchor=(0; 0)`, `active=true` y
`path_anchor_ready=true`; el primer comando angular fue aproximadamente
−0,0625 rad/s, aunque la ruta no había sido validada desde la pose nueva.

`07da8f4` correlaciona dos veces las posiciones y yaw de toda la cadena, con
tolerancias explícitas al jitter. Un cambio material descarta el resultado,
mantiene velocidad cero y solicita un nuevo plan. I-128 queda cerrada en código;
el freeze y el build local posterior se describen en la sección 5.

### 4.15. Fronteras finales de formación y seguimiento

I-129 e I-130 reprodujeron en formación y seguimiento la ventana temporal ya
cerrada para transporte. En formación, la marca era 10,0 s, el timeout 0,75 s y
el reloj avanzó de 10,0 a 11,0 durante el ciclo; aun así se publicaron 0,22 m/s y
la tarea siguió activa. El caso manual equivalente de seguimiento N=1 publicó
0,1 m/s y conservó `active=true`. Ambos controladores deben volver a comprobar
la frescura dentro de la frontera final del lote, antes del primer comando no
nulo. `07da8f4` coloca esa comprobación literalmente después de la geometría en
formación y justo antes del lote en seguimiento, manteniendo la decisión y la
publicación bajo la misma frontera.

I-131 entregó a formación un `ModelStates` truncado con
`name=['moving_box']` y `pose=[]`. El callback dejó una escena vacía y el
controlador avanzó 0,22 m/s hacia un objetivo que el modelo bloqueaba. I-132
inyectó un cuaternión crudo con `Inf`: la conversión produjo yaw finito 2,356,
la muestra fue aceptada tanto en modelo como en odometría y se publicaron
0,22 m/s lineales y −1,5 rad/s angulares. El cierre debe invalidar de forma
atómica longitudes distintas y comprobar los cuatro componentes del cuaternión
antes de convertirlos a yaw. La invalidación persiste hasta recibir una
instantánea posterior completa; los modelos configurados se filtran por el
mundo activo.

La revisión independiente añadió I-133 e I-134. El validador de formación
comprobaba finitud y límites de los seis componentes, pero permitía valores
pequeños en `linear.y/z` o `angular.x/y`, ejes que la base Burger no soporta.
Además, un nombre configurado duplicado en `ModelStates` podía limpiar una
invalidación si una pose NaN aparecía antes de otra válida, o dejar dos poses
ambiguas para el mismo modelo. `07da8f4` rechaza los ejes no holonómicos y hace
fallar cerrado los duplicados en formación, seguimiento y transporte; solo una
instantánea posterior completa y unívoca puede recuperar el control.

La primera iteración del grupo aprobó 95/95 pruebas de
seguimiento+formación, pero fue superada por los dos hallazgos de la revisión
independiente. El delta final integrado aprobó 6/6 regresiones focales y 497/497
globales en 109,067 s, además de `py_compile` con Python 3.8.10 para tres scripts
y tres tests y `git diff --check`. Estos son resultados aislados de código; la
repetición exacta y la imagen se registran abajo y la aceptación física continúa
pendiente.

### 4.16. Encuadre completo del marcador de destino (I-135)

La primera ejecución visible sobre la imagen local posterior a I-134 confirmó
que `target_marker` se publicaba y quedaba sincronizado en `x=-3.5`, `y=-4.0`.
Sin embargo, las capturas de aproximación, búsqueda y final mostraron la huella
magenta recortada por el borde inferior. Una revisión visual independiente y
una captura manual con *zoom out* permitieron separar las causas: el marcador
existía y estaba en la coordenada correcta; el defecto era el encuadre inicial
de la cámara, no la telemetría ni el algoritmo de transporte.

La pose anterior, `0 -12 10 0 0.7 1.5708`, no cubría de manera completa el área
de trabajo de `-4` a `4` con el viewport real de 990×334. Se sustituyó por
`0 -14.4 12 0 0.72 1.5708` y se añadió una regresión que fija esa pose junto con
las propiedades cinemáticas del marcador. El mundo aprobó 8/8 pruebas y
`git diff --check`. Después se reconstruyó la imagen y se comprobó en Gazebo
visible que el marcador completo permaneciera dentro del encuadre en las cuatro
esquinas del área: `(4; 4)`, `(-4; 4)`, `(4; -4)` y `(-4; -4)`. I-135 queda
cerrada en el código integrado por la PR #106. Posteriormente, CI aprobó ese
cambio, el merge `1448a31bbbbfd77588bada109947098cc95d9dda` se desplegó y la
repetición visible postdeploy confirmó el marcador en el visor productivo. La
evidencia concreta de esa comprobación se presenta en la sección 8; los datos
que siguen en esta subsección conservan la trazabilidad de la prueba local que
precedió al despliegue.

La evidencia temporal de cuatro esquinas está en
`/tmp/robotswarm-local-final-camera-20260722T0454Z/`:

- nordeste: SHA-256
  `75c53cf725a69c20568d00a529754d463bb79957c2e10e904d4b46e2c22ee839`;
- noroeste: SHA-256
  `84750bdbb15eddb1babaeffedf10205de84a816fd6e474e13a28f35a887152ca`;
- sudeste: SHA-256
  `ade73f6e43125f0945a60606d3db7cb6e10538abc19b2d3fb18e1ead1b848e14`;
- sudoeste: SHA-256
  `86f32de8ad0628c0fae1d97b53a1ae3ed9dcf86f73fa753ec5c89a886ea48f51`.

El gate utilizó `D3D12 (NVIDIA GeForce RTX 3080)`, midió 57,279 FPS de
cámara, 58,791 eventos de posrenderizado por segundo y RTF 2,997. La imagen
resultante completó catkin al 100 %, tiene ID
`sha256:e17579ed83e0c37a9ff9b03817652aeb935573b307801ddc5863d29f2a92ae0d`,
tamaño 4.231.381.706 bytes y fecha de creación
`2026-07-22T04:53:51.679721236Z`.

## 5. Evidencia histórica y local

Una corrida local histórica, denominada v11, sí aprobó el perfil cargado antes
del candidato actual. Sus ensayos de uno, dos y cuatro robots desplazaron la
caja 0,0070 m, 0,0340 m y 1,0424 m. Los RTF recalculados fueron 2,9969, 2,9962 y
2,9975, y la ganancia frente al robot individual fue 148,9143 veces. Las cuatro
unidades permanecieron conectadas mediante dos raíces y dos compañeros.

En esa misma corrida, el GRF llegó a `DONE/completed` después de que `tb3_1`
avisara a los otros tres robots. Se registraron 1598 muestras con los cuatro
contribuyentes útiles, 0,5002 m de avance hacia la meta, eficiencia 0,9946 y RTF
exterior 2,9756. La sonda visible concurrente midió 58,816 FPS de cámara, 58,831
eventos de posrenderizado por segundo y RTF 2,996 en una RTX 3080.

Estas cifras son útiles como referencia física, pero no cierran producción: la
salida original no quedó asociada al SHA productivo actual ni satisface las
correcciones posteriores del instrumento. Tampoco se reutilizan sus capturas
como si pertenecieran al gate vigente.

En el freeze provisional anterior a I-117–I-125 aprobaron 81/81 pruebas del
conjunto loaded y 170/170 de ROS/mundo. El corte integral confirmó además ROS
461/461 y los siete arneses de aceptación 241/241; dentro de estos últimos, el
arnés cargado conserva 49/49 contratos y la matriz ROS 61/61. Son pruebas
locales del instrumento; no demuestran por sí mismas que la carga haya sido
transportada en el sitio desplegado. La rama parte de la base productiva
`9f49e17435a1ddd6b93b7834b2896d57059616fe`. El código está integrado
localmente hasta `07da8f4`; después de `377a0e3`, seguimiento+lifecycle aprobó
206/206 y la ejecución global ROS aislada 473/473, además de Python 3.8 y
`git diff --check`. I-126 se integró como `568979d` tras aprobar 40/40 pruebas
de seguimiento y 483/483 globales; I-127 se integró como `4450c13` tras aprobar
172/172 de lifecycle y 487/487 globales. I-128–I-134 quedaron integradas como
`07da8f4` tras 95/95 de seguimiento+formación, 6/6 focales finales y 497/497
globales aisladas. Antes del ajuste de cámara, el árbol principal exacto aprobó
497/497 ROS en 109,460 s; este valor se conserva como trazabilidad del corte
pre-cámara. Ese corte también aprobó 241/241 contratos —incluidos 49/49 loaded
y 61/61 de matriz— y 8/8 pruebas de física del mundo, todos con RC=0. También
aprobaron Python 3.8.10, workflows, Compose, `diff-check` y el escaneo de
secretos. La imagen entonces etiquetada `robotswarm-ros:local-final-safe`
completó catkin al 100 %, con ID
`sha256:02fbd5c2302d3af0eb9e543af04bb4507292786b7155b39b54e9d294b27f4864`
y tamaño 4.231.381.324 bytes.

Sobre esa imagen se realizaron dos corridas locales `transport_grf_n4`, ambas
con N=4, proceso visible y RC=0:

| Corrida | Origen de la tolerancia | Resultado | Avance del objeto | Distancia final | RTF | Participación y seguridad |
| --- | --- | --- | ---: | ---: | ---: | --- |
| Compatibilidad heredada | Valor omitido; `arrival_tolerance=0.50` heredado | PASS | 0,5022 m | 0,4978 m | 2,9965 | 4/4 contribuidores útiles; 0 colisiones |
| Parámetro equivalente a la interfaz | `arrival_tolerance=0.25` inyectado solo en memoria en el arnés; sin modificar el repositorio | PASS | 0,7535 m | 0,2465 m | 2,9963 | 4/4 contribuidores útiles; 0 colisiones |

La repetición con tolerancia 0,25 registró una fracción de muestras con los
cuatro robots útiles de 0,8707, velocidad lineal máxima de 0,1664 m/s y
aceleración máxima en la ventana de medición de 0,6281 m/s². `tb3_1` encontró
el objeto y avisó a `tb3_0`,
`tb3_2` y `tb3_3`; hubo un máximo simultáneo de cuatro robots buscando y cuatro
en encuentro. El marcador se publicó y sincronizó en `(-3.5; -4.0)`. El cambio
temporal del parámetro reprodujo lo que enviaría la interfaz sin convertir una
edición del arnés en parte del candidato.

Gazebo fue visible, no headless, sobre `D3D12 (NVIDIA GeForce RTX 3080)`. El
preflight concurrente midió 57,182 FPS de cámara, 58,795 eventos de
posrenderizado por segundo y RTF 2,997. Los logs y capturas temporales están en
`/tmp/robotswarm-local-final-visible-20260722T0444Z/`; sus PNG principales son:

- `transport-approach.png`, SHA-256
  `fa356ca8b084ee62e093f5993b1134a5b65ac2a857e492096e1906a0677c19d6`;
- `tolerance-025-search.png`, SHA-256
  `d27774ed3560056919728b83dce8386a28187ba4f0e9d7234ac8959e706aff7b`;
- `tolerance-025-done.png`, SHA-256
  `044573e7ca9a43d347947e51e2db6560e364006b737f4242f7c49c445ca049b4`;
- `tolerance-025-zoomout2.png`, SHA-256
  `9e88ce8444111379ce740c2c7ae4fb01e20c516dcc585f5da4c4c12fe6e1e6c7`.

Las tres primeras imágenes permitieron reproducir I-135 y la cuarta confirmó
que el marcador no faltaba, sino que quedaba fuera del encuadre predeterminado.
Por ello la imagen `02fbd5c2…f4864` se conserva como evidencia exacta de las
dos corridas algorítmicas, pero está superada únicamente en el encuadre visual
por la imagen `e17579ed…ae0d` descrita en 4.16. El nuevo build cerró la
comprobación local de las cuatro esquinas. Después del ajuste, el rebuild y la
imagen nueva, la suite ROS exacta se repitió y aprobó 497/497 en 109,592 s con
RC=0. Esa ejecución fue el freeze final local de I-128–I-135 y quedó
supersedida por `6f1af927…4cb5` y 576/576 pruebas de I-136–I-140; los 109,460 s
anteriores permanecen únicamente como trazabilidad del corte pre-cámara.

Para comprobar que el cambio de cámara no alteró el comportamiento, se repitió
el escenario físico visible exacto sobre la imagen `e17579ed…ae0d`. Se inyectó
`arrival_tolerance=0.25` solo en memoria y se conservó la flota al terminar; no
se modificó el repositorio ni se atribuye a esta corrida una prueba de borrado
de recursos. El arnés terminó PASS con RC=0: el objeto avanzó 0,7523 m, quedó a
0,2477 m del destino y mantuvo RTF 2,9962, con 4/4 robots útiles, fracción de
muestras con toda la flota útil de 0,9355 y cero colisiones. La velocidad lineal
máxima fue 0,1649 m/s y la aceleración máxima en la ventana de medición fue
0,7883 m/s², por debajo del límite de 1 m/s².

`tb3_1` descubrió el objeto y avisó a `tb3_0`, `tb3_2` y `tb3_3`. El máximo
simultáneo fue de cuatro robots tanto en búsqueda como en encuentro, y el
marcador quedó publicado y sincronizado en `(-3.5; -4.0)`. Las transiciones se
observaron a 0,278 s de pared para `SEARCH`, 39,191 s para `APPROACH`, 107,725 s
para `PUSH` y 131,991 s para `DONE`, medidos desde el inicio de la corrida; son
tiempos acumulados de transición, no duraciones independientes de cada fase.

La evidencia está en
`/tmp/robotswarm-local-final-camera-20260722T0454Z/`. El log saneado
`transport-n4-tolerance-025-final.log`, SHA-256
`3ad8ad2b65b7589f34d52f716990d8675e57cef9474d9679af9407a81612ac43`,
acredita las métricas terminales. La única captura aceptada de esta corrida es
`final-n4-search.png`, SHA-256
`d036d9814b893441f2c57be52a4ab58415339f85bfeba6055177820417211f11`:
muestra completos el objeto y el marcador fantasma, además de los cuatro robots
en búsqueda dentro del nuevo encuadre. `final-n4-push.png` y
`final-n4-done.png` se excluyen expresamente: fueron capturadas después de que
la limpieza o el reset alteraran la escena y, pese a sus nombres, no acreditan
visualmente ni la fase `PUSH` ni la pose final. La conclusión física terminal
procede del log correlacionado, no de esas dos imágenes.

Ninguna de estas corridas locales incluye el gate completo de capacidad con la
caja de 0,75 kg ni se reinterpreta como prueba postdeploy. El escenario N=4 con
la física normal sí se repitió después de desplegar la PR #106, junto con N=2,
N=3 y N=10, como se registra en la sección 8. En cambio, la repetición física
del perfil cargado sobre `1448a31…d9dda` continúa pendiente. La última
aceptación productiva completa de 0,75 kg sigue perteneciendo al release
histórico `fbef23eaae2b1b1d5be51ad3fa03e0298239289a` y se documenta en las
secciones 6 y 7.

## 6. Diagnóstico productivo del 21 de julio de 2026

Todos los intentos de esta tabla usaron Chrome 150 visible, una sesión pública
nueva N=4 y el release `fbef23e…9289a`. Un intento rechazado sirve para localizar
la causa, pero nunca se suma a la aceptación.

| Intento | Observación | Clasificación |
| --- | --- | --- |
| `loaded-n4-final` | El visor HLS entregó 30,9 FPS; el arranque de Gazebo midió 49,59 FPS y RTF 2,996. El probe terminó antes de confirmar la barrera de 0,75 kg. | Rechazado; limpieza completa. |
| `loaded-n4-diagnostic` | Se obtuvo un `LOAD_RESULT_JSON` fallido y siete muestras de masa. | Rechazado como fallo del probe y de reemplazo/visibilidad de la carga. |
| `loaded-n4-diagnostic-2` | Hubo una muestra de 0,75 kg seguida de seis de 0,25 kg. | Rechazado; `model_placement_failed` permitió localizar el reset que restauraba la caja de práctica. |
| `loaded-n4-dom-controls` | La sesión llegó a `Ready`, pero no apareció un fotograma HLS dentro del presupuesto. | Rechazado como `cleanup-failed`: contenedor y red desaparecieron, pero el runtime del lease no se confirmó ausente a tiempo. |
| `loaded-n4-hybrid-mse` | MSE/H.264, los clics auditados y el visor aprobaron; el marcador activo llegó entrelazado. | Rechazado por `LOADED_GRF_ACTIVE_JSON` malformado; limpieza completa. |
| `loaded-n4-atomic-markers` | La escritura atómica eliminó el marcador malformado, pero el probe terminó antes de iniciar el GRF N=4. | Rechazado; limpieza completa. El diagnóstico estructurado de ese punto todavía necesitaba conservarse en todos los caminos de excepción. |
| `loaded-n4-classified` | El HLS inicial midió 31,0 FPS y Gazebo 49,563/49,990 FPS con RTF 2,996. Después de terminar el payload, el arnés volvió a pedir HLS cuando ya había vencido el TTL intencional de cinco minutos del lease. | Rechazado por un gate HLS final redundante; limpieza completa. |
| `loaded-n4-post-ttl-fix` | Conservó las mediciones HLS tomadas dentro del TTL, completó capacidad, GRF, restauración y limpieza sobre el mismo release. | **Aprobado** con `status=passed`, `success=true` y limpieza completa. |

Los intentos rechazados se conservan porque explican cómo se llegó al instrumento
actual; sus resultados no se mezclan entre sí. La aprobación se apoya solamente
en `loaded-n4-post-ttl-fix`, que produjo su propio conjunto completo de
documentos, hashes y capturas correlacionadas.

## 7. Aprobación productiva: `loaded-n4-post-ttl-fix`

La corrida se ejecutó entre las 22:21:26 y las 22:27:32 UTC del 21 de julio de
2026. El informe saneado terminó con `status=passed`, `success=true` y quedó
vinculado al release productivo
`fbef23eaae2b1b1d5be51ad3fa03e0298239289a`. La imagen del contenedor estaba
inmovilizada por ese SHA, la sesión N=4 llegó a `Ready` y el display, el lease,
la red privada y los masters ROS/Gazebo pertenecían a la misma sesión.

### 7.1. Video y rendimiento visible

Chrome 150 se ejecutó con GPU y en modo visible. El HLS entregó entre 30,0 y
30,8 FPS en las comprobaciones de apertura, captura y cierre del intervalo útil.
Durante la coincidencia entre carga, GRF y preflight se midieron 152 fotogramas
en 5,051 s: 30,094 FPS presentados y decodificados, sin fotogramas descartados.
Las lecturas inmediatamente anterior y posterior a la captura de `PUSH` fueron
ambas de 30,0 FPS, por encima del mínimo de 27,0 FPS.

El `gzclient` primario informó 49,548 FPS de cámara, 50,034 eventos de
posrenderizado por segundo y RTF 2,996. El preflight oficial, ejecutado mientras
el GRF cargado seguía activo, identificó el adaptador `D3D12 (NVIDIA GeForce RTX
3080)` y midió 58,469 FPS de cámara, 62,489 eventos de posrenderizado por segundo
y RTF 2,996. Por tanto, no se utilizó renderizado headless ni un renderer por
software.

Al terminar el payload, el lease ya había alcanzado su TTL deliberado y el panel
mostró el rechazo temporal esperado. Esta observación no invalida el gate: el
video se midió dentro del TTL, durante el proceso cargado y a ambos lados de la
captura correlacionada. El arreglo consistió en registrar el estado final del
lease sin volver a exigir una reproducción que, por diseño, ya debía estar
revocada.

### 7.2. Capacidad física y comportamiento GRF

Con el mismo perfil de 0,75 kg, los ensayos de uno, dos y cuatro robots
desplazaron la caja 0,0070 m, 0,0354 m y 1,0836 m, respectivamente. La ganancia
de la flota sobre el robot individual fue 154,8 veces. Los RTF recalculados de
los tres ensayos fueron 2,9951, 2,9975 y 2,9941; los cuatro robots conservaron la
estructura prevista de dos raíces en contacto con la carga y dos compañeros.

En el tramo algorítmico, un descubridor emitió el aviso a la flota completa. Los
cuatro robots registraron movimiento de búsqueda, los cuatro completaron el
encuentro y los cuatro contribuyeron al empuje. Se obtuvieron 1621 muestras GRF,
de las cuales 1618 confirmaron simultáneamente a toda la flota como útil. La caja
avanzó 0,5001 m hacia la meta, con eficiencia 0,9946 y RTF exterior 2,9942. Estos
valores satisfacen el mínimo físico de 0,50 m y demuestran búsqueda, aviso,
reunión y transporte colaborativo en una sola tarea.

### 7.3. Limpieza y conjunto de evidencia

La limpieza terminó con `cleanup.complete=true`. El postcheck oficial observó la
tarea terminal, roster vacío, ausencia de modelos `tb3_*` y restauración de la
caja de práctica de 0,25 kg. También quedaron ausentes los hijos acotados, el
preflight temporal, su workspace, el visor, el publicador, el lease, el
contenedor, la red, Chrome y su perfil efímero.

El hash del conjunto saneado es
`c9095a31ec596c4dccaf12d3ef3beb109ddd652399b44962215ffc94011a7dac`.
El [reporte saneado](assets/commissioning-2026-07/final-fbef23e/carga-n4-reporte.json)
y las tres PNG pertenecen a esta misma corrida. Se inspeccionaron para excluir
correos, UUID, tokens y rutas internas antes de incorporarlos al manifiesto:

- [Antes del probe cargado](assets/commissioning-2026-07/final-fbef23e/carga-n4-antes.png), 131134 bytes, SHA-256 `4579c14a4020fb2e3527ed3ac18926f7eaf712ef60e641938dd164084903c89a`;
- [Durante el `PUSH` cargado](assets/commissioning-2026-07/final-fbef23e/carga-n4-durante-push.png), 149175 bytes, SHA-256 `7a5be6cb6e57cabfab83a94c340a1920620e5496973822bcbed581dfb079abaa`;
- [Después del probe, con la regresión UTC visible](assets/commissioning-2026-07/final-fbef23e/contador-utc-antes.png), 58771 bytes, SHA-256 `fdfbef8f65b81178508f281130ea1ebf851aa9144bac9665a3024d77cf022206`.

Este resultado aprueba el transporte cargado N=4 del release productivo
`fbef23e…9289a`. El candidato posterior ya fue publicado, pero el procedimiento
cargado de esta sección todavía debe repetirse sobre ese SHA nuevo. Por ello,
esta aprobación histórica no se extiende de manera automática al despliegue
actual.

## 8. Liberación de la PR #106 y comprobación posterior

### 8.1. Artefacto desplegado

La [PR #106](https://github.com/SkateWarp/RobotSwarm/pull/106) integró el
control escalable del enjambre, la tolerancia de llegada de 0,25 m y el marcador
fantasma. Los checks de CI finalizaron correctamente y el merge produjo el SHA
`1448a31bbbbfd77588bada109947098cc95d9dda`. El despliegue GPU utilizó de forma
exacta la imagen:

```text
robotswarm/ros-noetic:git-1448a31bbbbfd77588bada109947098cc95d9dda-29893535071-1
```

El ID observado de la imagen comienza por `d38cdd616eb1`. Esta relación entre
merge, etiqueta e imagen evita atribuir a producción una prueba ejecutada sobre
un build local diferente. La referencia de reversión anterior al cambio se
conserva como `rollback/pre-formation-ghost-9f49e17`, que apunta a `9f49e17`.

El despliegue no unificó los dos perfiles físicos. El escenario normal conserva
la caja de 0,25 kg con `mu=mu2=0.05` y el formulario usa una tolerancia inicial
de 0,25 m. El gate cargado continúa usando, de manera explícita, 0,75 kg con
`mu=mu2=0.25`. Así, el recorrido más largo de la demostración normal no se
obtiene aligerando silenciosamente la carga académica.

Como línea base previa al despliegue, la repetición local visible N=4 con el
perfil normal avanzó 0,7523 m, terminó a 0,2477 m del destino y sostuvo RTF
2,9962. Los cuatro robots aportaron movimiento útil y no se registraron
colisiones. Ese resultado fue reproducible con el marcador completo dentro del
encuadre, pero sigue identificado como evidencia local, no como aceptación
cargada postdeploy.

### 8.2. Evidencia visual antes y después

La comparación versionada permite comprobar el cambio sin depender de las
carpetas temporales de aceptación:

- [Antes, sobre `9f49e17`: el destino no se representaba en Gazebo](assets/commissioning-2026-07/final-1448a31/marcador-antes-9f49e17.png).
- [Después, sobre `1448a31`: el marcador magenta aparece en el visor productivo N=10](assets/commissioning-2026-07/final-1448a31/marcador-despues-1448a31.png).
- [Búsqueda N=4 local con el objeto y el marcador dentro del encuadre](assets/commissioning-2026-07/final-1448a31/busqueda-n4-local.png).

La captura «después» confirma que el modelo fantasma se publicó en producción:
la huella magenta y la caja translúcida indican al espectador el destino sin
añadir colisiones, gravedad ni fuerzas al mundo. La captura N=4 complementa la
explicación del encuadre, pero no se presenta como imagen productiva. Los hashes,
la procedencia y el uso válido de cada PNG están registrados en el
[manifiesto del conjunto](assets/commissioning-2026-07/final-1448a31/README.md).

### 8.3. Matriz de transporte con física normal

La comprobación posterior al despliegue ejecutó el transporte normal con varios
tamaños de flota. Su estado es el siguiente:

| Robots | Resultado postdeploy | Observación |
| ---: | --- | --- |
| 1 | Aprobado con el arnés corregido | `DONE`, 0,5013 m, RTF 2,9962, utilidad 1/1, cero colisiones, sonda NVIDIA 58,711 FPS, HLS 30,164 FPS y limpieza completa sobre `1448a31`. |
| 2 | Aprobado | El escenario de transporte completó sus criterios con la flota exacta. |
| 3 | Aprobado | El escenario de transporte completó sus criterios con la flota exacta. |
| 4 | Aprobado | El escenario de transporte completó sus criterios con la flota exacta. |
| 10 | Aprobado | El objeto avanzó 0,5036 m, quedó a 0,4964 m del destino y mantuvo RTF 2,9672; participaron 10/10 robots y se observaron cero colisiones. |

El rechazo N=1 inicial no demostró un fallo de ROS, del video ni del transporte.
Fue una pérdida de evidencia causada por el orden temporal del instrumento: el
informe se recogía demasiado tarde. La corrección copia y valida el
`render-report` inmediatamente después de la sonda gráfica, antes de esperar el
final de ROS. La repetición productiva utilizó esa versión del arnés contra la
misma imagen `1448a31` y aprobó. Terminó antes de necesitar una renovación del
lease; por ello prueba la preservación temprana y el cleanup, mientras las ramas
de renovación permanecen acreditadas por contratos.

Los cinco tamaños N=1, N=2, N=3, N=4 y N=10 tienen ahora un resultado aprobado
del transporte normal sobre `1448a31`. Esto no cierra por sí solo la matriz ROS:
las formaciones corregidas conservan su propio proceso de entrega y validación.

### 8.4. Límites de la aceptación actual

Después de `1448a31` aún no se ha ejecutado el gate completo de capacidad con
la caja de 0,75 kg y `mu=mu2=0.25`. En consecuencia, no se afirma una nueva
aceptación cargada para ese release. El último resultado cargado completo sigue
siendo `loaded-n4-post-ttl-fix` sobre `fbef23e…9289a`, descrito en la sección 7.

La evidencia actual permite afirmar tres hechos más acotados: el marcador es
visible en producción, la tolerancia normal de 0,25 m permite observar un
recorrido mayor sin cambiar la física y el transporte normal aprobó con 1, 2, 3,
4 y 10 robots. Quedan fuera de este cierre el gate cargado post-`1448a31` y el
cierre final de los escenarios de formación. Esta delimitación evita mezclar
una mejora visual o de usabilidad con una aceptación física que todavía no se
ha repetido.
