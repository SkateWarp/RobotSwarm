# Evidencia saneada de I-147

I-147 se abrió el 24 de julio de 2026 durante la aceptación posterior al
despliegue `e3dc7ad37c7525c53f24f9601102b32152103a5d`. El frontend, el backend
y el worker GPU estaban publicados con ese mismo SHA. Los reportes completos
permanecen fuera del repositorio, con permisos `0600`; este documento conserva
solo datos operativos que no identifican usuarios, sesiones ni leases.

## Formación S/N=10: intento rechazado

La matriz visible aprobó triángulo N=3, cuadrado N=5, A/N=7, V/N=8 y diamante
N=9. La última fila agotó sus 85,4005 s de pared todavía en `forming`. Ocho
robots recorrieron entre 1,82 y 4,15 m, mientras `tb3_6` y `tb3_7` recorrieron
solo 0,1403 y 0,1401 m porque pertenecían a lotes posteriores todavía
retenidos. Entre los robots ya liberados, `tb3_3` permaneció a 0,4616 m de su
slot. La separación mínima fue 0,3944 m, el despeje mínimo de obstáculo
0,2274 m, el RTF 2,988 y no hubo colisiones.

El resultado se rechazó correctamente: no se rebajó el error máximo de 0,12 m
ni se confundió una tarea `forming` con una formación válida. El arnés
preservó el resultado ROS aun cuando la comprobación visual se detuvo antes
del muestreo activo, y completó el cleanup. El reporte tiene SHA-256
`7e41dd38f62a3b97240a256d7111e3892e7e5642e8b50fd7990f2f5aa5375419`.

Una sesión fresca, sin cambiar código ni parámetros, sí aprobó S/N=10:
75,0317 s continuos en `moving`, error máximo 0,0951 m, separación mínima
0,4029 m, despeje 0,2518 m, RTF 2,9846, cero colisiones y limpieza completa.
Su SHA-256 es
`8b79c44f462e3a5756b7c4f15ea7a60d701f5acad65931fb2551b54e6f57a4e0`.
La repetición demuestra que la geometría es factible, pero no elimina el
defecto intermitente del primer intento.

## Diagnóstico y corrección

El secuenciador mueve primero los robots que bloquean corredores y conserva
detenidos los lotes posteriores. Este principio evitó colisiones, pero no
tenía una salida de liveness cuando un miembro ya liberado dejaba de avanzar
fuera de la banda segura de 0,14 m. Aumentar solamente el timeout habría
ocultado el estancamiento sin asegurar recuperación.

La primera corrección publicada mediante la PR #111 medía, por robot, la
distancia al *waypoint* activo. Un avance acumulado de 0,01 m reiniciaba el
reloj; un cambio de lote, waypoint o generación iniciaba una medición nueva.
La suite local aprobó 628/628 y el despliegue GPU
`30064823558` activó el SHA
`917b06b6c78d450e01943f54a08854eec08bc6a8`.

La primera S/N=10 de ese SHA volvió a ser rechazada a los 85,3957 s con la
misma distribución: `tb3_6`/`tb3_7` recorrieron 0,1402/0,1400 m, el error
máximo fue 3,7228 m, RTF 2,9855 y cero colisiones. El reporte tiene SHA-256
`15bb0ae034bb5d983ad76b98b58bd5495f1f807332621b4fa8d70acc11f48b96`.
La geometría final casi idéntica y la ausencia de un centro replanificado
demostraron que los avances microscópicos reiniciaban el reloj aunque no
pudieran liberar el corredor dentro del presupuesto.

El segundo candidato exige 0,10 m acumulados por ventana de 20 s, equivalentes
a 0,005 m/s. Un giro o un tramo saludable mantiene margen amplio; un equilibrio
de evasión demasiado lento dispara la recuperación. Si un miembro permanece
fuera de la banda segura sin alcanzar esa tasa:

1. no se publica otro lote positivo;
2. se envía la parada correlacionada a toda la flota;
3. se vuelve a planificar desde las poses vivas de Gazebo; y
4. se conserva el límite existente de dos replans y toda la validación
   geométrica.

No cambiaron la distancia entre robots, el despeje de obstáculos, la
tolerancia final, las velocidades ni la aceleración. La primera versión local
del detector comparaba contra el slot final y rechazó dos desvíos saludables
por waypoints. Se descartó. La versión publicada de 0,01 m también quedó
refutada por producción. El segundo candidato informa además lote activo,
número de lotes, replans y último estancamiento en el status correlacionado.

## Repetición física del segundo candidato

Antes de publicar otro commit se construyó la imagen local inmutable
`robotswarm/ros-noetic:i147-rate-local`
(`sha256:cc31061f7002…`) y se inició un `gzclient` visible de WSLg contra su
único `gzserver`. La prueba mantuvo la S/N=10 en modo móvil durante 75,026 s
de pared y terminó con `passed=true`:

- los diez robots recorrieron entre 2,1400 y 5,1409 m;
- el error final máximo medido fue 0,0943 m y el informado 0,0952 m;
- la separación mínima entre robots fue 0,4049 m;
- el despeje mínimo de obstáculo fue 0,2484 m;
- la velocidad máxima fue 0,2236 m/s y la aceleración filtrada máxima
  0,7839 m/s²;
- el RTF fue 2,9882 durante 432,348 s simulados; y
- no hubo episodios de colisión ni fallos de limpieza.

El reporte saneado local conserva permisos `0600` y SHA-256
`f9a463b785889712aa7f8561046aaa70cf47b784797b71d6e147333068731a67`.
El cliente gráfico directo mostró advertencias de recursos TurtleBot ausentes
en el host, porque esta comprobación no empleaba todavía el montaje de modelos
del publicador. Por ello la corrida demuestra dinámica, seguridad y liveness
con ventana visible, pero no se usa como gate de integridad visual ni de FPS;
ese gate se repetirá mediante el publicador empaquetado después del despliegue.

Al terminar, el arnés ya había detenido el nodo de formación y no fue posible
leer otra muestra de `/formation/status`. Este límite instrumental no invalida
las métricas físicas, pero impedía conservar el bloque de replan en el reporte.
Se añadió `routing` a la lista saneada del arnés y una regresión de contrato
antes de publicar el candidato; la repetición productiva deberá conservarlo
dentro del mismo resultado, no mediante una lectura tardía.

## Concurrencia API

El gate API de `e3dc7ad` aprobó autenticación, rosters N=3/N=7, aislamiento
401/404, dos leases HLS, rotación y revocación de tokens. Sin embargo, el
triángulo estático terminó antes de que el segundo worker persistiera
`startedAt`, por lo que los intervalos no se solaparon y el reporte completo
se rechazó. La limpieza de ambas sesiones fue completa. Su SHA-256 es
`722364808a7e2e85b20ce923417bb6e62ae8e38bbde63e7442fac0373c9189be`.

El instrumento usa ahora la letra A con tres robots y separación 0,7 m, la
misma figura no trivial empleada por el recorrido visible de dos navegadores.
No añade pausas artificiales: exige reposicionamiento físico antes de
completar y continúa verificando el solapamiento mediante las marcas
persistidas por el backend.

Sobre `917b06b` el gate final aprobó con 6,902 s de solapamiento. La letra A
terminó `Completed/Succeeded` mientras FollowLeader continuaba `Running`; los
tokens HLS cruzados devolvieron 401 y ambas sesiones terminaron `Stopped`. El
reporte `0600` tiene SHA-256
`724e5a03a85ed975c386a8fdf99fcf7c000c4d9388bcfe293aebef51852a634e`.

El recorrido posterior en dos ventanas Chrome visibles también aprobó entrada,
fullscreen, cierre/reapertura del visor B y continuidad de ROS. Los streams
midieron 30,089 y 30,069 FPS durante 10 s, con 303 frames y cero drops cada
uno. Su reporte tiene SHA-256
`14890ae59945c337b5b6a116f947fc85e561f8a76a182e222aad79fe932a8bc5`.

## Verificación local previa al despliegue

- suite ROS del primer candidato: 628/628 en 112,130 s;
- suite ROS del segundo candidato, después de añadir la telemetría al arnés:
  628/628 en 111,465 s;
- contratos de aceptación: 254/254;
- regresiones focales del controlador: 107/107; el grupo live, 5/5;
- repetición física S/N=10: aprobada en 75,026 s activos, RTF 2,9882 y cero
  colisiones;
- compilación Python y `git diff --check`: aprobados.

API y navegación visible ya quedaron aceptadas sobre `917b06b`; S/N=10 refutó
el primer umbral y mantiene abierto el cierre algorítmico. El segundo candidato
aprobó localmente y se someterá ahora a un único CI, despliegue y prueba
productiva.

## Refutación productiva del segundo candidato

La PR #112 integró el segundo candidato como
`2193c3d506e7e04eee79cc5245ae47d5db1227ce`. El gate normal, el despliegue del
backend y el despliegue GPU aprobaron en una sola ejecución cada uno
(`30066740098`, `30066968177` y `30067030369`). El worker activó la unidad
versionada `2193c3d…-30067030369-1`, reconoció la RTX 3080 y permaneció con
cero reinicios.

Dos S/N=10 frescas del SHA exacto fueron rechazadas. La primera duró 85,3905 s
de pared, alcanzó RTF 2,9908 y terminó con error máximo 2,3958 m. La segunda
duró 85,3781 s, alcanzó RTF 2,9922 y terminó con error máximo 3,6117 m. En
ambas hubo seis lotes, un replan vivo, video visible y cero episodios de
colisión; el cleanup fue completo. Los reportes `0600` tienen SHA-256
`90d5c52b853e3d1d00c5fdc80b91f7fff157651d122110b871e766056c8365f2` y
`60b3b03939b7f125a58639d8d4544300e3ec07eeb310d3783ec07eb78f56d6c5`.

Esta repetición refutó la hipótesis de que bastaba detectar una tasa baja. El
replan seguía produciendo corredores seguros, pero el secuenciador esperaba
que todo el lote activo llegase prácticamente a su destino antes de liberar el
siguiente. Un robot podía haber dejado de interceptar el corredor mucho antes
y, aun así, el resto de la flota permanecía inmóvil. El comportamiento era
seguro, pero innecesariamente serial.

## Experimento descartado y corrección de concurrencia segura

Primero se probó ordenar también las dependencias por los slots futuros y
elevar el progreso útil a 0,20 m por 20 s. Esta variante aislada volvió a
agotar el presupuesto con seis lotes y un replan. Se descartó antes de publicar
porque cambiaba el orden, pero no la condición que retenía los lotes.

La corrección final conserva el grafo de dependencias y añade una segunda
condición de liberación:

1. para cada robot ya liberado se reconstruye el corredor que todavía le queda,
   desde su pose viva, por los waypoints restantes y hasta su slot;
2. se reconstruyen del mismo modo las rutas del lote siguiente;
3. el lote siguiente se libera solamente cuando esos corredores vivos ya no
   se cruzan con el despeje geométrico del plan; y
4. todos los lotes liberados continúan recibiendo control hasta converger, en
   lugar de detener el anterior al adelantar el índice.

Si falta una pose, un slot, un waypoint finito o el plan de despeje, la
comprobación devuelve `false` y conserva el lote detenido. El umbral de
0,20 m/20 s queda como recuperación secundaria; no sustituye la prueba
geométrica. También se ordena antes al robot cuya ruta atraviesa el slot futuro
de otro, de modo que el segundo no se estacione sobre un corredor todavía
necesario.

Se añadieron regresiones para el bloqueo por slot futuro, la transición
«corredor cruzado → corredor libre» y la continuidad simultánea de los dos
lotes. La suite completa aprobó 631/631 en 105,452 s y los siete arneses
contractuales aprobaron 254/254. Sus registros locales `0600` tienen SHA-256
`51001192a1927cb4904993293b9b4432db159d4b509840b1d9015db90d80bafb` y
`3f6656298074e22854aa92ab10d4a50d4d8f4ccaa1979e8c0ebf9e9329c3fbf4`.

## Tres repeticiones visibles del tercer candidato

La imagen local inmutable `robotswarm/ros-noetic:i147-overlap-local`
(`sha256:e8788919466c…`) se ejecutó tres veces con un único `gzserver` y un
`gzclient` visible mediante WSLg. Cada corrida exigió diez robots, 75 s
continuos en estado `moving`, RTF ≥2,90, precisión ≤0,12 m, aceleración
filtrada ≤1 m/s², distancias de seguridad y cero colisiones.

| Repetición | Pared / activa | Error máximo | RTF | Separación / obstáculo | Aceleración | Resultado |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | 138,0731 / 75,0307 s | 0,0938 m | 2,9887 | 0,4063 / 0,2544 m | 0,7801 m/s² | Aprobada |
| 2 | 139,5628 / 75,0178 s | 0,0953 m | 2,9895 | 0,4025 / 0,2537 m | 0,6706 m/s² | Aprobada |
| 3 | 137,8556 / 75,0178 s | 0,0963 m | 2,9892 | 0,4031 / 0,2547 m | 0,7064 m/s² | Aprobada |

Las tres usaron dos lotes solapados, no necesitaron replan y registraron cero
episodios de colisión. Los hashes de sus resultados saneados son,
respectivamente,
`86de6c8c08754626bf4c6a26840da6abeed8d2d6aa9083215af73a01f98bd4fa`,
`893648d0853efe72b681e8efb56d79b684766235e2edd320b2d5e74349b516b7` y
`5b0964ff7f34a842daabd1eb63025622e7986d774b5ec2d3738b33f36d25f787`.
Después de la tercera repetición se cerraron contenedor, servidor y cliente;
no quedaron procesos Gazebo huérfanos.

El tercer candidato aún no se presenta como cierre productivo: primero debe
aprobar un único CI, desplegarse con el SHA exacto y repetir la aceptación
visible sobre ese mismo release.

## I-148: presupuesto de una colocación productiva más larga

La PR #113 integró el tercer candidato como
`ec4980b502dd99f7d8598387d6ff1da4a1229e61`. El PR y `main` aprobaron sus
jobs en el primer intento; el backend se desplegó mediante `30069263190` y el
único dispatch GPU `30069309217` activó la imagen inmutable
`sha256:b46895cf96ed…`. La unidad quedó activa con RTX 3080 y cero reinicios.

La primera S/N=10 productiva de ese release se rechazó a los 85,3957 s de
pared. El resultado no reprodujo el bloqueo anterior: el controlador había
liberado nueve robots, avanzó hasta el quinto de seis lotes, no informó
estancamiento ni ejecutó replans. `tb3_6` continuaba a 1,3223 m de su slot y
el último robot permanecía retenido detrás de ese corredor. La simulación
alcanzó 252,816 s, RTF 2,9605, HLS 31,2 FPS, separación mínima 0,3897 m,
despeje mínimo 0,2288 m, aceleración filtrada 0,7427 m/s² y cero colisiones.
La limpieza retiró sesión, lease, contenedor, red, publicador y workspace. El
reporte `0600` tiene SHA-256
`e821ecbe05a421d21e052f1ea96b36b01ebc8756a753f275306ef890c53f46f9`.

El timeout histórico se había dimensionado con una convergencia de 208,75 s
simulados y cubría 229,5 s al RTF conservador 2,7. La nueva colocación ya
demostró progreso sano a 252,816 s, por fuera de ese supuesto. I-148 eleva
solo la envolvente S/N=10 a 120 s de pared: a RTF 2,7 cubre 324 s simulados y
deja 71,184 s más que el último borde observado. No cambia el algoritmo, el
error máximo de 0,12 m, la ventana activa de 75 s, las velocidades, la
aceleración, el RTF mínimo ni las distancias de seguridad. Una corrida
posterior deberá demostrar que ese margen termina la tarea; el hecho de ampliar
el presupuesto no se presenta por sí solo como aprobado.

Antes de publicar I-148, la regresión focal aprobó 5/5, la suite ROS completa
631/631 en 106,956 s y los siete arneses contractuales 254/254. Los registros
locales `0600` tienen SHA-256
`9fc21ae6387095f527662716bc7781a9d011ae4a25a7a0f99d6c1f1aad73c587` y
`44863656a2db458d52f30a64d9f0cd4596fd210ac8516c72e774dc4b715e23a5`.
