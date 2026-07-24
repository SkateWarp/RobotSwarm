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
