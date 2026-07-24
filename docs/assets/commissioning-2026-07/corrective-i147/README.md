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

La corrección mide, por robot, la distancia al *waypoint* activo. Un avance
acumulado de al menos 0,01 m reinicia el reloj; un cambio de lote, waypoint o
generación inicia una medición nueva. Si un miembro permanece 20 s simulados
sin progreso útil y fuera de la banda segura:

1. no se publica otro lote positivo;
2. se envía la parada correlacionada a toda la flota;
3. se vuelve a planificar desde las poses vivas de Gazebo; y
4. se conserva el límite existente de dos replans y toda la validación
   geométrica.

No cambiaron la distancia entre robots, el despeje de obstáculos, la
tolerancia final, las velocidades ni la aceleración. La primera versión local
del detector comparaba contra el slot final y rechazó dos desvíos saludables
por waypoints. Se descartó. La versión corregida mide el tramo activo y
acumula pasos pequeños antes de decidir que existe progreso.

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

## Verificación local previa al despliegue

- suite ROS completa final: 628/628 en 112,130 s;
- contratos de aceptación: 254/254;
- regresiones focales finales de liveness y matriz móvil: 3/3;
- compilación Python y `git diff --check`: aprobados.

El SHA integrado se someterá a S/N=10, API concurrente y navegación visible.
Esta evidencia previa no se presenta como cierre productivo.
