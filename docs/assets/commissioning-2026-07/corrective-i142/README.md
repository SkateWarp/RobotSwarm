# Evidencia saneada de I-142

Esta evidencia corresponde a los recorridos públicos del 23 de julio de 2026
contra el release desplegado
`1448a31bbbbfd77588bada109947098cc95d9dda`. Los JSON completos se conservan
fuera del repositorio con modo `0600`; no se versionan perfiles, credenciales,
tokens, leases ni identificadores de sesión.

## Intentos rechazados

El primer gate API comprobó sesiones N=3/N=7, rosters, playlists, revocación e
isolamiento, pero se rechazó porque el triángulo estático con separación 0,7 m
podía estar ya satisfecho por el spawn y terminar antes de que FollowLeader
registrara su inicio. Se cambió solamente el parámetro del arnés a 1,3 m para
exigir reposicionamiento observable. No se alteró el producto.

El primer recorrido Chrome llegó a interacción, fullscreen, tareas concurrentes
y cierre/reapertura del visor B. Se rechazó al sumar como tiempo presentado
negativo una corrección hacia atrás del borde vivo HLS. El arnés conserva ahora
el retroceso como diagnóstico separado y mantiene los gates independientes de
frames decodificados, callbacks, FPS y drops.

El segundo recorrido volvió a probar interacción y fullscreen, pero muestreó la
toolbar en la transición breve entre la salida de fullscreen de Chrome y el
repintado de React. La espera final se acotó a 5 s y exige de nuevo `En vivo` y
el botón `Pantalla completa`; no acepta un estado incompleto.

## Resultado final

- el gate API multiusuario aprobó y su JSON tiene SHA-256
  `f913d7681b7318170a7d70f49d4837dda0334136f030dc2139730fe4f8a7e3fa`;
- responsive aprobó 360, 768, 1366 y 1920 píxeles; su JSON tiene SHA-256
  `9c542dbde5ae06e4e3f3b2f886593f0aa430dbfbd841e688387294235e461e96`;
- el recorrido final de dos Chrome visibles aprobó con SHA-256
  `95c6a6563288b8621778950ada3f8dd969cc609fcee1d212cadffd56e3c0a1be`;
- ambos visores aceptaron clic, arrastre, rueda, teclado y fullscreen;
- el clic de inicio tuvo un skew de 0,198 ms y ambos paneles se observaron
  `Running` en la misma ronda, con 1,759 ms entre muestras;
- A completó la letra A mientras B siguió ejecutando FollowLeader;
- B cerró y reabrió solamente su visor sin detener ROS;
- ambos videos midieron aproximadamente 30 FPS, cero drops y más de 10 s de
  avance; y
- después de detener A, B continuó `Running` y decodificando durante otros
  10 s. La limpieza retiró sesiones propias, controles, fullscreen, perfiles,
  procesos Chrome y puertos CDP.

Los contratos locales posteriores aprobaron 17/17 para el gate API y 41/41
para el recorrido visible.
