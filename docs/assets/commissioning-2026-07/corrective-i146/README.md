# Evidencia saneada de I-146

I-146 se abrió el 23 de julio de 2026 durante la aceptación visible del corte
productivo `2445a3735ca42bd574c1cfbb1fcb2bbc7b2dce18`. El despliegue GPU
`30055847809` había aprobado y dejó activo el release
`2445a3735ca42bd574c1cfbb1fcb2bbc7b2dce18-30055847809-1`.

## Síntoma observado

Una matriz de seis formaciones aprobó triángulo N=3, pero las cinco filas
siguientes agotaron la espera del visor antes de iniciar ROS. MediaMTX sí
registró publicadores H.264 y, en esos casos, creó el muxer HLS. Un cuadrado
fresco posterior tampoco alcanzó un fotograma decodificado. Ambos ensayos se
rechazaron y retiraron por completo sus sesiones, redes, contenedores, leases y
publicadores.

El frontend concedía 30 s a la aparición de la lista HLS. La preparación real
incluye la pantalla X privada, `gzclient`, el renderer D3D12, FFmpeg, RTSP y el
primer acceso HLS. En producción esta secuencia tardó aproximadamente 28 s en
los casos normales. Por tanto, una variación pequeña de red o arranque podía
consumir el presupuesto aunque el worker y MediaMTX estuvieran sanos.

## Corrección candidata

La ventana inicial y de recuperación HLS pasa de 30 a 60 s. Es un presupuesto
acotado y no modifica el TTL del lease, la propiedad de la sesión ni el
cleanup. El reproductor continúa reintentando solo dentro de esa ventana y
conserva los mensajes distintos para 404, 429, 502/504 y otros fallos de red.

El arnés conserva además el último estado saneado de la interfaz cuando el
visor no arranca. Si ROS termina antes del primer muestreo visual, permite que
el proceso hijo concluya su teardown dentro del timeout ya autorizado, con un
máximo de 60 s, para no perder `RESULT_JSON` y `SUMMARY_JSON`. Esta espera no
convierte una tarea terminal en aprobación: únicamente preserva el diagnóstico.

## Repetición visible aceptada

Antes de aplicar el cambio de frontend, una repetición limpia de cuadrado N=5
demostró que la ruta completa podía funcionar:

| Medida | Resultado |
| --- | ---: |
| FPS promedio de Gazebo | 58,203 |
| FPS posterior de Gazebo | 62,504 |
| FPS decodificados en Chrome | 29,960 |
| FPS presentados en Chrome | 30,157 |
| RTF físico | 2,9959 |
| Error final máximo | 0,0888 m |
| Separación mínima no esperada | 0,4277 m |
| Despeje mínimo de obstáculo | 0,3714 m |
| Colisiones | 0 |
| Ventana activa de formación | 75,0451 s |
| Limpieza | Completa |

El reporte saneado tiene SHA-256
`f739be7a5fea7e38d8ec7930d8f23d6db1f40511385d6a5d73fe614ca65ce60d`.
La captura del navegador tiene SHA-256
`c2b26c6e574f895a92513fea5187e9d4b0d34ab9302415120d724151b619764e`.
Este resultado acepta solo cuadrado N=5 sobre `2445a37`; no se presenta como
matriz global.

## Por qué se observaron dos ventanas de Gazebo

El visor privado mantiene un `gzclient` principal ligado al lease del usuario.
Durante la aceptación, la sonda oficial abre un segundo `gzclient` temporal en
la misma pantalla privada y contra los mismos masters, con el fin de medir
NVIDIA, FPS y RTF mientras ROS sigue activo. La segunda ventana no contiene
otro `gzserver` ni otra física. La sonda elimina su grupo de procesos al
terminar.

Después de cerrar las superficies visibles se comprobaron cero `gzclient`,
cero `gzserver`, cero Xvfb del visor y cero contenedores ROS residuales. En uso
normal, fuera del arnés, existe un cliente gráfico por lease; dos usuarios o
dos sesiones concurrentes mantienen clientes distintos de forma intencional.

## Verificación local del cambio

- frontend React: 164/164 pruebas, 30/30 suites;
- prueba focal de HLS: 37/37;
- build optimizado de frontend: aprobado;
- contratos del arnés de matriz: 74/74;
- `git diff --check`: aprobado.

El cierre de I-146 requiere todavía publicar el candidato, repetir el arranque
HLS contra el bundle de Cloudflare resultante y completar las filas ROS
pendientes. Hasta entonces, la ampliación a 60 s se clasifica como corrección
local verificada y no como aceptación postdeploy.
