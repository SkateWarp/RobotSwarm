# Evidencia saneada de I-141

Esta carpeta describe la reproducción local visible del 23 de julio de 2026.
No contiene credenciales, identificadores de sesión, tokens, perfiles de
navegador ni el log JSON completo. Los archivos crudos permanecen fuera del
repositorio con modo `0600`.

## Secuencia antes/después

1. La primera ejecución `formation_S_n10` sobre el delta local anterior llegó
   a iniciar el callback de formación, pero el orquestador la marcó `FAILED`
   por heartbeat obsoleto mientras el solver seguro todavía calculaba rutas.
   No hubo asignaciones ni colisiones. El log local tiene SHA-256
   `6a257d7e8474cb73ad2716c124470de451229111ae06560bf9065e24cdbf28bb`.
2. Después de publicar el estado correlacionado `forming` y añadir una gracia
   acotada solo para planificación estacionaria, la tarea ya no fue abortada.
   Sin embargo, dos asentamientos de Gazebo —0,057456 m/0,067711 rad y
   0,019426 m/0,050031 rad— agotaron dos replans seguros y dejaron cuatro
   robots esperando cuando terminó el plazo. El resultado se rechazó. Su log
   tiene SHA-256
   `5c7e64dac256810e58337fcf0221ee3b4fbb92f42504e03fd5ffa42cd63abb29`.
3. La corrección final conserva la revalidación geométrica completa y admite
   únicamente el asentamiento normal de un Burger recién creado: 0,08 m y
   0,10 rad, con límites configurables máximos de 0,15 m y 0,20 rad. Un cambio
   grande como el histórico de 1,830611 m continúa siendo rechazado.

## Resultado aprobatorio

La imagen local inmutable
`sha256:3394046cede94cd3855cf8b834546176f48453369b93b0879ad13f75d39b5f48`
se ejecutó sin montajes de fuente, con `gzclient` visible, WSLg, `/dev/dxg`,
D3D12 y una NVIDIA GeForce RTX 3080. El caso `formation_S_n10` aprobó:

- 10/10 robots asignados y 75,0004 s continuos en estado `moving`;
- error final máximo independiente de 0,0952 m;
- aceleración lineal filtrada máxima de 0,7039 m/s²;
- distancia mínima entre robots de 0,4025 m;
- distancia mínima a obstáculo estático de 0,2491 m;
- cero episodios de colisión y cero contactos inesperados; y
- RTF 2,9851.

El log saneado local tiene SHA-256
`b491a7e5978ec6c2cb22982cf8cbd7428be33aa7b3aa6a34e27475505e9a3e25`.
La sonda gráfica previa de la misma sesión de trabajo informó 59,523 FPS de
cámara, 62,501 eventos de posrenderizado por segundo y RTF 2,996; su reporte
local tiene SHA-256
`37836cd13c7ea6f306fcf4d83083beb95565d2f3fb1a1a780f4159169edb3680`.

La suite ROS completa posterior aprobó 624/624 pruebas en 118,539 s. Este
resultado acredita el árbol local; todavía debe repetirse después de integrar y
desplegar el SHA correctivo exacto.
