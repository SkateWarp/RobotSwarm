# Evidencia saneada de I-143

Esta incidencia se encontró el 23 de julio de 2026 durante la reconstrucción
del candidato local exacto, antes de publicarlo. La prueba empleó Gazebo
Classic visible mediante WSLg, render D3D12 sobre una NVIDIA GeForce RTX 3080,
RTF objetivo 3,0 y la interfaz pública `/swarm/commands`.

## Antes: bloqueo del secuenciador

La primera corrida S/N=10 no llegó al estado `moving`. Seis robots convergieron
y cuatro permanecieron esperando. El robot que retenía el primer lote terminó
a 0,1246 m de su slot: estaba dentro de la banda segura de histéresis de
0,14 m, pero fuera de la tolerancia estricta de llegada de 0,09 m. El
secuenciador consultaba solamente esta última condición y, por ello, no liberó
los corredores posteriores. La corrida se rechazó correctamente a los
85,39 s; no hubo colisiones y RTF fue 2,986.

La investigación descartó falta de memoria: WSL informaba cerca de 12 GiB
disponibles y el contenedor consumía aproximadamente 1 GiB. También se
comprobaron `/clock`, `/gazebo/get_world_properties`, las conexiones
`/{robot}/cmd_vel` y la correlación del `task_id`. El fallo era lógico y
reproducible en el cambio entre lotes, no una desconexión del visor.

## Corrección

Se separaron dos conceptos que antes compartían el mismo indicador:

- un robot sigue necesitando la tolerancia estricta para declarar que alcanzó
  su slot y para aprobar la formación;
- un corredor puede quedar libre cuando el robot ya está dentro de la banda
  segura de retención.

La validación geométrica viva, la exclusión de obstáculos, el límite Burger y
el gate final de error máximo de 0,12 m no se redujeron. Se agregó una regresión
unitaria que coloca al primer robot entre 0,09 y 0,14 m, comprueba que no se
declare convergido y exige que el segundo lote sí reciba movimiento.

## Después: candidato exacto

La imagen inmutable local
`sha256:4caa2ea97dc55e3f0e4929568e255fcbea0d6969318522e34f889a6df72691e9`
aprobó S/N=10:

- ventana activa visible: 15,0061 s;
- error final máximo: 0,0936 m;
- distancia mínima entre robots: 0,4013 m;
- despeje mínimo de obstáculos: 0,2498 m;
- velocidad máxima: 0,2228 m/s;
- aceleración máxima filtrada: 0,8874 m/s²;
- RTF: 2,9912; y
- colisiones: 0.

El log completo permanece fuera del repositorio, con permisos `0600`, en
`/tmp/robotswarm-acceptance/exact-final/formation-S-n10-release-band.log`.
Su SHA-256 es
`a87ed980d1e2370c74b971051774b27209898967494678beae2e688329168f2d`.
No se versiona el archivo crudo
porque contiene telemetría extensa. Las capturas visuales anteriores y
posteriores del comisionamiento continúan bajo `final-1448a31/`; una imagen
aislada no sustituye las métricas temporales correlacionadas de esta prueba.
