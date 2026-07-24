# Evidencia saneada de I-145

I-145 se abrió el 23 de julio de 2026 al desplegar el worker GPU del corte
`ea25434c17a45c9032857a5cc86c40ea42b46c21`. El gate de `main` y el
despliegue backend habían aprobado; `robot.zerav.la/health` y `rs.zerav.la`
respondían HTTP 200.

## Síntoma y contención

Los runs `30054706834` y `30054818947` terminaron en el paso **Acquire
scheduling drain lease**. En ambos casos `curl` agotó los 5 s durante la
resolución DNS de `robot.zerav.la` y devolvió código 28. El fallo ocurrió antes
de adquirir el lease, hacer checkout, construir la imagen o cambiar el enlace
de release. Por ello no existió despliegue parcial: el servicio siguió activo
en el release
`1d497d4ac33c41416b48300475434a0d11e92d50-30051789195-1`.

El primer run fallido se repitió una sola vez después de comprobar cinco
resoluciones correctas, entre 0,064 y 0,130 s, todas con HTTP 200. La misma
falla en el segundo run descartó un evento aislado y se suspendieron nuevos
dispatches.

## Diagnóstico

El endpoint configurado era `https://robot.zerav.la`. La terminal WSL resolvió
la dirección `148.101.81.9`; el mismo binario `/usr/bin/curl`, ejecutado tres
veces con el entorno del proceso `Runner.Listener`, resolvió en
0,106–0,112 s y recibió HTTP 200. El runner no tenía variables de proxy. La
evidencia es compatible con una intermitencia breve de DNS durante el inicio
del job, no con una caída del backend ni con un error del algoritmo ROS.

El workflow reintentaba las consultas posteriores de estado de drenaje, pero
la adquisición inicial tenía un único intento. Esa asimetría convertía un
fallo transitorio de red en un despliegue fallido aun cuando todavía no se
había mutado producción.

## Corrección

La adquisición conserva `connect-timeout=5`, `max-time=20`, autenticación,
revisión objetivo y validación HTTP 200. Se permiten como máximo tres intentos
cuando `curl` devuelve un error de transporte, con 5 s entre intentos. Una
respuesta HTTP se procesa de inmediato: los estados distintos de 200 no se
reintentan ni se disimulan. Si los tres transportes fallan, la credencial se
retira del entorno y el job termina cerrado.

El CI contiene un contrato textual para impedir que se elimine
accidentalmente este reintento. No se modifica backend, frontend, ROS ni el
lease de seguridad.
