# Datos crudos de la aceptación N=10

Este directorio conserva una selección de los datos que respaldan las Figuras 5–7, 9 y 10 del informe de comisionamiento. Se evitó incluir archivos XWD intermedios y el registro continuo de 18 MB porque no añaden información a los resultados estructurados.

- `before-run25-raw.log`: registro del ensayo anterior que quedó detenido en `APPROACH`.
- `gui-preflight-gpu.json`: FPS, renderizado y factor de tiempo real antes de la carga.
- `acceptance.result.json`: mediciones completas del transporte colaborativo aceptado.
- `search-probe-feasible.result.json`: mediciones completas de búsqueda, aviso y respuesta.
- `*.summary.json`: resúmenes reducidos para consulta rápida.
- `final-isolation-state.txt`: comprobación de limpieza del contenedor y de los procesos.
- `manifest.txt`: inventario histórico del conjunto capturado antes de seleccionar la evidencia que se conservaría en el repositorio.
- `checksums.sha256`: hashes de todos los archivos conservados y de las cinco figuras derivadas. Se comprueban, desde este directorio, con `sha256sum -c checksums.sha256`.

La revisión base ensayada se encuentra en `source-sha.txt`. Estas mediciones son evidencia visible previa al release; la aceptación del SHA desplegado se documenta por separado en el informe final. Los registros pueden contener identificadores efímeros de tareas o contenedores, pero no incluyen contraseñas, tokens ni credenciales del sistema.
