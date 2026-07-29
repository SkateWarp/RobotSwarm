# Evidencia de recuperación y repetición del 28 de julio

Este directorio conserva una selección saneada de la aceptación ejecutada
después de recuperar el backend. Los reportes originales usan los nombres
temporales generados por cada arnés; las imágenes seleccionadas se copiaron sin
recomprimir y recibieron nombres académicos estables. Por eso el campo `file`
del JSON no siempre coincide con el nombre preservado.

No se incluyeron correos, contraseñas, JWT, UUID de sesión, direcciones privadas
ni nombres del worker. La selección evita versionar capturas redundantes sin
presentarlas como inexistentes.

## Recorrido administrativo

| Nombre del reporte o función | Archivo preservado | SHA-256 |
| --- | --- | --- |
| `templates.png` | `admin-plantillas.png` | `63edd4d9fef0e780ff8d3dd022eeb5f93971eb62bc5baf343d826e1bc91a9566` |
| `robots-dialog.png` | `admin-robots-dialogo.png` | `fb83bf359a6ec5a7b4f19edc964ff4db8cfb7ef442a038f9e01bbc1b88ea6e1a` |
| `groups.png` | `admin-grupos.png` | `16fa42e470d03c31cba0d2e8112425ff5c1de55b8604e79ad5ada40211bb3c01` |
| `users-dialog.png` | `admin-usuarios-dialogo.png` | `743354412c4510d048842e7e7891f8ef2fabe0e70bbe2476cfd43d7f09ebe81e` |

El arnés también produjo `groups-dialog`, `groups-temporary`, `history`,
`history-dialog`, `robots`, `templates-dialog` y `users`. Esas siete capturas
se revisaron durante la corrida, pero se omitieron de la selección por ser
redundantes. Sus hashes y resultados permanecen en
`secciones-admin-reporte.json`.

Dos recorridos instrumentales anteriores se conservan como rechazados:

- `secciones-admin-intento-cdp-rechazado.json` demuestra que el arnés se negó a
  adjuntarse cuando Chrome normalizó el primer marcador de propiedad;
- `secciones-admin-intento-privacidad-rechazado.json` demuestra que una
  coincidencia ambigua con la contraseña corta bloqueó la captura y que el
  grupo temporal se eliminó de todos modos.

El tercer recorrido sustituyó el marcador por una página `data:` aleatoria y
ocultó las coincidencias cortas antes de volver a comprobar la imagen. Es el
único que se presenta como aceptado.

## Transporte iniciado desde React

| Nombre original | Archivo preservado | SHA-256 |
| --- | --- | --- |
| `transport-search-gazebo.png` | `transporte-busqueda-gazebo.png` | `567c973a6f7c1dd6fcda1a786bc61e062941e67df37fd2f9318f742479e4f70c` |
| `transport-push-gazebo.png` | `transporte-empuje-gazebo.png` | `9d8abdf69291f6edb5c6ebf1b5112eaeff012715be82e089e7315e7938f08b71` |
| `transport-done-panel.png` | `transporte-final-panel.png` | `69c0794ed3edb1a81f64fa43a9e44a204ce1d70c533d6ac9c48e124d85a2a213` |

`transport-search-panel.png` y `transport-push-panel.png` se omitieron porque
el panel final y las dos escenas de Gazebo cubren los mismos hitos. El reporte
`transporte-ui-reporte.json` conserva sus hashes y confirma que las cinco
capturas originales se obtuvieron de la misma corrida.

## Matriz visible y repeticiones aisladas

La primera matriz posterior a la recuperación ejecutó catorce escenarios con
Chrome normal y Gazebo visible. Ocho casos aprobaron integralmente y seis
quedaron rechazados; el reporte conserva ambos grupos sin convertir los fallos
en resultados favorables.

| Clasificación | Archivo preservado | SHA-256 |
| --- | --- | --- |
| matriz inicial, resultado mixto | `matriz-ros-inicial.json` | `f05d60ea47b882c3f3b2a256ebaa2971b8f95202953d1705e27773a9241d77ef` |
| triángulo N=3, repetición aprobada | `formacion-triangulo-n3-repeticion.json` | `039e751ea6053799338010b8871731cee8f8e15e022fdbc3e7e6c5fa5c133be4` |
| letra A N=7, repetición aprobada | `formacion-a-n7-repeticion.json` | `3ee87f6d03583242876e689096d8fe92944a04709068c8037510592126ed1617` |
| letra S N=10, interrupción voluntaria | `formacion-s-n10-interrumpida.json` | `6ac24dc699f0810f6f1cc90b2ef46be381243ae5ee7d88952ddc75e8e8206f0b` |
| letra S N=10, visor oculto rechazado | `formacion-s-n10-visor-oculto-rechazada.json` | `9b37ad922c612b38d30af6ff54e88f8000dd51740db46423254120b72bdfd6bd` |
| letra S N=10, asentamiento rechazado | `formacion-s-n10-asentamiento-rechazada.json` | `a8352f104e6de76b9bf309415f517eae3b5a9baf277826659ee1eeb73cd6e656` |
| render del intento de asentamiento | `formacion-s-n10-asentamiento-render.json` | `c5d58f12ddec035825b4ec9e47f28253a1456ed37bf11773a727e2bb838b766b` |

La interrupción S/N=10 ocurrió al detectar una compilación ESP-IDF ajena que
habría contaminado RTF y FPS. Sólo se interrumpió el arnés propio; la limpieza
fue completa. El segundo rechazo S/N=10 ocurrió antes de iniciar ROS: Chrome
tenía el documento oculto, no solicitó HLS y no decodificó video. Este último
reporte documenta el síntoma que motivó la corrección del instrumento; no es
evidencia de un fallo del algoritmo.

El tercer intento S/N=10 confirmó que el visor ya era válido: HLS rondó
31 FPS y Gazebo visible promedió 49,332 FPS mediante D3D12/RTX 3080. ROS
conservó RTF 2,9687 y cero colisiones, pero agotó dos replans porque siete
robots se asentaron aproximadamente 0,177 m mientras el solver usaba la
instantánea anterior. Por eso se preserva expresamente como rechazo del
algoritmo previo a la barrera de estabilidad; el reporte de render adjunto
demuestra que no fue otro falso negativo gráfico. El log crudo del contenedor
ya había sido retirado por la limpieza normal, así que no se inventa una
atribución más precisa que la disponible en el protocolo conservado.

Los JSON se revisaron antes de copiarlos. No contienen correos, contraseñas,
tokens, UUID de sesión, direcciones privadas ni nombres del worker. Los
resultados físicos definitivos se añadirán con nombres diferentes después del
despliegue para conservar la separación entre el artefacto anterior y el
candidato final.

`SHA256SUMS` se genera al cerrar la selección final, después de incorporar los
reportes de matriz y multiusuario aceptados.
