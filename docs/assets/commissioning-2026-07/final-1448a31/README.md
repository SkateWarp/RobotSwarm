# Evidencia visual del marcador de destino

Este directorio conserva tres capturas sanitizadas relacionadas con el
marcador fantasma del transporte colaborativo. No contiene credenciales,
identificadores de sesión, tokens ni direcciones internas.

| Archivo | Procedencia | Uso válido |
| --- | --- | --- |
| `marcador-antes-9f49e17.png` | Matriz visible anterior al cambio, escenario `transport_grf_n10` | Comparación «antes»: el destino todavía no se representaba en Gazebo |
| `marcador-despues-1448a31.png` | Matriz visible posterior al despliegue `1448a31`, escenario `transport_grf_n10` | Comparación «después»: el disco magenta identifica el destino y el visor mantiene diez robots operativos |
| `busqueda-n4-local.png` | Ensayo local visible N=4 sobre la imagen candidata previa al despliegue | Evidencia complementaria de búsqueda distribuida y encuadre completo; no es evidencia productiva |

La captura productiva «después» demuestra la representación visual, pero no
se utiliza por sí sola para aceptar el resultado físico del algoritmo. Las
métricas de desplazamiento, participación, colisiones, FPS y factor de tiempo
real proceden del protocolo correlacionado del mismo ensayo.

## Integridad

```text
d036d9814b893441f2c57be52a4ab58415339f85bfeba6055177820417211f11  busqueda-n4-local.png
4f27a3112d536da9a734a0b4d4d1a7b10117edded00c7451a723e1ff96018ed9  marcador-antes-9f49e17.png
916560190eb08d81e1608c61adbed756b6df299391d5b320b35a656343ea6a9b  marcador-despues-1448a31.png
```
