# Evidencia saneada de I-144

I-144 se abrió el 23 de julio de 2026 durante la aceptación posterior al
despliegue `1d497d4ac33c41416b48300475434a0d11e92d50`. El gate de `main`, el
backend y el worker GPU habían desplegado correctamente el mismo SHA.

## Resultado postdeploy rechazado

La matriz visible de seis formaciones aprobó triángulo N=3, letra A/N=7 y
S/N=10. Cuadrado N=5, V/N=8 y diamante N=9 se detuvieron exactamente en sus
presupuestos históricos de 40, 55 y 50 s. Los tres conservaban tarea
`forming`, RTF entre 2,9922 y 2,9962, cero colisiones y limpieza completa. Por
ello el resultado global 3/6 se rechazó; no se presentó como cierre parcial.

El reporte completo, modo `0600`, está en
`/tmp/robotswarm-acceptance/postdeploy-1d497d4/formations.json`, SHA-256
`4c35bcee50ab5ee6e3ed958194b326bdb186a495d3383fabdf12ef6c07254c08`.

## Diagnóstico

Las filas aprobadas necesitaron entre 87,058 y 140,105 s totales porque,
después del ensamblaje, el arnés añade una ventana activa de 75 s. En cambio,
las tres filas rechazadas terminaron antes de iniciar esa ventana. Las rutas
seguían avanzando, no existían contactos y el reloj mantenía RTF cercano a 3.
Los límites representaban 108, 148,5 y 135 s simulados al RTF mínimo 2,7,
insuficientes para algunas ubicaciones válidas del spawn.

Una repetición local V/N=8 encontró además tres replans falsos. La posición
cambió solo 2–4 mm, pero el asentamiento físico giró robots entre 0,104 y
0,133 rad mientras se calculaban las rutas. Ningún comando positivo había sido
liberado. El Burger se valida como un disco y esa rotación no cambia su
geometría de colisión.

## Corrección y resultados locales

Los presupuestos de cuadrado, V y diamante pasan a 90 s: 243 s simulados al
RTF mínimo. No cambian el error máximo de 0,12 m, las distancias de seguridad,
el límite de aceleración ni la física. La tolerancia predeterminada de yaw
durante el commit pasa de 0,10 a 0,15 rad; conserva el tope de 0,20 rad, la
tolerancia posicional de 0,08 m, dos replans como máximo y toda la revalidación
geométrica.

La imagen local exacta
`sha256:56b82f57903fcbbad6ce68ea1d8210bd0b13faf03366e8ba544e6b1b7b66be9f`
aprobó:

- cuadrado N=5: 62,6399 s, error 0,0945 m, separación 0,4222 m y cero
  colisiones;
- V/N=8: 78,4313 s, error 0,0937 m, separación 0,3613 m y cero colisiones; y
- diamante N=9: 86,3403 s, error 0,0960 m, separación 0,3878 m y cero
  colisiones.

Todos sostuvieron 15 s en `moving` y RTF ≥2,90. Los logs `0600` tienen,
respectivamente, SHA-256
`eb40a23d0de0ee23ebbf50304876426ea40e570801af78bc77f9502f5f5d884c`,
`3076ff27e094bbf2491ccfdc7e5cb0e57aaa851aadf5615cc0a621294bc81017`
y `04b188d8d2e1287f0c67787effd85152f0586113a81b705fd923334a5aab3267`.

Una repetición intermedia de V no alcanzó el roster porque `gzserver` terminó
con `Segmentation fault` al crear el cuarto modelo. Quedaban cerca de 12 GiB
disponibles y el servidor Gazebo desapareció mientras `gzclient` seguía vivo.
Se clasificó como fallo ambiental de Gazebo Classic/WSLg, se retiró el
contenedor y la repetición fresca aprobó. No se usó como evidencia del
algoritmo ni se ocultó del registro.

## Freeze local

La suite ROS completa aprobó 626/626 pruebas en 111,472 s. Los contratos de
aceptación aprobaron 253/253 en 5,581 s al ejecutarse solos. Una ejecución
paralela previa agotó el límite de 5 s en dos contratos de supervisión; al
terminar no existían fixtures huérfanos y la repetición aislada aprobó sin
cambios. Se registró como interferencia temporal del arnés, no como defecto
del producto.
