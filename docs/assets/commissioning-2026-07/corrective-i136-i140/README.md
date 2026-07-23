# Evidencia previa a la entrega correctiva I-136–I-140

Este directorio conserva un resumen saneado de las comprobaciones ejecutadas
antes de publicar el PR correctivo. No contiene credenciales, UUID de sesiones,
tokens, identificadores de leases ni direcciones internas.

La imagen se construyó desde el árbol exacto, sin montar el checkout dentro de
los contenedores de aceptación. Las formaciones se observaron en ventanas reales
de Gazebo sobre WSLg. La sonda oficial comprobó el renderer D3D12 de la NVIDIA
GeForce RTX 3080, los FPS de la cámara y el factor de tiempo real durante la
ventana activa de cada algoritmo.

El primer intento gráfico se conserva únicamente como diagnóstico: abrió la
ventana, pero utilizó `llvmpipe`, rindió 6,03 FPS y fue rechazado. La causa fue
que el contenedor manual no tenía disponibles `/dev/dxg` y las bibliotecas D3D12
de WSL. Se creó un contenedor fresco con esos recursos y se repitieron los casos;
solo esas repeticiones NVIDIA forman parte del resultado aceptado.

En N=3 el error máximo medido sobre el estado real fue 0,0921 m y el reporte del
comportamiento indicó 0,0925 m. En N=10 fueron 0,0974 m y 0,0981 m,
respectivamente. Esta separación evita confundir la medición independiente con
la telemetría producida por el propio controlador.

La prueba productiva N=1 utilizó el arnés local con I-139 contra el despliegue
`1448a31`. El caso terminó antes de necesitar renovar el lease: por tanto,
demuestra la conservación temprana del reporte atestado y la limpieza final,
pero no se presenta como observación física de una renovación. Las ramas de
renovación, fallo de HLS y binding desconocido permanecen cubiertas por las
pruebas contractuales.

Los reportes completos siguen bajo custodia local fuera del repositorio. Sus
hashes se incluyen en `predeploy-summary.json` para poder detectar cualquier
cambio sin publicar identificadores operativos.
