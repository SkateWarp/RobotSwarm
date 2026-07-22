# Aceptación del transporte colaborativo con carga

## 1. Objetivo y alcance

Esta prueba comprueba que cuatro TurtleBot3 Burger pueden encontrar, rodear y
transportar juntos una carga que no debería ser desplazada de forma útil por un
solo robot. No basta con observar movimiento en Gazebo: la aceptación relaciona
la física de la carga, los estados del algoritmo GRF, el visor HLS abierto en un
navegador normal y la limpieza posterior de la sesión.

La caja verde del escenario habitual tiene una masa de 0,25 kg. Sirve para las
pruebas rápidas de búsqueda, aviso, encuentro y control, pero un Burger puede
moverla por sí solo. Para estudiar reparto físico de carga se utiliza el perfil
`transport_crate_loaded`, de 0,75 kg y con un coeficiente de fricción de contacto
de 0,25.

La selección de esos valores parte de una estimación sencilla:

- fuerza de inicio de movimiento de la caja: `0,75 × 0,25 × g = 1,84 N`;
- tracción estimada de un Burger de 1 kg: `1,00 × 0,10 × g = 0,98 N`;
- tracción estimada de cuatro Burger: `4 × 0,98 = 3,92 N`.

Este cálculo solo sirve para escoger un caso de prueba razonable. El criterio de
aceptación procede de las mediciones del simulador y no de la estimación.

## 2. Ejecución supervisada

El gate productivo se inicia desde el equipo con GPU cuando no existe otra
sesión de aceptación. Se emplea Chrome visible, sin `--headless` y sin
`--disable-gpu`, y se crea una sesión nueva de cuatro robots mediante la misma
interfaz pública que utiliza una persona.

```bash
python3 scripts/acceptance/robotswarm-loaded-n4-e2e.py \
  --execute-production \
  --deployment-commit <sha-productivo-completo> \
  --credentials <archivo-privado-0600> \
  --chrome <chrome-visible> \
  --profile-root <directorio-temporal> \
  --output <informe-saneado.json>
```

El archivo de credenciales no forma parte de la evidencia. El informe final se
escribe con permisos `0600`, oculta identificadores privados y conserva hashes
SHA-256 de los documentos que sí pueden auditarse.

La secuencia supervisada es la siguiente:

1. Se comprueba que Chrome dispone de Media Source Extensions (MSE),
   `SourceBuffer` y soporte H.264 compatible con `hls.js` antes de solicitar un
   lease. Así, un navegador incapaz de reproducir el visor no consume una sesión
   productiva para terminar después en un falso diagnóstico de Gazebo.
2. Se crea la sesión N=4, se comprueba su roster y se vincula un único contenedor
   administrado al SHA esperado.
3. Se abre el HLS privado y se exige al menos un fotograma decodificado. El
   `gzclient` primario debe pertenecer al publicador de ese lease y usar los
   mismos `ROS_MASTER_URI`, `GAZEBO_MASTER_URI`, red y display privado que la
   sesión.
4. La sonda oficial instala la caja de 0,75 kg y ejecuta ensayos de capacidad con
   uno, dos y cuatro robots. A continuación mantiene esa carga para el GRF N=4.
5. Mientras el GRF cargado sigue activo, se abre un segundo `gzclient` temporal
   en el mismo display y se ejecuta el preflight oficial de GPU, FPS y RTF. El
   navegador mide HLS durante el mismo intervalo.
6. Durante `PUSH`, una captura debe quedar encerrada entre dos estados nuevos de
   la misma tarea, con masa fresca de 0,75 kg, roster completo y HLS por encima
   del mínimo. Solo después se validan los resultados y la limpieza.

La opción interna `--external-viewer-verified` no certifica una ventana por sí
misma. Solo se entrega a la sonda ROS después de que el supervisor haya
correlacionado el proceso real, el lease, el display y los dos masters. Ejecutar
el probe aislado con esa opción no constituye evidencia válida.

## 3. Criterios físicos y algorítmicos

En los ensayos de capacidad se envía la misma orden lineal de 0,16 m/s durante
12 segundos simulados. La prueba aprueba únicamente cuando se cumplen todos los
siguientes puntos:

- un robot desplaza la caja como máximo 0,05 m;
- las dos raíces en contacto directo la desplazan como máximo 0,06 m;
- cuatro robots la desplazan al menos 0,20 m;
- el avance con la flota es al menos cuatro veces el avance individual;
- cada robot avanza como mínimo 0,05 m;
- las dos raíces conservan contacto medido con la carga y los otros dos robots
  conservan contacto con su predecesor; y
- los tres ensayos mantienen RTF no menor que 2,90.

Las duraciones `push_duration_*` abarcan solo el comando positivo. Los ocho
mensajes de velocidad cero de la parada segura se publican después de capturar
los extremos. Por este motivo se admite una tolerancia de 0,25 s simulados, pero
no se suma la parada al intervalo que se utiliza para recalcular el RTF.

El segundo tramo ejecuta el algoritmo normal de transporte con la carga todavía
instalada. Se exige la secuencia no regresiva `SEARCH → APPROACH → PUSH → DONE`,
un único robot descubridor, aviso a sus tres compañeros, búsqueda inicial por
los cuatro, encuentro en las coordenadas comunicadas y contribución útil de toda
la flota. La prioridad es ocupar primero los dos contactos directos con la caja;
los demás robots empujan a través de cadenas contiguas de compañeros. El avance
físico mínimo hacia el objetivo se mantiene en 0,50 m.

El umbral que publica el runner es dinámico: parte de la distancia restante al
entrar en `PUSH`, resta la tolerancia de llegada y se limita por la configuración.
Puede aparecer como 0,4999 m por redondeo. El arnés valida ese eco con un epsilon
de 0,001 m, pero en una comprobación separada continúa exigiendo 0,50 m de avance
físico real.

## 4. Correcciones introducidas durante el diagnóstico

### 4.1. Reinstalación de la carga después de cada reset

El segundo diagnóstico productivo observó un marcador inicial de 0,75 kg y,
después, seis muestras de 0,25 kg. El `LOAD_RESULT_JSON` estructurado clasificó
el fallo como `model_placement_failed`: `reset_fleet()` había restablecido la
caja de práctica antes de colocar los robots del siguiente subensayo.

La sonda ahora recibe el XML seleccionado en cada `run_trial()`. Primero reinicia
la flota, después reinstala la misma caja cargada y solo entonces organiza los
empujadores. Para poder comprobar el SHA productivo anterior sin modificarlo, el
supervisor inspecciona la firma del probe desplegado: si esa versión aún no
acepta `payload_xml`, aplica una envoltura de compatibilidad a `reset_fleet()`;
si ya lo acepta, no la aplica. Esto evita tanto perder la carga como instalarla
dos veces tras el futuro despliegue.

El marcador `LOADED_PAYLOAD_READY_JSON` se emite únicamente después de comprobar
en el XML una masa de 0,75 kg. Además, el supervisor exige una muestra posterior
del servicio de propiedades de Gazebo; el marcador por sí solo no abre la
barrera de carga.

### 4.2. Ejecución del preflight en una montura ejecutable

El directorio del lease situado bajo `/run/user/1000` estaba en una montura
`noexec`. Guardar allí el plugin permitía copiarlo, pero no mapearlo como objeto
compartido. Los archivos ejecutables se preparan ahora en un directorio privado
`/tmp/robotswarm-matrix-probe-*`, con modo `0700`. El sandbox de `bwrap` monta el
script y el plugin individualmente, en solo lectura, sobre `/viewer` antes de
realizar `--chdir`; su `/tmp` interno sigue siendo un `tmpfs` aislado.

La limpieza no elimina una ruta arbitraria. Antes de retirar el workspace se
comprueban padre, nombre, propietario, modo, lista cerrada de archivos y tipos
regulares. Si cualquiera de esas condiciones cambia, la aceptación falla de
forma cerrada.

### 4.3. Identidad del `gzclient` y espacios de PID

Un primer intento rechazó incorrectamente el visor primario porque comparó el
PID informado dentro del sandbox con el PID visto desde el host. Linux publica
ambas identidades en `NSpid`. La correlación actual recorre únicamente los
descendientes del publicador ligado a la sesión, exige el ejecutable esperado,
el argumento `/viewer/plugin.so`, el display correcto y acepta el PID reportado
solo si coincide con el PID del host o con una entrada de `NSpid`.

Para detectar procesos temporales abandonados ya no se reutiliza el PID que ve
un namespace. Cada preflight recibe un token aleatorio en
`ROBOTSWARM_ACTIVE_PROBE_TOKEN`; el token se hereda por Python, `bwrap` y
`gzclient`. El supervisor recorre `/proc`, compara también el *start tick* para
evitar reutilización de PID y aplica `SIGTERM` seguido de `SIGKILL` si queda un
proceso propio. La aceptación requiere que la búsqueda final quede vacía.

### 4.4. Compatibilidad con Python 3.8

La máquina de trabajo utiliza Python 3.8. Esta versión no ofrece
`subprocess.Popen(umask=...)` ni `str.removesuffix()`. El hijo que genera archivos
privados se inicia mediante un programa fijo de `sh` que ejecuta `umask 077` y
realiza `exec` con cada argumento separado; no evalúa texto procedente del hijo.
Para retirar `.json` se utiliza una comprobación de sufijo y un corte explícito.
Las dos rutas tienen regresiones focales para evitar que una validación local
funcione solo en un Python más reciente que el host real.

### 4.5. Marcadores JSON concurrentes

El monitor de masa y la sonda oficial comparten el mismo pipe. En el intento
`loaded-n4-hybrid-mse` dos escrituras se entrelazaron y produjeron un
`LOADED_GRF_ACTIVE_JSON` malformado. El intento se rechazó correctamente y no se
interpretó su contenido parcial como estado ROS.

Los marcadores vivos se serializan ahora sin `NaN`, incluyendo prefijo, JSON y
salto de línea en una sola escritura `os.write`. Se rechaza cualquier mensaje
mayor de 4096 bytes, límite adoptado para permanecer dentro de la escritura
atómica del pipe. El lector también detecta prefijos conocidos incrustados en un
JSON roto y conserva solamente longitud, hash y tipo de marcador; no guarda el
texto potencialmente sensible.

La escritura atómica del informe tiene un propósito distinto: cada actualización
se genera en un archivo temporal privado, se sincroniza y se sustituye mediante
`os.replace`. Así, una interrupción no deja un JSON final parcialmente escrito.

### 4.6. Presupuestos de espera, HLS y limpieza

El visor utiliza un solo presupuesto de 240 s desde el clic en «Abrir visor»
hasta el primer fotograma. El tiempo consumido esperando el lease se descuenta
antes de esperar video, en lugar de iniciar un segundo timeout completo. Si el
fotograma no llega, el informe registra un estado acotado de MSE, recursos HLS,
elementos de video y mensajes visibles, sin copiar tokens.

El preflight dispone de 45 s para calentamiento, medición y cierre. El timeout
exterior debe añadir al menos cinco segundos para observar su terminación; de lo
contrario, los argumentos se rechazan antes de crear una sesión. La medición HLS
de cinco segundos debe quedar por completo dentro del proceso cargado, de la
tarea GRF y del preflight.

Un intento que agotó la espera del fotograma eliminó contenedor y red, pero el
directorio del lease tardó más y el informe quedó como `cleanup-failed`. Esto es
un fallo real de la compuerta de limpieza, aunque la causa funcional fuera HLS.
En cambio, cuando el probe termina antes de emitir su postcheck estructurado, la
destrucción comprobada del contenedor inmutable demuestra que su tarea, roster y
proceso ya no existen. El informe conserva ambas ideas por separado:
`officialProbeTaskRosterClean=false` indica que faltó el postcheck del probe, y
`taskRosterClean=true` solo puede obtenerse entonces mediante
`containerAbsent=true`. De esta forma no se presenta un falso fallo de recursos,
pero tampoco se inventa una limpieza oficial que nunca fue observada.

## 5. Evidencia histórica y local

Una corrida local histórica, denominada v11, sí aprobó el perfil cargado antes
del candidato actual. Sus ensayos de uno, dos y cuatro robots desplazaron la
caja 0,0070 m, 0,0340 m y 1,0424 m. Los RTF recalculados fueron 2,9969, 2,9962 y
2,9975, y la ganancia frente al robot individual fue 148,9143 veces. Las cuatro
unidades permanecieron conectadas mediante dos raíces y dos compañeros.

En esa misma corrida, el GRF llegó a `DONE/completed` después de que `tb3_1`
avisara a los otros tres robots. Se registraron 1598 muestras con los cuatro
contribuyentes útiles, 0,5002 m de avance hacia la meta, eficiencia 0,9946 y RTF
exterior 2,9756. La sonda visible concurrente midió 58,816 FPS de cámara, 58,831
eventos de posrenderizado por segundo y RTF 2,996 en una RTX 3080.

Estas cifras son útiles como referencia física, pero no cierran producción: la
salida original no quedó asociada al SHA productivo actual ni satisface las
correcciones posteriores del instrumento. Tampoco se reutilizan sus capturas
como si pertenecieran al gate vigente.

En el árbol de trabajo actual aprobaron 47/47 contratos focales del arnés cargado
y 60/60 contratos de la matriz ROS. Son pruebas locales del instrumento; no
demuestran por sí mismas que la carga haya sido transportada en el sitio
desplegado. El árbol contiene cambios sin desplegar sobre el commit local
`2dda980fc743eb3c19f9d245fab9a99802418b0d`, mientras que las corridas públicas de
esta sección utilizan el release
`fbef23eaae2b1b1d5be51ad3fa03e0298239289a`.

## 6. Diagnóstico productivo del 21 de julio de 2026

Todos los intentos de esta tabla usaron Chrome 150 visible, una sesión pública
nueva N=4 y el release `fbef23e…9289a`. Un intento rechazado sirve para localizar
la causa, pero nunca se suma a la aceptación.

| Intento | Observación | Clasificación |
| --- | --- | --- |
| `loaded-n4-final` | El visor HLS entregó 30,9 FPS; el arranque de Gazebo midió 49,59 FPS y RTF 2,996. El probe terminó antes de confirmar la barrera de 0,75 kg. | Rechazado; limpieza completa. |
| `loaded-n4-diagnostic` | Se obtuvo un `LOAD_RESULT_JSON` fallido y siete muestras de masa. | Rechazado como fallo del probe y de reemplazo/visibilidad de la carga. |
| `loaded-n4-diagnostic-2` | Hubo una muestra de 0,75 kg seguida de seis de 0,25 kg. | Rechazado; `model_placement_failed` permitió localizar el reset que restauraba la caja de práctica. |
| `loaded-n4-dom-controls` | La sesión llegó a `Ready`, pero no apareció un fotograma HLS dentro del presupuesto. | Rechazado como `cleanup-failed`: contenedor y red desaparecieron, pero el runtime del lease no se confirmó ausente a tiempo. |
| `loaded-n4-hybrid-mse` | MSE/H.264, los clics auditados y el visor aprobaron; el marcador activo llegó entrelazado. | Rechazado por `LOADED_GRF_ACTIVE_JSON` malformado; limpieza completa. |
| `loaded-n4-atomic-markers` | La escritura atómica eliminó el marcador malformado, pero el probe terminó antes de iniciar el GRF N=4. | Rechazado; limpieza completa. El diagnóstico estructurado de ese punto todavía necesitaba conservarse en todos los caminos de excepción. |
| `loaded-n4-classified` | El HLS inicial midió 31,0 FPS y Gazebo 49,563/49,990 FPS con RTF 2,996. Después de terminar el payload, el arnés volvió a pedir HLS cuando ya había vencido el TTL intencional de cinco minutos del lease. | Rechazado por un gate HLS final redundante; limpieza completa. |
| `loaded-n4-post-ttl-fix` | Conservó las mediciones HLS tomadas dentro del TTL, completó capacidad, GRF, restauración y limpieza sobre el mismo release. | **Aprobado** con `status=passed`, `success=true` y limpieza completa. |

Los intentos rechazados se conservan porque explican cómo se llegó al instrumento
actual; sus resultados no se mezclan entre sí. La aprobación se apoya solamente
en `loaded-n4-post-ttl-fix`, que produjo su propio conjunto completo de
documentos, hashes y capturas correlacionadas.

## 7. Aprobación productiva: `loaded-n4-post-ttl-fix`

La corrida se ejecutó entre las 22:21:26 y las 22:27:32 UTC del 21 de julio de
2026. El informe saneado terminó con `status=passed`, `success=true` y quedó
vinculado al release productivo
`fbef23eaae2b1b1d5be51ad3fa03e0298239289a`. La imagen del contenedor estaba
inmovilizada por ese SHA, la sesión N=4 llegó a `Ready` y el display, el lease,
la red privada y los masters ROS/Gazebo pertenecían a la misma sesión.

### 7.1. Video y rendimiento visible

Chrome 150 se ejecutó con GPU y en modo visible. El HLS entregó entre 30,0 y
30,8 FPS en las comprobaciones de apertura, captura y cierre del intervalo útil.
Durante la coincidencia entre carga, GRF y preflight se midieron 152 fotogramas
en 5,051 s: 30,094 FPS presentados y decodificados, sin fotogramas descartados.
Las lecturas inmediatamente anterior y posterior a la captura de `PUSH` fueron
ambas de 30,0 FPS, por encima del mínimo de 27,0 FPS.

El `gzclient` primario informó 49,548 FPS de cámara, 50,034 eventos de
posrenderizado por segundo y RTF 2,996. El preflight oficial, ejecutado mientras
el GRF cargado seguía activo, identificó el adaptador `D3D12 (NVIDIA GeForce RTX
3080)` y midió 58,469 FPS de cámara, 62,489 eventos de posrenderizado por segundo
y RTF 2,996. Por tanto, no se utilizó renderizado headless ni un renderer por
software.

Al terminar el payload, el lease ya había alcanzado su TTL deliberado y el panel
mostró el rechazo temporal esperado. Esta observación no invalida el gate: el
video se midió dentro del TTL, durante el proceso cargado y a ambos lados de la
captura correlacionada. El arreglo consistió en registrar el estado final del
lease sin volver a exigir una reproducción que, por diseño, ya debía estar
revocada.

### 7.2. Capacidad física y comportamiento GRF

Con el mismo perfil de 0,75 kg, los ensayos de uno, dos y cuatro robots
desplazaron la caja 0,0070 m, 0,0354 m y 1,0836 m, respectivamente. La ganancia
de la flota sobre el robot individual fue 154,8 veces. Los RTF recalculados de
los tres ensayos fueron 2,9951, 2,9975 y 2,9941; los cuatro robots conservaron la
estructura prevista de dos raíces en contacto con la carga y dos compañeros.

En el tramo algorítmico, un descubridor emitió el aviso a la flota completa. Los
cuatro robots registraron movimiento de búsqueda, los cuatro completaron el
encuentro y los cuatro contribuyeron al empuje. Se obtuvieron 1621 muestras GRF,
de las cuales 1618 confirmaron simultáneamente a toda la flota como útil. La caja
avanzó 0,5001 m hacia la meta, con eficiencia 0,9946 y RTF exterior 2,9942. Estos
valores satisfacen el mínimo físico de 0,50 m y demuestran búsqueda, aviso,
reunión y transporte colaborativo en una sola tarea.

### 7.3. Limpieza y conjunto de evidencia

La limpieza terminó con `cleanup.complete=true`. El postcheck oficial observó la
tarea terminal, roster vacío, ausencia de modelos `tb3_*` y restauración de la
caja de práctica de 0,25 kg. También quedaron ausentes los hijos acotados, el
preflight temporal, su workspace, el visor, el publicador, el lease, el
contenedor, la red, Chrome y su perfil efímero.

El hash del conjunto saneado es
`c9095a31ec596c4dccaf12d3ef3beb109ddd652399b44962215ffc94011a7dac`.
El [reporte saneado](assets/commissioning-2026-07/final-fbef23e/carga-n4-reporte.json)
y las tres PNG pertenecen a esta misma corrida. Se inspeccionaron para excluir
correos, UUID, tokens y rutas internas antes de incorporarlos al manifiesto:

- [Antes del probe cargado](assets/commissioning-2026-07/final-fbef23e/carga-n4-antes.png), 131134 bytes, SHA-256 `4579c14a4020fb2e3527ed3ac18926f7eaf712ef60e641938dd164084903c89a`;
- [Durante el `PUSH` cargado](assets/commissioning-2026-07/final-fbef23e/carga-n4-durante-push.png), 149175 bytes, SHA-256 `7a5be6cb6e57cabfab83a94c340a1920620e5496973822bcbed581dfb079abaa`;
- [Después del probe, con la regresión UTC visible](assets/commissioning-2026-07/final-fbef23e/contador-utc-antes.png), 58771 bytes, SHA-256 `fdfbef8f65b81178508f281130ea1ebf851aa9144bac9665a3024d77cf022206`.

Este resultado aprueba el transporte cargado N=4 del release productivo
`fbef23e…9289a`. No sustituye la repetición posterior al despliegue del candidato
final: cuando los ajustes del arnés y de ROS se publiquen, el mismo procedimiento
deberá ejecutarse otra vez sobre el nuevo SHA para comprobar que el artefacto
final conserva el comportamiento aprobado aquí.
