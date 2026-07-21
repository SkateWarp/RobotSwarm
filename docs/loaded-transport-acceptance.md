# Loaded transport acceptance

The arena's green 0.25 kg crate is a fast practice payload.  It is useful for
search, rendezvous and controller regression tests, but one TurtleBot3 Burger
can move it.  That does not demonstrate physical load sharing.

The separate `transport_crate_loaded` profile is 0.75 kg with a 0.25 contact
friction coefficient.  The values were chosen from a simple traction bracket:

- loaded crate breakaway estimate: `0.75 * 0.25 * g = 1.84 N`
- stock 1 kg Burger traction estimate: `1.00 * 0.10 * g = 0.98 N`
- four-Burger traction estimate: `4 * 0.98 = 3.92 N`

The estimates only choose a reasonable profile.  The simulator result is the
acceptance evidence.

## Capacity probe

Run the probe only while the visible GPU Gazebo session is idle.  It refuses
to run without a `/gazebo_gui` node or while a swarm task is active.

```bash
source /opt/ros/noetic/setup.bash
source /catkin_ws/devel/setup.bash
rosrun robot_swarm_bridge gazebo_gui_preflight.py \
  --min-render-fps 45 \
  --min-real-time-factor 2.90 \
  --report /tmp/robotswarm-loaded-gui-preflight.json
python3 /catkin_ws/src/robot_swarm_bridge/test/robotswarm_payload_load_live.py \
  --fleet-count 4 \
  --min-rtf 2.90 \
  --external-viewer-verified \
  --verify-grf-n4
```

The preflight and load probe must use the same ROS/Gazebo master and run back to
back. Keep both JSON results together. The `/gazebo_gui` ROS node checked by the
load probe is only a legacy in-container liveness precondition. The production
viewer runs on the GPU host, outside the ROS container, so its supervised gate
passes `--external-viewer-verified` after binding the viewer to the same session.
The preceding preflight is still the evidence that proves a real visible
viewport, the NVIDIA renderer, at least 45 rendered FPS and physics RTF of at
least 2.90. Do not use that option or accept a loaded run when the supervising
viewer evidence is missing or failed.

The probe temporarily replaces `transport_object` with the amber loaded
crate.  It applies the same 0.16 m/s command to one Burger, to the two direct
payload roots, and then to four Burgers arranged as those same roots with a
companion behind each one.  It passes only when:

- one robot moves the crate no more than 0.05 m in 12 simulated seconds;
- the two roots alone move it no more than 0.06 m;
- four robots move it at least 0.20 m;
- fleet payload progress is at least four times the single-robot result;
- every fleet robot advances at least 0.05 m and remains connected to its
  declared payload-root or companion contact; and
- all three trials maintain the requested real-time factor.

With `--verify-grf-n4`, the same invocation keeps the loaded crate installed
long enough to run the normal four-robot acceptance. That second phase starts
the payload outside initial sensor range and requires SEARCH, finder notice,
fleet rendezvous and the synchronized GRF push before cleanup.

The final `LOAD_RESULT_JSON` line contains the measured payload and per-robot
travel.  The probe deletes its robots and restores the normal green practice
crate even after a failed gate.  An interrupt also sends zero velocity before
cleanup.

Cleanup is intentionally fail-closed. The probe records roster and Gazebo
generations before issuing `delete_robots`, attaches a unique `request_id` and
accepts only the matching `/fleet/delete_result` with an empty
`remaining_robot_ids` list. It then requires a post-command empty roster and no
`tb3_*` model. The practice crate is not restored if the correlated task does
not stop, deletion is partial, or either observation is missing. When the GRF
child has to be interrupted, the parent waits for a fresh status carrying that
child's explicit task ID and uses a bounded SIGINT→SIGTERM→SIGKILL escalation.

The fixed-command phase isolates Gazebo load capacity. A physical load-sharing
claim is accepted only when the optional `transport_grf_n4` phase also passes,
because that phase exercises search, notification, rendezvous, role assignment
and the GRF controller rather than calibration commands.

## Notas de diagnóstico del probe

La aparición de `transport_object` en `/gazebo/model_states` no garantiza que
`/gazebo/set_model_state` ya pueda resolverlo en el mismo ciclo. Gazebo puede
publicar primero el estado y responder durante unos milisegundos que el modelo
no existe. La colocación reintenta únicamente ese mensaje cada 50 ms y durante
un máximo de cinco segundos. Una respuesta distinta falla de inmediato; el
reintento no sirve para ocultar errores generales del servicio.

`--external-viewer-verified` existe para la topología de producción, donde el
`gzclient` visible se ejecuta en el host GPU y no registra `/gazebo_gui` dentro
del contenedor ROS. La opción no descubre ni certifica por sí sola una ventana:
solo el supervisor que ya correlacionó proceso, lease, display y masters puede
entregarla después de aprobar el preflight visible. Sin esa correlación, el
probe debe ejecutarse sin la opción y conserva la exigencia fail-closed del nodo
`/gazebo_gui`.

El umbral que publica el runner GRF es dinámico: se calcula con la distancia a
la meta al comenzar `PUSH`, menos la tolerancia de llegada, y se limita por el
valor configurado. Por redondeo o por un movimiento previo puede ser 0,4999 m.
El supervisor vuelve a calcular y valida ese eco, incluida su base y el epsilon
de 0,001 m, pero mantiene en otra comprobación el mínimo físico de avance real
en 0,50 m. Un eco coherente de 0,4999 m no permite aprobar un desplazamiento de
0,4999 m.

Los 12,0 s de cada ensayo de capacidad abarcan solo el comando positivo. Los
ocho mensajes cero de la parada segura se publican después de capturar los
extremos simulado y de pared y no forman parte de `push_duration_*`. Esta
separación es relevante a RTF 3,0: la parada añade aproximadamente 0,52 s
simulados y el gate exterior rechaza una duración mayor que 12,25 s.

## Historical commissioning observation (not a final gate)

The loaded profile moved 0.0072 m with one Burger and 0.0336 m with the two
payload roots alone. With two companions pushing through those roots, all four
robots stayed connected and moved the crate 1.1796 m: 164.8 times the
single-robot result. The normal search/rendezvous transport matrix also passed
with 1, 3, 4, and 10 robots, with every robot moving and joining the reported
payload location before coordinated transport.

This observation predates the current candidate and its raw output was not
retained with a final commit SHA. It is useful for choosing representative
parameters, but it is not evidence for the production acceptance gate. The
current local candidate has since repeated the practice-payload N=1 case in a
visible window: it reached `DONE`, advanced the crate 0.5005 m at RTF 2.9964,
confirmed its single useful pusher in 100% of the applicable samples, and
recorded zero collisions. That result validates the one-robot lifecycle only;
it does not demonstrate physical load sharing. A subsequent practice-payload
N=3 run also reached `DONE`: one finder notified both teammates, all three
robots connected and pushed, and the crate advanced approximately 0.5005 m at
RTF 2.996 with zero collisions. This validates local three-robot coordination,
not the 0.75 kg capacity claim.

## Estado local del candidato cargado

El primer GRF N=4 cargado agotó el tope anterior de 180 s después de unos 70 s
de `PUSH`: había avanzado 0,2335 m, acumulado 752 muestras y mantenido 4/4
contribuidores útiles, sin colisiones. La cronología justificó límites de pared
de 60/100/190 s para `SEARCH`/`APPROACH`/`PUSH`, bajo un tope total de 355 s;
el requisito de 0,50 m no cambió.

La corrida v10 ya había completado la física, pero el gate integral la rechazó
porque las duraciones de capacidad incluían aproximadamente 0,52 s de la parada
segura. La medición se corrigió y se cubrió con una regresión. En v11, los
ensayos de uno, dos y cuatro robots avanzaron 0,0070, 0,0340 y 1,0424 m. Sus
duraciones simuladas fueron 12,048, 12,024 y 12,000 s, con RTF recalculados de
2,9969, 2,9962 y 2,9975. La ganancia exterior recalculada fue 148,9143×. Las
cuatro unidades permanecieron conectadas mediante dos raíces sobre la carga y
dos compañeros.

El GRF v11 terminó `SEARCH` en 39,2083 s de pared, entró en `PUSH` a los
115,4389 s y llegó a `DONE/completed` a los 252,6 s. `tb3_1` informó el hallazgo
a los otros tres robots; los cuatro buscaron, completaron el rendezvous y
empujaron. Se registraron 1598 muestras, todas con la flota completa útil,
0,5002 m de avance hacia la meta, eficiencia 0,9946 y RTF exterior 2,9756. La
métrica ground-truth registró un atraque declarado de raíz y el contador de
seguridad ya filtrado no registró contactos.
La sonda visible concurrente con `PUSH` midió 58,816 FPS de cámara, 58,831
eventos de posrenderizado por segundo y RTF 2,996 en la RTX 3080, con viewport
1618×869.

La primera evaluación exterior de v11 todavía produjo un falso rechazo: asumía
que cada subensayo debía usar `tb3_0…tb3_{N-1}`. El gestor asignó correctamente
namespaces frescos y monotónicos: `tb3_0`, luego `tb3_1`–`tb3_2` y finalmente
`tb3_3`–`tb3_6`. La validación corregida permite un ordinal inicial arbitrario,
pero exige nombres `tb3_<n>` canónicos, mapas de progreso y conexión idénticos,
bloques contiguos sin reutilización y secuencia monotónica entre ensayos. Los
37/37 contratos existentes y la evidencia real v11 aprobaron después de la
corrección.

Una revisión P2 posterior encontró una frontera adicional en la captura de
`PUSH`. El arnés comprobaba correlación, masa, fase y progreso antes y después de
la PNG, pero podía conservarla aunque el HLS hubiese caído por debajo del mínimo
durante ese intervalo. La validación usa ahora el mismo
`MATRIX.MINIMUM_BROWSER_VIDEO_FPS` de la matriz, actualmente 27,0 FPS: ambas
lecturas decodificadas que rodean la captura deben ser finitas y alcanzar el
umbral. Una regresión con 26,9 FPS antes y 27,0 FPS después demuestra el rechazo.
Esta regresión forma parte de los 38/38 contratos vigentes; no se volvió a
etiquetar la captura local anterior como si el gate hubiese sido medido en
producción.

El mismo cierre exige que el historial del task avance de manera no decreciente
`SEARCH` → `APPROACH` → `PUSH`. Una secuencia que vuelva de `PUSH` a `APPROACH`
falla aunque conserve el mismo identificador, masa y roster. La suite ROS que
incluye esta regresión terminó en 416/416.

I-088 añadió una comprobación temporal que no modifica ni vuelve a interpretar
la corrida física v11. Un contador agregado podía terminar acompañado por una
evidencia final válida de atraque y ocultar que uno de sus episodios había
comenzado antes. Muestrear estados a 10 Hz tampoco bastaba, y sellar el episodio
en el `task_lock` del orquestador todavía dependía del orden de callbacks. La
autoridad quedó en la evaluación de seguridad: `ObstacleAvoidance` detecta el
flanco filtrado sincrónicamente y `CollaborativeTransport` aporta la tarea, fase,
UUID de fuente, secuencia de origen, secuencia de control y tiempos de simulación
y pared.

Ese origen publica un stream v2 de hasta 128 eventos dentro del mismo
`/transport/status`, incluido el terminal. El orquestador valida continuidad,
watermark, ventana de tarea, UUID, identidad, fase, tiempos y capacidad, y copia
solo novedades de manera idempotente. El `Bool` de compatibilidad no incrementa
el contador durante transporte. El arnés conserva tiempos de origen y de
observación separados y falla ante huecos, reinicios, regresiones o capacidad
agotada.

La señal ya excluye mediante la máscara de seguridad los contactos con la carga
y los vecinos declarados de la cadena. Por ello todo flanco filtrado restante es
inesperado, incluso en `PUSH` y `DONE`. El atraque se demuestra de forma
independiente mediante contacto carga/predecesor, geometría ground-truth y
métricas GRF; una evidencia final válida nunca reclasifica un contacto de
seguridad.

Con ello, capacidad, GRF, preflight y limpieza aprueban la compuerta cargada
local. El cierre completo vigente del árbol terminó con ROS 427/427, frontend
149/149 en 28 suites, worker 124/124 y backend con 250 pruebas aprobadas más 8
opt-in omitidas por diseño en la ejecución sin PostgreSQL configurado. El filtro
ordinario aprobó 250/250, las focales de cuentas 23/23 y las ocho opt-in 8/8 contra
PostgreSQL 17.10. Los siete arneses offline sumaron 193/193 contratos
(16+38+13+44+37+30+15). La matriz incluye ahora un contrato N=2 que exige dos
raíces sobre la carga, cero compañeros y dos empujadores útiles;
todavía no existe una corrida física N=2 del GRF. Esta aceptación no corresponde
todavía al release: no se ejecutaron CI, despliegue, repetición sobre el SHA
productivo ni capturas sanitizadas para ese cierre. La evidencia versionada de
N=10 que aparece en el informe final corresponde a una corrida distinta.

La verificación integrada incluyó además `py_compile` sobre 14 módulos, sintaxis
Bash, lint de 75 archivos frontend, build de producción y `git diff --check`;
todos esos gates quedaron en verde dentro del árbol local.
