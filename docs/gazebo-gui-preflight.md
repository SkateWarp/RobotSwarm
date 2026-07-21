# Gazebo GUI rendering preflight

This probe answers three different questions without treating them as the same
measurement:

- Did a real `gzclient` process create an on-screen user-camera viewport?
- Which OpenGL device rendered that viewport, and does it match NVIDIA?
- What rendered FPS did the user camera sustain, and what physics real-time
  factor did the server sustain during the same interval?

Rendered FPS comes from Gazebo Classic's
[`gazebo::rendering::Camera::AvgFPS()`](https://github.com/gazebosim/gazebo-classic/blob/gazebo11/gazebo/rendering/Camera.hh).
The probe also counts post-render callbacks over the same window and requires
that observed frame rate to meet the limit, so a stale rolling camera average
cannot pass by itself.
Physics RTF is calculated independently from the simulation-time and real-time
deltas in the world's Gazebo
[`WorldStatistics`](https://github.com/gazebosim/gazebo-classic/blob/gazebo11/gazebo/msgs/world_stats.proto)
messages. This works even before sensor-bearing robots are spawned. A 3.0 RTF
means physics is advancing at roughly three simulation seconds per wall
second; it does not mean the screen is drawing three frames per second.

## Build and run

Build the workspace in the same image or checkout used by the simulation:

```bash
cd swarm_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

With `gzserver` already running and the WSLg display available inside the
container, run:

```bash
rosrun robot_swarm_bridge gazebo_gui_preflight.py \
  --min-render-fps 45 \
  --min-real-time-factor 2.90 \
  --report /tmp/robotswarm-gui-report.json
```

The command starts one temporary, visible `gzclient`, attaches it to the
existing `GAZEBO_MASTER_URI`, waits two seconds, measures for five seconds, and
then stops only the client it started. It never starts, pauses, restarts, or
stops `gzserver`, and it does not signal an already-running Gazebo window. A
second Gazebo window is expected while the probe runs.

The default GPU match is `nvidia|geforce|rtx`, which accepts WSLg renderer names
such as `D3D12 (NVIDIA GeForce RTX 3080)` and rejects `llvmpipe`. Use
`--gpu-pattern` only when deliberately validating another hardware renderer.
The plugin is normally found from the sourced catkin prefix. An explicit path
can be supplied when diagnosing an unusual install layout:

```bash
rosrun robot_swarm_bridge gazebo_gui_preflight.py \
  --plugin /catkin_ws/devel/lib/librobotswarm_gazebo_gui_probe.so
```

A successful result looks like this:

```text
Gazebo GUI preflight passed
  renderer: D3D12 (NVIDIA GeForce RTX 3080) | Microsoft Corporation
  viewport: gzclient_camera 1280x720
  rendered FPS: 60.12 (gazebo::rendering::Camera::AvgFPS, 5.0 s)
  post-render rate: 60.24 FPS (observed callbacks over the same window)
  physics RTF: 2.976 (gazebo.msgs.WorldStatistics delta(sim_time)/delta(real_time); this is not FPS)
```

The command exits nonzero for a missing display, missing plugin, early
`gzclient` exit, software or unexpected GPU renderer, inactive viewport,
insufficient FPS, missing RTF data, or an RTF below the selected threshold.
The JSON report records the `gzclient` PID and executable so a stale report from
the shared desktop cannot pass the check.

## Commissioning result

The visible WSL session on the NVIDIA GeForce RTX 3080 passed with a
2258x1277 viewport, 58.631 Gazebo camera FPS, a 60.774 Hz observed post-render
rate, and a 2.996 physics real-time factor. The result was measured with the
window visible; it was not a headless or prerecorded run.

A later probe against the exact one-robot transport scene also used a visible
`gzclient` and the D3D12 NVIDIA renderer. It measured a 1618x869 viewport,
61.888 camera FPS, 62.498 post-render callbacks per second, and a 2.997 physics
RTF. The transport runner that followed on the same server completed at RTF
2.9964. These two RTF values are kept separate: the first belongs to the GUI
probe immediately before the task, while the second covers the algorithm run.

The subsequent exact N=3 candidate scene also passed in a visible window. With
the same 1618x869 viewport and D3D12 NVIDIA GeForce RTX 3080 renderer, it
measured 57.907 camera FPS, 58.887 post-render callbacks per second, and RTF
2.996. Its transport runner completed separately at approximately the same RTF,
with zero collisions.

La corrida local cargada v10 añadió una primera medición superpuesta a `PUSH`,
pero el conjunto se rechazó porque el intervalo de capacidad incluía la parada
segura. La repetición v11 corrigió ese defecto y cerró la compuerta local. Su
sonda concurrente identificó un viewport 1618×869 y D3D12 en la NVIDIA GeForce
RTX 3080; midió 58,816 FPS de cámara, 58,831 eventos de posrenderizado por
segundo y RTF 2,996. El GRF correlacionado alcanzó `DONE` a RTF exterior 2,9756,
con los cuatro robots empujando y cero contactos inesperados. Esta evidencia
aprueba el rendimiento gráfico local durante el empuje cargado. Todavía no
sustituye la sonda sobre la revisión desplegada ni demuestra aislamiento de dos
visores públicos.

This is a local graphics preflight, not a viewer-isolation test. Passing it does
not prove that separate users have private displays or streams, and it does not
exercise H.264/NVENC, MediaMTX, WHEP, ICE, or TURN.
