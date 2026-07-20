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
  --verify-grf-n4
```

The preflight and load probe must use the same ROS/Gazebo master and run back to
back. Keep both JSON results together. The `/gazebo_gui` ROS node checked by the
load probe is only a liveness precondition; the preceding preflight is the gate
that proves a real visible viewport, the NVIDIA renderer, at least 45 rendered
FPS and physics RTF of at least 2.90. Do not accept a loaded run when that
preflight is missing or failed.

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
loaded profile and the N=1/3/4 cases must be repeated on the deployed revision;
the versioned N=10 evidence in the final commissioning report is a separate run.
