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
python3 /catkin_ws/src/robot_swarm_bridge/test/robotswarm_payload_load_live.py \
  --fleet-count 4 \
  --min-rtf 2.90
```

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

The final `LOAD_RESULT_JSON` line contains the measured payload and per-robot
travel.  The probe deletes its robots and restores the normal green practice
crate even after a failed gate.  An interrupt also sends zero velocity before
cleanup.

This probe isolates the Gazebo load capacity.  A physical load-sharing claim
still needs the normal `transport_grf_n4` acceptance case to pass with this
loaded profile installed, because that second test exercises search,
notification, rendezvous, role assignment and the GRF controller rather than
fixed calibration commands.

## Commissioning result

The loaded profile moved 0.0072 m with one Burger and 0.0336 m with the two
payload roots alone. With two companions pushing through those roots, all four
robots stayed connected and moved the crate 1.1796 m: 164.8 times the
single-robot result. The normal search/rendezvous transport matrix also passed
with 1, 3, 4, and 10 robots, with every robot moving and joining the reported
payload location before coordinated transport.
