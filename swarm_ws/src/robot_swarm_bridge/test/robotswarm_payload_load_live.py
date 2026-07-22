#!/usr/bin/env python3
"""Measure whether a transport payload really needs a small robot team.

This is a physics calibration probe, not a replacement for the transport
acceptance test.  It drives an aligned Burger chain at a fixed command so the
loaded crate can be checked independently of search, routing and GRF control.
The normal practice crate is restored before the script exits.
"""

import argparse
import json
import math
import re
import signal
import subprocess
import sys
import threading
import time
import uuid
import xml.etree.ElementTree as ET
from pathlib import Path

import rosnode
import rospy
from gazebo_msgs.msg import ModelState, ModelStates
from gazebo_msgs.srv import DeleteModel, SetModelState, SpawnModel
from geometry_msgs.msg import Pose, Twist
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String


PACKAGE_DIR = Path(__file__).resolve().parents[1]
LOADED_MODEL = PACKAGE_DIR / 'models' / 'transport_crate_loaded' / 'model.sdf'
PRACTICE_WORLD = PACKAGE_DIR / 'worlds' / 'swarm_arena.world'
MODEL_NAME = 'transport_object'
ROBOT_RE = re.compile(r'^tb3_(\d+)$')
TERMINAL_TASK_STATES = {'idle', 'completed', 'failed', 'stopped'}
PAYLOAD_REPLACEMENT_ATTEMPTS = 3
PAYLOAD_READINESS_SAMPLES = 2
PAYLOAD_READINESS_TIMEOUT = 5.0
PAYLOAD_MISSING_RETRY_TIMEOUT = 0.75
PAYLOAD_POSITION_TOLERANCE = 0.04


class _RecoverablePayloadRace(RuntimeError):
    """A freshly spawned payload disappeared while Gazebo was settling."""


def robot_sort_key(name):
    match = ROBOT_RE.match(str(name).strip('/'))
    return int(match.group(1)) if match else sys.maxsize


def read_model_xml(path, xpath):
    root = ET.parse(path).getroot()
    model = root.find(xpath)
    if model is None:
        raise RuntimeError('{} has no transport_object model'.format(path))
    embedded_pose = model.find('pose')
    if embedded_pose is not None:
        model.remove(embedded_pose)
    # Gazebo's spawn_sdf_model service needs a complete SDF document. The
    # loaded profile already lives below <sdf>, while the practice model is
    # embedded one level deeper in the arena world, so wrap both the same way.
    document = ET.Element('sdf', {
        'version': root.attrib.get('version', '1.6'),
    })
    document.append(model)
    return ET.tostring(document, encoding='unicode')


def push_layout(count, crate_x, crate_y):
    """Put up to two roots on the crate and extend balanced rear chains."""
    if count < 1:
        raise ValueError('robot count must be positive')

    root_x = crate_x - 0.315
    if count == 1:
        return [(root_x, crate_y)]

    lanes = (crate_y - 0.09, crate_y + 0.09)
    positions = [(root_x, lanes[0]), (root_x, lanes[1])]
    for index in range(2, count):
        lane = (index - 2) % 2
        depth = (index - 2) // 2 + 1
        positions.append((root_x - 0.15 * depth, lanes[lane]))
    return positions


def push_connections(robots, positions, crate_position):
    """Describe the two payload roots and each rear chain connection."""
    report = {}
    for index, robot in enumerate(robots):
        x, y = positions[robot]
        if index < min(2, len(robots)):
            dx = abs(x - crate_position[0]) - 0.20
            dy = abs(y - crate_position[1]) - 0.20
            surface_distance = math.hypot(max(dx, 0.0), max(dy, 0.0))
            connected = x < crate_position[0] and surface_distance <= 0.12
            report[robot] = {
                'role': 'payload_root',
                'parent': MODEL_NAME,
                'contact_distance_m': surface_distance,
                'connected': connected,
            }
            continue

        parent = robots[index - 2]
        parent_x, parent_y = positions[parent]
        distance = math.hypot(x - parent_x, y - parent_y)
        report[robot] = {
            'role': 'companion',
            'parent': parent,
            'contact_distance_m': distance,
            'connected': (
                distance <= 0.17 and report[parent]['connected']
            ),
        }
    return report


class LoadProbe:
    def __init__(self, args):
        self.args = args
        self.lock = threading.RLock()
        self.stop_requested = False
        self.roster = []
        self.roster_generation = 0
        self.models = {}
        self.model_states_generation = 0
        self.sim_time = None
        self.swarm_task = {}
        self.swarm_status_generation = 0
        self.task_status_fresh_for_cleanup = True
        self.cleanup_task_id = None
        self.delete_results = {}
        self.payload_model_xml = None
        self.command_pub = rospy.Publisher(
            '/swarm/commands', String, queue_size=10
        )

        rospy.Subscriber(
            '/fleet/robot_list', String, self._roster_cb, queue_size=1
        )
        rospy.Subscriber(
            '/gazebo/model_states', ModelStates, self._models_cb, queue_size=1
        )
        rospy.Subscriber('/clock', Clock, self._clock_cb, queue_size=1)
        rospy.Subscriber(
            '/swarm/status', String, self._swarm_cb, queue_size=1
        )
        rospy.Subscriber(
            '/fleet/delete_result', String,
            self._delete_result_cb, queue_size=5,
        )

        rospy.wait_for_service('/gazebo/delete_model', timeout=10.0)
        rospy.wait_for_service('/gazebo/spawn_sdf_model', timeout=10.0)
        rospy.wait_for_service('/gazebo/set_model_state', timeout=10.0)
        self.delete_model = rospy.ServiceProxy(
            '/gazebo/delete_model', DeleteModel
        )
        self.spawn_model = rospy.ServiceProxy(
            '/gazebo/spawn_sdf_model', SpawnModel
        )
        self.set_model_state = rospy.ServiceProxy(
            '/gazebo/set_model_state', SetModelState
        )

    def _roster_cb(self, msg):
        roster = [item.strip('/') for item in msg.data.split(',') if item]
        with self.lock:
            self.roster = roster
            self.roster_generation += 1

    def _models_cb(self, msg):
        with self.lock:
            self.models = {
                name: {
                    'position': (pose.position.x, pose.position.y, pose.position.z),
                    'yaw': math.atan2(
                        2.0 * (
                            pose.orientation.w * pose.orientation.z
                            + pose.orientation.x * pose.orientation.y
                        ),
                        1.0 - 2.0 * (
                            pose.orientation.y * pose.orientation.y
                            + pose.orientation.z * pose.orientation.z
                        ),
                    ),
                }
                for name, pose in zip(msg.name, msg.pose)
            }
            self.model_states_generation += 1

    def _clock_cb(self, msg):
        with self.lock:
            self.sim_time = msg.clock.to_sec()

    def _swarm_cb(self, msg):
        try:
            status = json.loads(msg.data)
        except (TypeError, json.JSONDecodeError):
            return
        with self.lock:
            self.swarm_task = status.get('task', {})
            self.swarm_status_generation += 1

    def _delete_result_cb(self, msg):
        try:
            result = json.loads(msg.data)
        except (TypeError, json.JSONDecodeError):
            return
        request_id = str(result.get('request_id') or '')
        if not request_id:
            return
        with self.lock:
            self.delete_results[request_id] = result

    def log(self, message):
        print('[load-probe] ' + message, file=sys.stderr, flush=True)

    def wait_for(
        self, predicate, timeout, description, continue_after_stop=False,
    ):
        deadline = time.monotonic() + timeout
        while (
            not rospy.is_shutdown()
            and (continue_after_stop or not self.stop_requested)
        ):
            with self.lock:
                if predicate():
                    return
            if time.monotonic() >= deadline:
                raise RuntimeError('timeout waiting for ' + description)
            time.sleep(0.05)
        raise RuntimeError('probe stopped while waiting for ' + description)

    def wait_sim(self, seconds):
        with self.lock:
            start = self.sim_time
        if start is None:
            raise RuntimeError('Gazebo clock is unavailable')
        self.wait_for(
            lambda: self.sim_time is not None
            and self.sim_time >= start + seconds,
            max(10.0, seconds * 2.0),
            '{:.2f} simulated seconds'.format(seconds),
        )

    def wait_for_model_observations(
        self, name, present, after_generation, samples, description,
        expected_position=None,
    ):
        """Require consecutive, fresh ModelStates observations.

        ModelStates has no header or model-instance identifier.  A cached
        name can therefore belong to the object that was just deleted.  The
        generation fence makes every accepted sample newer than the service
        operation that it is meant to confirm.
        """
        last_generation = after_generation
        matching_samples = 0
        fresh_samples = 0
        last_present = None

        def ready():
            nonlocal last_generation, matching_samples
            nonlocal fresh_samples, last_present

            generation = self.model_states_generation
            if generation <= last_generation:
                return False
            last_generation = generation
            fresh_samples += 1

            model = self.models.get(name)
            last_present = model is not None
            matches = last_present is present
            if matches and present and expected_position is not None:
                position = model.get('position')
                matches = (
                    isinstance(position, tuple)
                    and len(position) >= 3
                    and all(
                        abs(float(position[index]) - expected_position[index])
                        <= PAYLOAD_POSITION_TOLERANCE
                        for index in range(3)
                    )
                )

            matching_samples = matching_samples + 1 if matches else 0
            return matching_samples >= samples

        try:
            self.wait_for(
                ready,
                PAYLOAD_READINESS_TIMEOUT,
                description,
            )
        except RuntimeError as exc:
            # Only a fresh, authoritative absence is repairable.  A silent
            # ModelStates stream, an unexpected pose or an operator stop must
            # remain visible as the original failure.
            if (
                not self.stop_requested
                and not rospy.is_shutdown()
                and fresh_samples > 0
                and last_present is False
            ):
                raise _RecoverablePayloadRace(
                    '{} disappeared in fresh Gazebo telemetry'.format(name)
                ) from exc
            raise
        return last_generation

    def send(self, command, parameters=None):
        payload = {'command': command, 'parameters': parameters or {}}
        self.command_pub.publish(String(data=json.dumps(payload)))

    def ensure_idle_and_visible(self, external_viewer_verified=False):
        self.wait_for(
            lambda: self.command_pub.get_num_connections() > 0,
            10.0,
            '/swarm/commands subscriber',
        )
        self.wait_for(
            lambda: self.sim_time is not None and bool(self.models),
            10.0,
            'Gazebo telemetry',
        )
        self.wait_for(
            lambda: 'status' in self.swarm_task,
            5.0,
            'swarm task status',
        )
        gui_nodes = {
            name.rstrip('/') for name in rosnode.get_node_names()
            if name.rstrip('/').endswith('gazebo_gui')
        }
        if not gui_nodes and not external_viewer_verified:
            raise RuntimeError(
                'visible /gazebo_gui node or verified external viewer is required'
            )
        with self.lock:
            task_status = str(self.swarm_task.get('status') or 'idle').lower()
        if task_status not in {'idle', 'completed', 'failed', 'stopped'}:
            raise RuntimeError(
                'refusing to interrupt active task ({})'.format(task_status)
            )

    @staticmethod
    def _service_status(response):
        return ' '.join(str(response.status_message or '').lower().split())

    def _raise_if_stopping(self, description):
        if getattr(self, 'stop_requested', False) or rospy.is_shutdown():
            raise RuntimeError('probe stopped while ' + description)

    def _delete_payload_for_replacement(self):
        self._raise_if_stopping('replacing {}'.format(MODEL_NAME))
        with self.lock:
            generation = self.model_states_generation
            exists = MODEL_NAME in self.models
        if exists:
            response = self.delete_model(MODEL_NAME)
            if (
                not response.success
                and self._service_status(response)
                != 'deletemodel: model does not exist'
            ):
                raise RuntimeError(
                    'could not delete payload: ' + response.status_message
                )
        self.wait_for_model_observations(
            MODEL_NAME,
            False,
            generation,
            1,
            'fresh payload deletion',
        )

    def _spawn_payload(self, model_xml):
        self._raise_if_stopping('replacing {}'.format(MODEL_NAME))
        pose = Pose()
        pose.position.x = self.args.crate_x
        pose.position.y = self.args.crate_y
        pose.position.z = 0.1
        pose.orientation.w = 1.0
        with self.lock:
            generation = self.model_states_generation
        response = self.spawn_model(
            MODEL_NAME, model_xml, '', pose, 'world'
        )
        if not response.success:
            raise RuntimeError(
                'could not spawn payload: ' + response.status_message
            )
        self.wait_for_model_observations(
            MODEL_NAME,
            True,
            generation,
            PAYLOAD_READINESS_SAMPLES,
            'stable payload spawn',
        )

    def _set_pose(self, name, x, y, z, missing_timeout):
        state = ModelState()
        state.model_name = name
        state.reference_frame = 'world'
        state.pose.position.x = x
        state.pose.position.y = y
        state.pose.position.z = z
        state.pose.orientation.w = 1.0
        deadline = time.monotonic() + missing_timeout
        while True:
            self._raise_if_stopping('placing {}'.format(name))
            response = self.set_model_state(state)
            if response.success:
                return
            status = self._service_status(response)
            if status != 'setmodelstate: model does not exist':
                raise RuntimeError(
                    'could not place {}: {}'.format(
                        name, response.status_message
                    )
                )
            if time.monotonic() >= deadline:
                # Shutdown can begin while the service call is in flight.  It
                # is an operator/runtime stop, not a disappearing-model race.
                self._raise_if_stopping('placing {}'.format(name))
                raise _RecoverablePayloadRace(
                    '{} remained unavailable to SetModelState'.format(name)
                )
            time.sleep(0.05)

    def _place_payload_pose(self):
        with self.lock:
            generation = self.model_states_generation
        self._set_pose(
            MODEL_NAME,
            self.args.crate_x,
            self.args.crate_y,
            0.1,
            PAYLOAD_MISSING_RETRY_TIMEOUT,
        )
        self.wait_for_model_observations(
            MODEL_NAME,
            True,
            generation,
            PAYLOAD_READINESS_SAMPLES,
            'stable payload placement',
            expected_position=(self.args.crate_x, self.args.crate_y, 0.1),
        )

    def replace_payload(self, model_xml):
        for attempt in range(1, PAYLOAD_REPLACEMENT_ATTEMPTS + 1):
            self._raise_if_stopping('replacing {}'.format(MODEL_NAME))
            try:
                self._delete_payload_for_replacement()
                self._raise_if_stopping('replacing {}'.format(MODEL_NAME))
                self._spawn_payload(model_xml)
                self._raise_if_stopping('replacing {}'.format(MODEL_NAME))
                self._place_payload_pose()
                self.wait_sim(0.5)
                self.payload_model_xml = model_xml
                return
            except _RecoverablePayloadRace as exc:
                # Do not turn a shutdown that raced with Gazebo into another
                # delete/spawn cycle.
                self._raise_if_stopping('replacing {}'.format(MODEL_NAME))
                if attempt >= PAYLOAD_REPLACEMENT_ATTEMPTS:
                    raise RuntimeError(
                        'could not place {}: readiness race persisted after '
                        '{} replacement attempts'.format(
                            MODEL_NAME, PAYLOAD_REPLACEMENT_ATTEMPTS
                        )
                    ) from exc
                self.log(
                    'payload readiness was lost; retrying replacement '
                    '({}/{})'.format(
                        attempt + 1, PAYLOAD_REPLACEMENT_ATTEMPTS
                    )
                )

    def set_pose(self, name, x, y, z):
        try:
            self._set_pose(name, x, y, z, 5.0)
        except _RecoverablePayloadRace as exc:
            self._raise_if_stopping('placing {}'.format(name))
            raise RuntimeError(
                'could not place {}: model remained unavailable'.format(name)
            ) from exc

    def reset_payload_pose(self):
        try:
            self._place_payload_pose()
        except _RecoverablePayloadRace as exc:
            self._raise_if_stopping('resetting {}'.format(MODEL_NAME))
            model_xml = getattr(self, 'payload_model_xml', None)
            if model_xml is None:
                raise RuntimeError(
                    'could not place {}: no selected payload is available '
                    'for recovery'.format(MODEL_NAME)
                ) from exc
            self.log(
                'payload disappeared after replacement; reinstalling the '
                'selected profile'
            )
            self.replace_payload(model_xml)

    def reset_fleet(self, count):
        with self.lock:
            old_robots = set(self.roster)
        self.send('delete_robots')
        self.wait_for(lambda: not self.roster, 30.0, 'empty fleet')
        self.wait_for(
            lambda: not (old_robots & set(self.models)),
            8.0,
            'old Gazebo robot deletion',
        )
        self.send('spawn_robots', {
            'robot_count': count,
            'spawn_pattern': 'line',
        })
        self.wait_for(
            lambda: len(self.roster) == count,
            45.0,
            '{}-robot roster'.format(count),
        )
        self.wait_for(
            lambda: set(self.roster).issubset(self.models),
            10.0,
            'Gazebo robot models',
        )
        self.wait_sim(0.5)

    def arrange_pushers(self):
        with self.lock:
            robots = sorted(self.roster, key=robot_sort_key)
            z_by_robot = {
                robot: self.models[robot]['position'][2] for robot in robots
            }
        positions = push_layout(
            len(robots), self.args.crate_x, self.args.crate_y
        )
        for robot, (x, y) in zip(robots, positions):
            self.set_pose(robot, x, y, z_by_robot[robot])
        self.reset_payload_pose()
        self.wait_sim(0.75)
        # Clear any small contact impulse from placement before measuring.
        self.reset_payload_pose()
        self.wait_sim(0.25)
        return robots

    def stop_active_task(self):
        """Stop the correlated task before changing its robots or payload."""
        with self.lock:
            status_is_fresh = getattr(
                self, 'task_status_fresh_for_cleanup', True
            )
            task_id = str(self.swarm_task.get('task_id') or '')
            task_status = str(
                self.swarm_task.get('status') or 'idle'
            ).lower()
            expected_task_id = str(
                getattr(self, 'cleanup_task_id', None) or ''
            )

        if not status_is_fresh:
            raise RuntimeError(
                'no fresh swarm task status was observed after the child run'
            )
        if expected_task_id and task_id != expected_task_id:
            raise RuntimeError(
                'fresh swarm status did not match child task {}'.format(
                    expected_task_id
                )
            )
        terminal_states = TERMINAL_TASK_STATES
        if expected_task_id:
            terminal_states = TERMINAL_TASK_STATES - {'idle'}
        if task_status in terminal_states:
            return
        if not task_id:
            raise RuntimeError(
                'active swarm task has no task_id for correlated cleanup'
            )

        self.send('stop_task', {'task_id': task_id})
        self.wait_for(
            lambda: (
                str(self.swarm_task.get('task_id') or '') == task_id
                and str(self.swarm_task.get('status') or '').lower()
                in TERMINAL_TASK_STATES - {'idle'}
            ),
            8.0,
            'task {} to stop before cleanup'.format(task_id),
        )

    def robot_model_names(self):
        with self.lock:
            return {
                name for name in self.models
                if ROBOT_RE.fullmatch(str(name))
            }

    def run_trial(self, count, payload_xml=None):
        self.log('starting fixed-command trial with {} robot(s)'.format(count))
        self.reset_fleet(count)
        # A fleet reset can briefly invalidate Gazebo's model handle on the
        # production worker.  Reinstall the selected payload after the reset
        # so every capacity trial measures the same physical object.
        if payload_xml is not None:
            self.replace_payload(payload_xml)
        robots = self.arrange_pushers()
        publishers = {
            robot: rospy.Publisher(
                '/{}/cmd_vel'.format(robot), Twist, queue_size=1
            )
            for robot in robots
        }
        self.wait_for(
            lambda: all(pub.get_num_connections() > 0
                        for pub in publishers.values()),
            5.0,
            'Burger cmd_vel subscribers',
        )

        with self.lock:
            crate_start = self.models[MODEL_NAME]['position'][:2]
            robot_start = {
                robot: self.models[robot]['position'][:2] for robot in robots
            }
            sim_start = self.sim_time
        wall_start = time.monotonic()
        maximum_progress = 0.0
        maximum_lateral_drift = 0.0
        command = Twist()
        command.linear.x = self.args.command_speed

        try:
            while not rospy.is_shutdown() and not self.stop_requested:
                with self.lock:
                    now = self.sim_time
                    crate = self.models.get(MODEL_NAME, {}).get('position')
                if now is None or crate is None:
                    raise RuntimeError(
                        'Gazebo telemetry disappeared during push'
                    )
                if now - sim_start >= self.args.push_duration:
                    break
                maximum_progress = max(
                    maximum_progress, crate[0] - crate_start[0]
                )
                maximum_lateral_drift = max(
                    maximum_lateral_drift, abs(crate[1] - crate_start[1])
                )
                for publisher in publishers.values():
                    publisher.publish(command)
                time.sleep(0.02)

            # The acceptance contract measures the interval during which the
            # positive command is active.  Take both endpoints before the
            # repeated zero command below; that safety sequence takes about
            # half a simulated second at the target RTF and is not pushing.
            with self.lock:
                sim_push_end = self.sim_time
            wall_push_end = time.monotonic()
        finally:
            stop = Twist()
            for _ in range(8):
                for publisher in publishers.values():
                    publisher.publish(stop)
                time.sleep(0.02)
            for publisher in publishers.values():
                publisher.unregister()

        with self.lock:
            crate_end = self.models[MODEL_NAME]['position'][:2]
            robot_end = {
                robot: self.models[robot]['position'][:2] for robot in robots
            }

        maximum_progress = max(maximum_progress, crate_end[0] - crate_start[0])
        maximum_lateral_drift = max(
            maximum_lateral_drift, abs(crate_end[1] - crate_start[1])
        )
        robot_progress = {
            robot: robot_end[robot][0] - robot_start[robot][0]
            for robot in robots
        }
        connections = push_connections(robots, robot_end, crate_end)
        wall_elapsed = wall_push_end - wall_start
        sim_elapsed = sim_push_end - sim_start
        return {
            'robot_count': count,
            'command_speed_mps': self.args.command_speed,
            'push_duration_sim_s': sim_elapsed,
            'push_duration_wall_s': wall_elapsed,
            'real_time_factor': (
                sim_elapsed / wall_elapsed if wall_elapsed > 0.0 else None
            ),
            'payload_forward_progress_m': maximum_progress,
            'payload_final_forward_progress_m': crate_end[0] - crate_start[0],
            'payload_maximum_lateral_drift_m': maximum_lateral_drift,
            'robot_forward_progress_m': robot_progress,
            'final_push_connections': connections,
            'final_connected_robot_count': sum(
                1 for item in connections.values() if item['connected']
            ),
        }

    def cleanup(self, practice_xml):
        was_stopped = self.stop_requested
        self.stop_requested = False
        try:
            # A killed child may leave its ROS task running.  Do not replace
            # the payload under a live controller: stop the exact task first.
            self.stop_active_task()
            delete_request_id = 'load-cleanup-{}'.format(
                uuid.uuid4().hex[:8]
            )
            with self.lock:
                roster_generation = self.roster_generation
                model_states_generation = self.model_states_generation
            self.send('delete_robots', {
                'request_id': delete_request_id,
            })
            self.wait_for(
                lambda: delete_request_id in self.delete_results,
                10.0,
                'correlated fleet delete result',
            )
            with self.lock:
                delete_result = dict(
                    self.delete_results[delete_request_id]
                )
            remaining = delete_result.get('remaining_robot_ids')
            if not isinstance(remaining, list):
                raise RuntimeError(
                    'fleet delete result did not report remaining robots'
                )
            remaining = [
                str(name).strip('/') for name in remaining
                if str(name).strip('/')
            ]
            if remaining:
                raise RuntimeError(
                    'fleet delete left active robots: '
                    + ', '.join(sorted(remaining, key=robot_sort_key))
                )
            self.wait_for(
                lambda: (
                    self.roster_generation > roster_generation
                    and not self.roster
                ),
                30.0,
                'post-command cleanup fleet deletion',
            )
            self.wait_for(
                lambda: (
                    self.model_states_generation > model_states_generation
                    and not self.robot_model_names()
                ),
                8.0,
                'post-command cleanup Gazebo model deletion',
            )
            self.replace_payload(practice_xml)
            self.cleanup_task_id = None
        finally:
            self.stop_requested = was_stopped


def rounded(value):
    if isinstance(value, dict):
        return {key: rounded(item) for key, item in value.items()}
    if isinstance(value, float):
        return round(value, 4) if math.isfinite(value) else None
    return value


def evaluate(single, roots, fleet, args):
    failures = []
    single_progress = single['payload_forward_progress_m']
    fleet_progress = fleet['payload_forward_progress_m']
    gain = fleet_progress / max(single_progress, 0.005)
    if single_progress > args.single_max_progress:
        failures.append(
            'one robot moved the loaded crate more than {:.3f} m'.format(
                args.single_max_progress
            )
        )
    if roots['payload_forward_progress_m'] > args.root_only_max_progress:
        failures.append(
            'two payload roots moved the loaded crate more than {:.3f} m'.format(
                args.root_only_max_progress
            )
        )
    if fleet_progress < args.fleet_min_progress:
        failures.append(
            '{} robots moved the loaded crate less than {:.3f} m'.format(
                args.fleet_count, args.fleet_min_progress
            )
        )
    if gain < args.minimum_gain:
        failures.append(
            'multi-robot payload progress gain was below {:.1f}x'.format(
                args.minimum_gain
            )
        )
    lagging = [
        name for name, progress in fleet['robot_forward_progress_m'].items()
        if progress < args.min_robot_progress
    ]
    if lagging:
        failures.append(
            'fleet robots did not all advance: ' + ', '.join(
                sorted(lagging, key=robot_sort_key)
            )
        )
    if fleet['final_connected_robot_count'] != args.fleet_count:
        failures.append(
            'the complete fleet was not connected in the final push chain'
        )
    for trial in (single, roots, fleet):
        rtf = trial['real_time_factor']
        if rtf is None or rtf < args.min_rtf:
            failures.append(
                '{}-robot trial real-time factor was below {:.2f}'.format(
                    trial['robot_count'], args.min_rtf
                )
            )
    return failures, gain


def loaded_grf_command(args, task_id):
    """Build the child acceptance command while the heavy crate is installed."""
    runner = Path(__file__).with_name('robotswarm_live_acceptance.py')
    return [
        sys.executable,
        str(runner),
        '--scenario',
        'transport_grf_n4',
        '--min-rtf',
        str(args.min_rtf),
        '--task-id',
        task_id,
        '--delete-after',
    ]


def run_loaded_grf(probe, args):
    """Run search, rendezvous and GRF before the practice crate is restored."""
    task_id = 'loaded-grf-n4-{}'.format(uuid.uuid4().hex[:8])
    command = loaded_grf_command(args, task_id)
    probe.log('starting loaded transport_grf_n4 acceptance')
    process = subprocess.Popen(command)
    with probe.lock:
        probe.cleanup_task_id = task_id
        probe.task_status_fresh_for_cleanup = False
    interrupt_sent_at = None
    terminate_sent_at = None
    while process.poll() is None:
        if probe.stop_requested and interrupt_sent_at is None:
            process.send_signal(signal.SIGINT)
            interrupt_sent_at = time.monotonic()
        elif (
            interrupt_sent_at is not None
            and terminate_sent_at is None
            and time.monotonic() - interrupt_sent_at >= 20.0
        ):
            process.terminate()
            terminate_sent_at = time.monotonic()
        elif (
            terminate_sent_at is not None
            and time.monotonic() - terminate_sent_at >= 10.0
        ):
            process.kill()
        time.sleep(0.2)

    with probe.lock:
        status_generation_at_exit = probe.swarm_status_generation
    probe.wait_for(
        lambda: (
            probe.swarm_status_generation > status_generation_at_exit
            and str(probe.swarm_task.get('task_id') or '') == task_id
        ),
        5.0,
        'fresh status for child task {} after exit'.format(task_id),
        continue_after_stop=True,
    )
    with probe.lock:
        probe.task_status_fresh_for_cleanup = True
    return {
        'scenario': 'transport_grf_n4',
        'task_id': task_id,
        'exit_code': process.returncode,
        'passed': process.returncode == 0,
        'fresh_cleanup_status_observed': True,
    }


def build_parser():
    parser = argparse.ArgumentParser(
        description=(
            'Verify that the loaded transport crate resists one Burger but '
            'moves under a coordinated Burger chain in visible Gazebo.'
        )
    )
    parser.add_argument('--fleet-count', type=int, default=4)
    parser.add_argument('--command-speed', type=float, default=0.16)
    parser.add_argument('--push-duration', type=float, default=12.0)
    parser.add_argument('--crate-x', type=float, default=-0.8)
    parser.add_argument('--crate-y', type=float, default=-3.0)
    parser.add_argument('--single-max-progress', type=float, default=0.05)
    parser.add_argument('--root-only-max-progress', type=float, default=0.06)
    parser.add_argument('--fleet-min-progress', type=float, default=0.20)
    parser.add_argument('--min-robot-progress', type=float, default=0.05)
    parser.add_argument('--minimum-gain', type=float, default=4.0)
    parser.add_argument('--min-rtf', type=float, default=2.90)
    parser.add_argument(
        '--verify-grf-n4',
        action='store_true',
        help=(
            'after the capacity trials, run the complete N=4 search, notice, '
            'rendezvous and GRF acceptance while the loaded crate is installed'
        ),
    )
    parser.add_argument(
        '--external-viewer-verified',
        action='store_true',
        help=(
            'accept a visible GPU viewer verified by the supervising host '
            'acceptance runner instead of requiring an in-container '
            '/gazebo_gui node'
        ),
    )
    return parser


def main():
    args = build_parser().parse_args()
    if args.fleet_count < 3:
        raise SystemExit('--fleet-count must be at least 3')
    if args.command_speed <= 0.0 or args.push_duration <= 0.0:
        raise SystemExit('command speed and push duration must be positive')

    loaded_xml = read_model_xml(
        LOADED_MODEL, "./model[@name='transport_object']"
    )
    practice_xml = read_model_xml(
        PRACTICE_WORLD, "./world/model[@name='transport_object']"
    )
    rospy.init_node(
        'robotswarm_payload_load_live', anonymous=True, disable_signals=True
    )
    probe = LoadProbe(args)

    def request_stop(_signum, _frame):
        probe.stop_requested = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    result = {
        'profile': 'transport_crate_loaded',
        'profile_mass_kg': 0.75,
        'profile_friction': 0.25,
        'passed': False,
        'failures': [],
    }
    cleanup_needed = False
    try:
        probe.ensure_idle_and_visible(args.external_viewer_verified)
        cleanup_needed = True
        single = probe.run_trial(1, loaded_xml)
        roots = probe.run_trial(2, loaded_xml)
        fleet = probe.run_trial(args.fleet_count, loaded_xml)
        failures, gain = evaluate(single, roots, fleet, args)
        result.update({
            'single_robot_trial': rounded(single),
            'root_only_trial': rounded(roots),
            'fleet_trial': rounded(fleet),
            'payload_progress_gain': rounded(gain),
            'thresholds': {
                'single_robot_maximum_progress_m': args.single_max_progress,
                'two_root_maximum_progress_m': args.root_only_max_progress,
                'fleet_minimum_progress_m': args.fleet_min_progress,
                'minimum_robot_progress_m': args.min_robot_progress,
                'minimum_payload_progress_gain': args.minimum_gain,
                'minimum_real_time_factor': args.min_rtf,
            },
            'failures': failures,
            'passed': not failures,
        })
        if args.verify_grf_n4:
            grf = run_loaded_grf(probe, args)
            result['loaded_grf_trial'] = grf
            if not grf['passed']:
                result['failures'].append(
                    'loaded transport_grf_n4 acceptance failed with exit code '
                    + str(grf['exit_code'])
                )
    except Exception as exc:
        result['failures'].append(
            '{}: {}'.format(type(exc).__name__, exc)
        )
    finally:
        if cleanup_needed:
            try:
                probe.cleanup(practice_xml)
            except Exception as exc:
                result['failures'].append('cleanup failed: ' + str(exc))
        result['passed'] = not result['failures']

    print(
        'LOAD_RESULT_JSON ' + json.dumps(
            rounded(result), sort_keys=True, separators=(',', ':'),
            allow_nan=False,
        ),
        flush=True,
    )
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    sys.exit(main())
