#!/usr/bin/env python3
"""ROS-free checks for the loaded transport capacity probe."""

import importlib.util
from pathlib import Path
import sys
import types
import unittest
from unittest import mock


class _Message:
    def __init__(self, *args, **kwargs):
        for key, value in kwargs.items():
            setattr(self, key, value)


def _module(name, **values):
    module = types.ModuleType(name)
    for key, value in values.items():
        setattr(module, key, value)
    return module


def _load_probe():
    stubs = {
        'rosnode': _module('rosnode', get_node_names=lambda: []),
        'rospy': _module('rospy'),
        'gazebo_msgs': _module('gazebo_msgs'),
        'gazebo_msgs.msg': _module(
            'gazebo_msgs.msg', ModelState=_Message, ModelStates=_Message
        ),
        'gazebo_msgs.srv': _module(
            'gazebo_msgs.srv', DeleteModel=_Message,
            SetModelState=_Message, SpawnModel=_Message,
        ),
        'geometry_msgs': _module('geometry_msgs'),
        'geometry_msgs.msg': _module(
            'geometry_msgs.msg', Pose=_Message, Twist=_Message
        ),
        'rosgraph_msgs': _module('rosgraph_msgs'),
        'rosgraph_msgs.msg': _module(
            'rosgraph_msgs.msg', Clock=_Message
        ),
        'std_msgs': _module('std_msgs'),
        'std_msgs.msg': _module('std_msgs.msg', String=_Message),
    }
    originals = {name: sys.modules.get(name) for name in stubs}
    try:
        sys.modules.update(stubs)
        path = Path(__file__).with_name('robotswarm_payload_load_live.py')
        spec = importlib.util.spec_from_file_location(
            'robotswarm_payload_load_live_under_test', path
        )
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module
    finally:
        for name, original in originals.items():
            if original is None:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = original


PROBE = _load_probe()


class PayloadLoadProbeTests(unittest.TestCase):
    def args(self):
        return types.SimpleNamespace(
            fleet_count=4,
            single_max_progress=0.05,
            root_only_max_progress=0.06,
            fleet_min_progress=0.20,
            minimum_gain=4.0,
            min_robot_progress=0.05,
            min_rtf=2.70,
        )

    @staticmethod
    def trial(
        count, payload_progress, robot_progress, rtf=2.95,
        connected_count=None,
    ):
        return {
            'robot_count': count,
            'payload_forward_progress_m': payload_progress,
            'robot_forward_progress_m': robot_progress,
            'real_time_factor': rtf,
            'final_connected_robot_count': (
                count if connected_count is None else connected_count
            ),
        }

    def test_layout_uses_two_balanced_push_chains(self):
        positions = PROBE.push_layout(8, -0.8, -1.6)
        self.assertEqual(8, len(positions))
        self.assertEqual(2, len({y for _, y in positions}))
        self.assertEqual(positions[0][0], positions[1][0])
        self.assertEqual(positions[2][0], positions[3][0])
        self.assertLess(positions[2][0], positions[0][0])
        self.assertEqual(len(positions), len(set(positions)))

    def test_production_rtf_gate_is_the_default(self):
        args = PROBE.build_parser().parse_args([])

        self.assertEqual(2.90, args.min_rtf)
        self.assertFalse(args.verify_grf_n4)
        self.assertFalse(args.external_viewer_verified)

    def test_external_viewer_verification_is_explicit_and_fail_closed(self):
        probe = object.__new__(PROBE.LoadProbe)
        probe.command_pub = types.SimpleNamespace(get_num_connections=lambda: 1)
        probe.sim_time = 1.0
        probe.models = {'transport_object': {}}
        probe.swarm_task = {'status': 'idle'}
        probe.lock = PROBE.threading.RLock()
        probe.wait_for = lambda predicate, _timeout, description: self.assertTrue(
            predicate(), description
        )

        with self.assertRaisesRegex(RuntimeError, 'verified external viewer'):
            probe.ensure_idle_and_visible(False)

        probe.ensure_idle_and_visible(True)

    def test_single_robot_layout_is_centered_on_the_crate(self):
        self.assertEqual(
            [(-1.115, -1.6)],
            PROBE.push_layout(1, -0.8, -1.6),
        )

    def test_connection_report_requires_both_roots_and_rear_companions(self):
        robots = ['tb3_0', 'tb3_1', 'tb3_2', 'tb3_3']
        layout = dict(zip(robots, PROBE.push_layout(4, 0.0, 0.0)))

        connected = PROBE.push_connections(robots, layout, (0.0, 0.0))
        self.assertTrue(all(item['connected'] for item in connected.values()))
        self.assertEqual('tb3_0', connected['tb3_2']['parent'])
        self.assertEqual('tb3_1', connected['tb3_3']['parent'])

        layout['tb3_2'] = (-0.60, -0.09)
        broken = PROBE.push_connections(robots, layout, (0.0, 0.0))
        self.assertFalse(broken['tb3_2']['connected'])

    def test_model_xml_uses_spawn_pose_instead_of_embedded_pose(self):
        model_xml = PROBE.read_model_xml(
            PROBE.LOADED_MODEL,
            "./model[@name='transport_object']",
        )
        self.assertTrue(model_xml.startswith('<sdf version='))
        self.assertNotIn('<pose>', model_xml)
        self.assertIn('<mass>0.75</mass>', model_xml)

    def test_pose_retries_the_brief_post_spawn_service_race(self):
        class State:
            def __init__(self):
                self.pose = types.SimpleNamespace(
                    position=types.SimpleNamespace(),
                    orientation=types.SimpleNamespace(),
                )

        probe = object.__new__(PROBE.LoadProbe)
        probe.set_model_state = mock.Mock(side_effect=[
            types.SimpleNamespace(
                success=False,
                status_message='SetModelState: model does not exist',
            ),
            types.SimpleNamespace(success=True, status_message=''),
        ])

        with mock.patch.object(PROBE, 'ModelState', State), mock.patch.object(
            PROBE.time, 'sleep'
        ) as sleep:
            probe.set_pose('transport_object', -0.8, -3.0, 0.1)

        self.assertEqual(2, probe.set_model_state.call_count)
        sleep.assert_called_once_with(0.05)

    def test_pose_does_not_retry_a_permanent_service_rejection(self):
        class State:
            def __init__(self):
                self.pose = types.SimpleNamespace(
                    position=types.SimpleNamespace(),
                    orientation=types.SimpleNamespace(),
                )

        probe = object.__new__(PROBE.LoadProbe)
        probe.set_model_state = mock.Mock(return_value=types.SimpleNamespace(
            success=False,
            status_message='SetModelState: reference frame does not exist',
        ))

        with mock.patch.object(PROBE, 'ModelState', State), mock.patch.object(
            PROBE.time, 'sleep'
        ) as sleep, self.assertRaisesRegex(RuntimeError, 'reference frame'):
            probe.set_pose('transport_object', -0.8, -3.0, 0.1)

        self.assertEqual(1, probe.set_model_state.call_count)
        sleep.assert_not_called()

    def test_payload_readiness_requires_fresh_consecutive_observations(self):
        probe = object.__new__(PROBE.LoadProbe)
        probe.lock = PROBE.threading.RLock()
        probe.stop_requested = False
        probe.model_states_generation = 4
        probe.models = {
            'transport_object': {'position': (-0.8, -3.0, 0.1)},
        }

        def wait_for(predicate, _timeout, description):
            self.assertEqual('stable payload spawn', description)
            self.assertFalse(predicate(), 'the cached generation is stale')
            probe.model_states_generation = 5
            self.assertFalse(predicate(), 'one fresh sample is insufficient')
            probe.models = {}
            probe.model_states_generation = 6
            self.assertFalse(predicate(), 'an absence resets stability')
            probe.models = {
                'transport_object': {'position': (-0.8, -3.0, 0.1)},
            }
            probe.model_states_generation = 7
            self.assertFalse(predicate())
            probe.model_states_generation = 8
            self.assertTrue(predicate())

        probe.wait_for = wait_for

        generation = probe.wait_for_model_observations(
            'transport_object', True, 4, 2, 'stable payload spawn'
        )

        self.assertEqual(8, generation)

    def test_payload_readiness_does_not_reclassify_ros_shutdown(self):
        probe = object.__new__(PROBE.LoadProbe)
        probe.lock = PROBE.threading.RLock()
        probe.stop_requested = False
        probe.model_states_generation = 4
        probe.models = {}

        def wait_for(predicate, _timeout, _description):
            probe.model_states_generation = 5
            self.assertFalse(predicate())
            raise RuntimeError('probe stopped while waiting for readiness')

        probe.wait_for = wait_for

        with mock.patch.object(
            PROBE.rospy, 'is_shutdown', return_value=True, create=True
        ), self.assertRaisesRegex(RuntimeError, 'probe stopped') as raised:
            probe.wait_for_model_observations(
                'transport_object', True, 4, 2, 'stable payload spawn'
            )

        self.assertNotIsInstance(
            raised.exception, PROBE._RecoverablePayloadRace
        )

    def test_payload_replacement_retries_a_fresh_disappearance(self):
        class SpawnPose:
            def __init__(self):
                self.position = types.SimpleNamespace()
                self.orientation = types.SimpleNamespace()

        probe = object.__new__(PROBE.LoadProbe)
        probe.args = types.SimpleNamespace(crate_x=-0.8, crate_y=-3.0)
        probe.lock = PROBE.threading.RLock()
        probe.model_states_generation = 3
        probe.models = {'transport_object': {}}
        probe.delete_model = mock.Mock(return_value=types.SimpleNamespace(
            success=True, status_message='',
        ))
        probe.spawn_model = mock.Mock(return_value=types.SimpleNamespace(
            success=True, status_message='',
        ))
        probe.wait_for_model_observations = mock.Mock()
        probe._place_payload_pose = mock.Mock(side_effect=[
            PROBE._RecoverablePayloadRace('first instance disappeared'),
            None,
        ])
        probe.wait_sim = mock.Mock()
        probe.log = mock.Mock()

        with mock.patch.object(PROBE, 'Pose', SpawnPose):
            probe.replace_payload('loaded-model')

        self.assertEqual(2, probe.delete_model.call_count)
        self.assertEqual(2, probe.spawn_model.call_count)
        self.assertEqual(2, probe._place_payload_pose.call_count)
        probe.wait_sim.assert_called_once_with(0.5)
        self.assertEqual('loaded-model', probe.payload_model_xml)
        self.assertEqual(1, probe.log.call_count)

    def test_payload_replacement_exhaustion_is_sanitized_and_bounded(self):
        class SpawnPose:
            def __init__(self):
                self.position = types.SimpleNamespace()
                self.orientation = types.SimpleNamespace()

        probe = object.__new__(PROBE.LoadProbe)
        probe.args = types.SimpleNamespace(crate_x=-0.8, crate_y=-3.0)
        probe.lock = PROBE.threading.RLock()
        probe.model_states_generation = 3
        probe.models = {'transport_object': {}}
        probe.delete_model = mock.Mock(return_value=types.SimpleNamespace(
            success=True, status_message='',
        ))
        probe.spawn_model = mock.Mock(return_value=types.SimpleNamespace(
            success=True, status_message='',
        ))
        probe.wait_for_model_observations = mock.Mock()
        probe._place_payload_pose = mock.Mock(side_effect=
            PROBE._RecoverablePayloadRace('private Gazebo detail')
        )
        probe.wait_sim = mock.Mock()
        probe.log = mock.Mock()

        with mock.patch.object(PROBE, 'Pose', SpawnPose), self.assertRaisesRegex(
            RuntimeError, 'readiness race persisted after 3 replacement attempts'
        ) as raised:
            probe.replace_payload('loaded-model')

        self.assertNotIn('private Gazebo detail', str(raised.exception))
        self.assertEqual(3, probe.delete_model.call_count)
        self.assertEqual(3, probe.spawn_model.call_count)
        probe.wait_sim.assert_not_called()

    def test_payload_replacement_does_not_hide_a_permanent_pose_error(self):
        class SpawnPose:
            def __init__(self):
                self.position = types.SimpleNamespace()
                self.orientation = types.SimpleNamespace()

        probe = object.__new__(PROBE.LoadProbe)
        probe.args = types.SimpleNamespace(crate_x=-0.8, crate_y=-3.0)
        probe.lock = PROBE.threading.RLock()
        probe.model_states_generation = 3
        probe.models = {'transport_object': {}}
        probe.delete_model = mock.Mock(return_value=types.SimpleNamespace(
            success=True, status_message='',
        ))
        probe.spawn_model = mock.Mock(return_value=types.SimpleNamespace(
            success=True, status_message='',
        ))
        probe.wait_for_model_observations = mock.Mock()
        probe._place_payload_pose = mock.Mock(side_effect=RuntimeError(
            'could not place transport_object: reference frame rejected'
        ))
        probe.wait_sim = mock.Mock()
        probe.log = mock.Mock()

        with mock.patch.object(PROBE, 'Pose', SpawnPose), self.assertRaisesRegex(
            RuntimeError, 'reference frame rejected'
        ):
            probe.replace_payload('loaded-model')

        self.assertEqual(1, probe.delete_model.call_count)
        self.assertEqual(1, probe.spawn_model.call_count)
        probe.log.assert_not_called()

    def test_payload_reset_reinstalls_the_selected_profile_after_loss(self):
        probe = object.__new__(PROBE.LoadProbe)
        probe.payload_model_xml = 'loaded-model'
        probe._place_payload_pose = mock.Mock(side_effect=
            PROBE._RecoverablePayloadRace('instance disappeared')
        )
        probe.replace_payload = mock.Mock()
        probe.log = mock.Mock()

        probe.reset_payload_pose()

        probe.replace_payload.assert_called_once_with('loaded-model')
        probe.log.assert_called_once()

    def test_trial_duration_excludes_the_safe_stop_publications(self):
        class Command:
            def __init__(self):
                self.linear = types.SimpleNamespace(x=0.0)

        class Publisher:
            instances = []

            def __init__(self, *_args, **_kwargs):
                self.speeds = []
                self.unregistered = False
                self.__class__.instances.append(self)

            def get_num_connections(self):
                return 1

            def publish(self, command):
                self.speeds.append(command.linear.x)

            def unregister(self):
                self.unregistered = True

        probe = object.__new__(PROBE.LoadProbe)
        probe.args = types.SimpleNamespace(
            command_speed=0.16,
            push_duration=12.0,
        )
        probe.lock = PROBE.threading.RLock()
        probe.stop_requested = False
        probe.sim_time = 100.0
        probe.models = {
            'transport_object': {'position': (0.0, 0.0, 0.1)},
            'tb3_0': {'position': (-0.315, 0.0, 0.0)},
        }
        probe.log = lambda _message: None
        setup_events = []
        probe.reset_fleet = lambda count: setup_events.append(
            ('reset_fleet', count)
        )
        probe.replace_payload = lambda payload: setup_events.append(
            ('replace_payload', payload)
        )
        probe.arrange_pushers = lambda: (
            setup_events.append(('arrange_pushers', None)) or ['tb3_0']
        )
        probe.wait_for = lambda predicate, _timeout, description: (
            self.assertTrue(predicate(), description)
        )
        wall_time = [0.0]

        def sleep(_seconds):
            speed = Publisher.instances[0].speeds[-1]
            if speed > 0.0:
                probe.sim_time += 6.0
                wall_time[0] += 2.0
            else:
                # Eight zero-command cycles deliberately add 0.5 simulated
                # seconds. They prove that the returned push duration was
                # captured before the safe stop sequence.
                probe.sim_time += 0.0625
                wall_time[0] += 0.02

        with mock.patch.object(PROBE, 'Twist', Command), mock.patch.object(
            PROBE.rospy, 'Publisher', Publisher, create=True
        ), mock.patch.object(
            PROBE.rospy, 'is_shutdown', return_value=False, create=True
        ), mock.patch.object(
            PROBE.time, 'monotonic', side_effect=lambda: wall_time[0]
        ), mock.patch.object(PROBE.time, 'sleep', side_effect=sleep):
            result = probe.run_trial(1, 'loaded-model')

        self.assertEqual(12.0, result['push_duration_sim_s'])
        self.assertEqual(4.0, result['push_duration_wall_s'])
        self.assertEqual(3.0, result['real_time_factor'])
        self.assertEqual(112.5, probe.sim_time)
        self.assertEqual(8, Publisher.instances[0].speeds.count(0.0))
        self.assertTrue(Publisher.instances[0].unregistered)
        self.assertEqual(
            [
                ('reset_fleet', 1),
                ('replace_payload', 'loaded-model'),
                ('arrange_pushers', None),
            ],
            setup_events,
        )

    def test_capacity_gate_accepts_resistant_single_and_moving_fleet(self):
        single = self.trial(1, 0.02, {'tb3_0': 0.01})
        roots = self.trial(2, 0.03, {'tb3_0': 0.03, 'tb3_1': 0.03})
        fleet = self.trial(4, 0.24, {
            'tb3_1': 0.25, 'tb3_2': 0.25,
            'tb3_3': 0.24, 'tb3_4': 0.24,
        })

        failures, gain = PROBE.evaluate(single, roots, fleet, self.args())

        self.assertEqual([], failures)
        self.assertEqual(12.0, gain)

    def test_capacity_gate_rejects_easy_crate_and_idle_companion(self):
        single = self.trial(1, 0.10, {'tb3_0': 0.10})
        roots = self.trial(2, 0.10, {'tb3_0': 0.10, 'tb3_1': 0.10})
        fleet = self.trial(4, 0.21, {
            'tb3_1': 0.22, 'tb3_2': 0.22,
            'tb3_3': 0.00, 'tb3_4': 0.22,
        }, rtf=2.0, connected_count=3)

        failures, _gain = PROBE.evaluate(
            single, roots, fleet, self.args()
        )

        self.assertTrue(any('one robot moved' in item for item in failures))
        self.assertTrue(any('two payload roots' in item for item in failures))
        self.assertTrue(any('did not all advance' in item for item in failures))
        self.assertTrue(any('not connected' in item for item in failures))
        self.assertTrue(any('real-time factor' in item for item in failures))

    def test_loaded_grf_command_reuses_the_full_n4_acceptance(self):
        args = self.args()
        args.min_rtf = 2.90

        command = PROBE.loaded_grf_command(args, 'loaded-case-4')

        self.assertTrue(command[1].endswith('robotswarm_live_acceptance.py'))
        self.assertIn('transport_grf_n4', command)
        self.assertEqual('2.9', command[command.index('--min-rtf') + 1])
        self.assertEqual(
            'loaded-case-4', command[command.index('--task-id') + 1]
        )
        self.assertIn('--delete-after', command)

    def test_delete_result_is_indexed_by_its_request_id(self):
        probe = object.__new__(PROBE.LoadProbe)
        probe.lock = PROBE.threading.RLock()
        probe.delete_results = {}

        probe._delete_result_cb(types.SimpleNamespace(data=PROBE.json.dumps({
            'request_id': 'cleanup-42',
            'success': True,
        })))

        self.assertTrue(probe.delete_results['cleanup-42']['success'])

    def test_cleanup_accepts_empty_state_published_before_delete_result(self):
        probe = object.__new__(PROBE.LoadProbe)
        probe.lock = PROBE.threading.RLock()
        probe.stop_requested = True
        probe.swarm_task = {'task_id': 'task-loaded-4', 'status': 'running'}
        probe.roster = ['tb3_0']
        probe.roster_generation = 1
        probe.models = {'tb3_0': {}}
        probe.model_states_generation = 1
        probe.cleanup_task_id = 'task-loaded-4'
        probe.delete_results = {}
        events = []

        def send(command, parameters=None):
            events.append((command, parameters))
            if command == 'stop_task':
                probe.swarm_task['status'] = 'stopped'
            elif command == 'delete_robots':
                probe.roster = []
                probe.roster_generation += 1
                probe.models = {}
                probe.model_states_generation += 1

        def wait_for(predicate, _timeout, description):
            if description == 'correlated fleet delete result':
                request_id = events[-1][1]['request_id']
                probe.delete_results[request_id] = {
                    'request_id': request_id,
                    'remaining_robot_ids': [],
                }
            self.assertTrue(predicate(), description)

        probe.send = send
        probe.wait_for = wait_for
        probe.replace_payload = lambda model: events.append(
            ('replace_payload', model)
        )

        probe.cleanup('practice-model')

        self.assertEqual(
            ('stop_task', {'task_id': 'task-loaded-4'}), events[0]
        )
        self.assertEqual('delete_robots', events[1][0])
        self.assertTrue(
            events[1][1]['request_id'].startswith('load-cleanup-')
        )
        self.assertEqual(('replace_payload', 'practice-model'), events[2])
        self.assertTrue(probe.stop_requested)

    def test_cleanup_refuses_to_delete_or_restore_if_task_will_not_stop(self):
        probe = object.__new__(PROBE.LoadProbe)
        probe.lock = PROBE.threading.RLock()
        probe.stop_requested = True
        probe.swarm_task = {'task_id': 'task-stuck', 'status': 'running'}
        probe.roster = ['tb3_0']
        probe.roster_generation = 1
        probe.models = {'tb3_0': {}}
        probe.model_states_generation = 1
        probe.cleanup_task_id = 'task-stuck'
        probe.delete_results = {}
        events = []
        probe.send = lambda command, parameters=None: events.append(
            (command, parameters)
        )

        def timeout(_predicate, _seconds, _description):
            raise RuntimeError('timeout waiting for task stop')

        probe.wait_for = timeout
        probe.replace_payload = lambda model: events.append(
            ('replace_payload', model)
        )

        with self.assertRaisesRegex(RuntimeError, 'task stop'):
            probe.cleanup('practice-model')

        self.assertEqual(
            [('stop_task', {'task_id': 'task-stuck'})], events
        )
        self.assertTrue(probe.stop_requested)

    def test_cleanup_does_not_restore_payload_if_fleet_delete_times_out(self):
        probe = object.__new__(PROBE.LoadProbe)
        probe.lock = PROBE.threading.RLock()
        probe.stop_requested = True
        probe.swarm_task = {'task_id': 'task-finished', 'status': 'completed'}
        probe.roster = ['tb3_0']
        probe.roster_generation = 1
        probe.models = {'tb3_0': {}}
        probe.model_states_generation = 1
        probe.delete_results = {}
        events = []
        probe.send = lambda command, parameters=None: events.append(
            (command, parameters)
        )

        def wait_for(_predicate, _seconds, description):
            raise RuntimeError('timeout waiting for ' + description)

        probe.wait_for = wait_for
        probe.replace_payload = lambda model: events.append(
            ('replace_payload', model)
        )

        with self.assertRaisesRegex(RuntimeError, 'fleet delete'):
            probe.cleanup('practice-model')

        self.assertEqual('delete_robots', events[0][0])
        self.assertTrue(
            events[0][1]['request_id'].startswith('load-cleanup-')
        )
        self.assertTrue(probe.stop_requested)

    def test_cleanup_does_not_restore_payload_with_a_residual_robot_model(self):
        probe = object.__new__(PROBE.LoadProbe)
        probe.lock = PROBE.threading.RLock()
        probe.stop_requested = True
        probe.swarm_task = {'task_id': 'task-finished', 'status': 'completed'}
        probe.roster = ['tb3_0']
        probe.roster_generation = 1
        probe.models = {'tb3_0': {}, 'transport_object': {}}
        probe.model_states_generation = 1
        probe.delete_results = {}
        events = []

        def send(command, parameters=None):
            events.append((command, parameters))
            if command == 'delete_robots':
                probe.roster = []
                probe.roster_generation += 1
                probe.model_states_generation += 1

        def wait_for(predicate, _seconds, description):
            if description == 'correlated fleet delete result':
                request_id = events[-1][1]['request_id']
                probe.delete_results[request_id] = {
                    'request_id': request_id,
                    'remaining_robot_ids': [],
                }
            elif description == 'post-command cleanup Gazebo model deletion':
                self.assertFalse(predicate())
                raise RuntimeError('residual Gazebo robot model')
            self.assertTrue(predicate(), description)

        probe.send = send
        probe.wait_for = wait_for
        probe.replace_payload = lambda model: events.append(
            ('replace_payload', model)
        )

        with self.assertRaisesRegex(RuntimeError, 'residual Gazebo'):
            probe.cleanup('practice-model')

        self.assertEqual('delete_robots', events[0][0])
        self.assertTrue(probe.stop_requested)

    def test_cleanup_does_not_accept_empty_caches_from_before_delete(self):
        probe = object.__new__(PROBE.LoadProbe)
        probe.lock = PROBE.threading.RLock()
        probe.stop_requested = True
        probe.swarm_task = {'task_id': 'task-finished', 'status': 'completed'}
        probe.roster = []
        probe.roster_generation = 4
        probe.models = {'transport_object': {}}
        probe.model_states_generation = 7
        probe.delete_results = {}
        events = []
        probe.send = lambda command, parameters=None: events.append(
            (command, parameters)
        )

        def wait_for(predicate, _seconds, description):
            if description == 'correlated fleet delete result':
                request_id = events[-1][1]['request_id']
                probe.delete_results[request_id] = {
                    'request_id': request_id,
                    'remaining_robot_ids': [],
                }
                self.assertTrue(predicate())
                return
            self.assertEqual(
                'post-command cleanup fleet deletion', description
            )
            self.assertFalse(predicate())
            raise RuntimeError('no post-command roster observation')

        probe.wait_for = wait_for
        probe.replace_payload = lambda model: events.append(
            ('replace_payload', model)
        )

        with self.assertRaisesRegex(RuntimeError, 'post-command roster'):
            probe.cleanup('practice-model')

        self.assertEqual('delete_robots', events[0][0])
        self.assertTrue(probe.stop_requested)

    def test_cleanup_rejects_an_authoritative_nonempty_delete_result(self):
        probe = object.__new__(PROBE.LoadProbe)
        probe.lock = PROBE.threading.RLock()
        probe.stop_requested = True
        probe.swarm_task = {'task_id': 'task-finished', 'status': 'completed'}
        probe.roster = []
        probe.roster_generation = 2
        probe.models = {'transport_object': {}}
        probe.model_states_generation = 2
        probe.delete_results = {}
        events = []

        def send(command, parameters=None):
            events.append((command, parameters))
            if command == 'delete_robots':
                probe.roster_generation += 1
                probe.model_states_generation += 1

        def wait_for(predicate, _seconds, description):
            self.assertEqual('correlated fleet delete result', description)
            request_id = events[-1][1]['request_id']
            probe.delete_results[request_id] = {
                'request_id': request_id,
                'remaining_robot_ids': ['tb3_0'],
            }
            self.assertTrue(predicate())

        probe.send = send
        probe.wait_for = wait_for
        probe.replace_payload = lambda model: events.append(
            ('replace_payload', model)
        )

        with self.assertRaisesRegex(RuntimeError, 'left active robots'):
            probe.cleanup('practice-model')

        self.assertEqual('delete_robots', events[0][0])
        self.assertEqual(1, len(events))

    def test_cleanup_refuses_status_not_refreshed_after_child_exit(self):
        probe = object.__new__(PROBE.LoadProbe)
        probe.lock = PROBE.threading.RLock()
        probe.stop_requested = True
        probe.task_status_fresh_for_cleanup = False
        probe.cleanup_task_id = 'child-task'
        probe.swarm_task = {'task_id': 'stale-task', 'status': 'completed'}
        probe.roster = []
        probe.roster_generation = 1
        probe.models = {}
        probe.model_states_generation = 1
        events = []
        probe.send = lambda command, parameters=None: events.append(
            (command, parameters)
        )
        probe.replace_payload = lambda model: events.append(
            ('replace_payload', model)
        )

        with self.assertRaisesRegex(RuntimeError, 'no fresh swarm task status'):
            probe.cleanup('practice-model')

        self.assertEqual([], events)
        self.assertTrue(probe.stop_requested)

    def test_loaded_grf_rejects_a_fresh_but_unrelated_task_status(self):
        process = types.SimpleNamespace(returncode=1, poll=lambda: 1)
        probe = types.SimpleNamespace(
            stop_requested=False,
            log=lambda _message: None,
            lock=PROBE.threading.RLock(),
            swarm_status_generation=2,
            swarm_task={'task_id': 'previous-task', 'status': 'completed'},
            task_status_fresh_for_cleanup=True,
            cleanup_task_id=None,
        )

        def reject_unrelated(
            predicate, _timeout, description, continue_after_stop=False,
        ):
            self.assertIn('fresh status for child task', description)
            self.assertTrue(continue_after_stop)
            probe.swarm_status_generation += 1
            self.assertFalse(predicate())
            raise RuntimeError('child task status was not observed')

        probe.wait_for = reject_unrelated
        args = types.SimpleNamespace(min_rtf=2.90)

        with mock.patch.object(
            PROBE.subprocess, 'Popen', return_value=process
        ), self.assertRaisesRegex(RuntimeError, 'not observed'):
            PROBE.run_loaded_grf(probe, args)

        self.assertTrue(probe.cleanup_task_id.startswith('loaded-grf-n4-'))
        self.assertFalse(probe.task_status_fresh_for_cleanup)

    def test_loaded_grf_escalates_an_unresponsive_child_to_sigkill(self):
        class StuckProcess:
            def __init__(self):
                self.returncode = None
                self.signals = []
                self.terminate_calls = 0
                self.kill_calls = 0

            def poll(self):
                return self.returncode

            def send_signal(self, signum):
                self.signals.append(signum)

            def terminate(self):
                self.terminate_calls += 1

            def kill(self):
                self.kill_calls += 1
                self.returncode = -PROBE.signal.SIGKILL

        process = StuckProcess()
        probe = types.SimpleNamespace(
            stop_requested=True,
            log=lambda _message: None,
            lock=PROBE.threading.RLock(),
            swarm_status_generation=4,
            swarm_task={},
            task_status_fresh_for_cleanup=True,
            cleanup_task_id=None,
        )

        def wait_for(
            predicate, _timeout, description, continue_after_stop=False,
        ):
            self.assertIn('fresh status for child task', description)
            self.assertTrue(continue_after_stop)
            probe.swarm_status_generation += 1
            probe.swarm_task = {
                'task_id': probe.cleanup_task_id,
                'status': 'running',
            }
            self.assertTrue(predicate())

        probe.wait_for = wait_for
        args = types.SimpleNamespace(min_rtf=2.90)
        clock = [0.0, 21.0, 21.0, 32.0]

        with mock.patch.object(
            PROBE.subprocess, 'Popen', return_value=process
        ), mock.patch.object(
            PROBE.time, 'monotonic', side_effect=clock
        ), mock.patch.object(PROBE.time, 'sleep'):
            result = PROBE.run_loaded_grf(probe, args)

        self.assertEqual([PROBE.signal.SIGINT], process.signals)
        self.assertEqual(1, process.terminate_calls)
        self.assertEqual(1, process.kill_calls)
        self.assertEqual(-PROBE.signal.SIGKILL, result['exit_code'])
        self.assertFalse(result['passed'])
        self.assertTrue(result['fresh_cleanup_status_observed'])
        self.assertEqual(probe.cleanup_task_id, result['task_id'])
        self.assertTrue(probe.task_status_fresh_for_cleanup)


if __name__ == '__main__':
    unittest.main()
