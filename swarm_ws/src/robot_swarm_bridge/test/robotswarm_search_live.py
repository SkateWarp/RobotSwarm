#!/usr/bin/env python3
"""Observe one live collaborative-search run and check the hand-off.

The script intentionally does not spawn robots or start tasks.  Start it just
before publishing the transport task so it can hear the one-shot discovery
topic::

    python3 "$(rospack find robot_swarm_bridge)/test/robotswarm_search_live.py" \
        --task-id search-live-n10 --robot-count 10

A run passes only when every robot keeps receiving search commands and covers
real ground.  After discovery, the probe listens briefly for each notified
teammate to acknowledge the event or start travelling toward the payload.
"""

import argparse
import json
import math
import sys
import threading
import time

import rospy
from std_msgs.msg import String


class SearchProbe:
    MOTION_COMMAND_EPSILON = 0.005
    MIN_SEARCH_SAMPLES = 5
    MIN_MOVING_SAMPLES = 3
    MIN_MOVING_RATIO = 0.35
    MIN_SEARCH_PATH_M = 0.03
    MIN_RESPONSE_SAMPLES = 2
    MIN_RESPONSE_PROGRESS_M = 0.02

    def __init__(self, task_id, robot_count, approach_window=4.0):
        self.task_id = task_id
        self.robot_count = robot_count
        self.approach_window = max(0.5, float(approach_window))
        self.lock = threading.Lock()
        self.search_samples = 0
        self.search_observed_samples = {}
        self.moving_samples = {}
        self.initial_positions = {}
        self.latest_positions = {}
        self.search_path_length = {}
        self._search_previous_positions = {}
        self.notices = []
        self.approach_status = None
        self.approach_samples = 0
        self.approach_started_at = None
        self.approach_last_seen_at = None
        self.approach_observed_samples = {}
        self.approach_correlated_samples = {}
        self.approach_moving_samples = {}
        self.approach_initial_distances = {}
        self.approach_latest_distances = {}
        self.approach_path_length = {}
        self._approach_previous_positions = {}
        self.last_status = None
        self.collision_start = None
        self.collision_end = None

        rospy.Subscriber(
            '/transport/status', String, self._transport_status, queue_size=20
        )
        rospy.Subscriber(
            '/transport/discovery', String, self._discovery, queue_size=10
        )
        rospy.Subscriber(
            '/swarm/status', String, self._swarm_status, queue_size=10
        )

    @staticmethod
    def _decode(message):
        try:
            value = json.loads(message.data)
        except (AttributeError, TypeError, json.JSONDecodeError):
            return None
        return value if isinstance(value, dict) else None

    @staticmethod
    def _position(assignment):
        try:
            return float(assignment['x']), float(assignment['y'])
        except (KeyError, TypeError, ValueError):
            return None

    @classmethod
    def _command_is_moving(cls, assignment):
        command = assignment.get('command', {})
        try:
            command_size = (
                abs(float(command.get('linear', 0.0)))
                + abs(float(command.get('angular', 0.0)))
            )
        except (AttributeError, TypeError, ValueError):
            return False
        return command_size > cls.MOTION_COMMAND_EPSILON

    @staticmethod
    def _add_path_step(lengths, previous, namespace, position):
        old_position = previous.get(namespace)
        previous[namespace] = position
        if old_position is None:
            return

        step = math.hypot(
            position[0] - old_position[0],
            position[1] - old_position[1],
        )
        # A model reset or respawn should not count as useful travel.
        if step <= 0.5:
            lengths[namespace] = lengths.get(namespace, 0.0) + step

    @staticmethod
    def _object_position(status):
        discovery = status.get('discovery') or {}
        position = discovery.get('object_position')
        if not isinstance(position, dict):
            position = status.get('object_pos')
        if not isinstance(position, dict):
            return None
        try:
            return float(position['x']), float(position['y'])
        except (KeyError, TypeError, ValueError):
            return None

    def _transport_status(self, message):
        status = self._decode(message)
        if status is None or status.get('task_id') != self.task_id:
            return

        assignments = status.get('robot_assignments', {})
        if not isinstance(assignments, dict):
            assignments = {}

        with self.lock:
            self.last_status = status
            phase = status.get('phase')
            if phase == 'SEARCH':
                self.search_samples += 1
                for namespace, assignment in assignments.items():
                    if not isinstance(assignment, dict):
                        continue
                    position = self._position(assignment)
                    if position is None:
                        continue
                    self.initial_positions.setdefault(namespace, position)
                    self.latest_positions[namespace] = position
                    self.search_observed_samples[namespace] = (
                        self.search_observed_samples.get(namespace, 0) + 1
                    )
                    self._add_path_step(
                        self.search_path_length,
                        self._search_previous_positions,
                        namespace,
                        position,
                    )

                    if self._command_is_moving(assignment):
                        self.moving_samples[namespace] = (
                            self.moving_samples.get(namespace, 0) + 1
                        )
            elif phase == 'APPROACH' and status.get('discovery'):
                self.approach_status = status
                self.approach_samples += 1
                observed_at = time.monotonic()
                if self.approach_started_at is None:
                    self.approach_started_at = observed_at
                self.approach_last_seen_at = observed_at
                self._record_approach(assignments, status)

    def _record_approach(self, assignments, status):
        discovery = status.get('discovery') or {}
        finder = discovery.get('finder')
        notified = set(discovery.get('notified_robots') or ())
        object_position = self._object_position(status)

        for namespace, assignment in assignments.items():
            if namespace == finder or not isinstance(assignment, dict):
                continue

            position = self._position(assignment)
            if position is not None:
                self.approach_observed_samples[namespace] = (
                    self.approach_observed_samples.get(namespace, 0) + 1
                )
                self._add_path_step(
                    self.approach_path_length,
                    self._approach_previous_positions,
                    namespace,
                    position,
                )
                if object_position is not None:
                    distance = math.hypot(
                        position[0] - object_position[0],
                        position[1] - object_position[1],
                    )
                    self.approach_initial_distances.setdefault(
                        namespace, distance
                    )
                    self.approach_latest_distances[namespace] = distance

            if self._command_is_moving(assignment):
                self.approach_moving_samples[namespace] = (
                    self.approach_moving_samples.get(namespace, 0) + 1
                )

            is_correlated = (
                namespace in notified
                and assignment.get('notice_received') is True
                and assignment.get('activity') == 'responding_to_discovery'
            )
            if is_correlated:
                self.approach_correlated_samples[namespace] = (
                    self.approach_correlated_samples.get(namespace, 0) + 1
                )

    def _discovery(self, message):
        notice = self._decode(message)
        if notice is None or notice.get('task_id') != self.task_id:
            return
        with self.lock:
            self.notices.append(notice)

    def _swarm_status(self, message):
        status = self._decode(message)
        if status is None:
            return
        try:
            collisions = int(status.get('collisions', 0) or 0)
        except (TypeError, ValueError):
            return
        with self.lock:
            if self.collision_start is None:
                self.collision_start = collisions
            self.collision_end = collisions

    def _responder_evidence(self, discovery):
        finder = discovery.get('finder') if discovery else None
        responders = [
            namespace
            for namespace in (discovery or {}).get('notified_robots', ())
            if namespace != finder
        ]
        evidence = {}
        for namespace in responders:
            initial_distance = self.approach_initial_distances.get(namespace)
            latest_distance = self.approach_latest_distances.get(namespace)
            progress = 0.0
            if initial_distance is not None and latest_distance is not None:
                progress = initial_distance - latest_distance

            correlated = self.approach_correlated_samples.get(namespace, 0)
            moving = self.approach_moving_samples.get(namespace, 0)
            path_length = self.approach_path_length.get(namespace, 0.0)
            evidence[namespace] = {
                'responded': (
                    progress >= self.MIN_RESPONSE_PROGRESS_M
                    or path_length >= self.MIN_RESPONSE_PROGRESS_M
                    or moving >= self.MIN_RESPONSE_SAMPLES
                ),
                'correlated_status_samples': correlated,
                'moving_command_samples': moving,
                'path_length_m': path_length,
                'progress_toward_payload_m': progress,
            }
        return evidence

    def complete(self):
        with self.lock:
            if not self.notices or not self.approach_status:
                return False

            discovery = self.approach_status.get('discovery') or self.notices[0]
            evidence = self._responder_evidence(discovery)
            all_responded = all(
                item['responded'] for item in evidence.values()
            )
            elapsed = time.monotonic() - self.approach_started_at
            minimum_observation = min(1.0, self.approach_window)
            if all_responded and elapsed >= minimum_observation:
                return True
            return elapsed >= self.approach_window

    def report(self):
        with self.lock:
            reported_at = time.monotonic()
            names = sorted(
                set(self.initial_positions)
                | set(self.latest_positions)
                | set(self.moving_samples)
            )
            travel = {}
            for namespace in names:
                start = self.initial_positions.get(namespace)
                end = self.latest_positions.get(namespace)
                if start is None or end is None:
                    travel[namespace] = 0.0
                else:
                    travel[namespace] = math.hypot(
                        end[0] - start[0], end[1] - start[1]
                    )

            notices = list(self.notices)
            approach = self.approach_status or {}
            assignments = approach.get('robot_assignments', {})
            notice_received = [
                namespace for namespace, assignment in assignments.items()
                if isinstance(assignment, dict)
                and assignment.get('notice_received')
            ]
            discovery = approach.get('discovery') or (
                notices[0] if notices else None
            )
            collision_delta = max(
                0,
                (self.collision_end or 0) - (self.collision_start or 0),
            )
            approach_observation_seconds = max(
                0.0,
                reported_at - (self.approach_started_at or reported_at),
            )

            failures = []
            if self.search_samples < self.MIN_SEARCH_SAMPLES:
                failures.append(
                    'only {} SEARCH status samples; expected at least {}'.format(
                        self.search_samples, self.MIN_SEARCH_SAMPLES
                    )
                )
            if len(names) != self.robot_count:
                failures.append(
                    'observed {} of {} robots'.format(
                        len(names), self.robot_count
                    )
                )
            under_observed = [
                name for name in names
                if self.search_observed_samples.get(name, 0)
                < self.MIN_SEARCH_SAMPLES
            ]
            if under_observed:
                failures.append(
                    'robots were not observed throughout SEARCH: '
                    + ', '.join(under_observed)
                )
            moving_ratios = {}
            weak_search_commands = []
            for name in names:
                observed = self.search_observed_samples.get(name, 0)
                moving = self.moving_samples.get(name, 0)
                ratio = moving / observed if observed else 0.0
                moving_ratios[name] = ratio
                if (
                    moving < self.MIN_MOVING_SAMPLES
                    or ratio < self.MIN_MOVING_RATIO
                ):
                    weak_search_commands.append(name)
            if weak_search_commands:
                failures.append(
                    'robots lacked sustained search commands: '
                    + ', '.join(weak_search_commands)
                )
            short_search_paths = [
                name for name in names
                if self.search_path_length.get(name, 0.0)
                < self.MIN_SEARCH_PATH_M
            ]
            if short_search_paths:
                failures.append(
                    'robots covered less than 3cm while searching: '
                    + ', '.join(short_search_paths)
                )
            if len(notices) != 1:
                failures.append(
                    'expected one discovery notice, got {}'.format(
                        len(notices)
                    )
                )
            if not approach:
                failures.append('fleet never entered APPROACH after discovery')
            elif approach_observation_seconds < min(
                1.0, self.approach_window
            ):
                failures.append('APPROACH response window ended too soon')
            if len(notice_received) != self.robot_count:
                failures.append(
                    '{} of {} robots acknowledged the notice'.format(
                        len(notice_received), self.robot_count
                    )
                )
            if discovery is not None:
                notified = discovery.get('notified_robots', [])
                if len(notified) != max(0, self.robot_count - 1):
                    failures.append(
                        'finder notified {} teammates, expected {}'.format(
                            len(notified), max(0, self.robot_count - 1)
                        )
                    )
                finder = discovery.get('finder')
                expected_responders = set(names) - {finder}
                if set(notified) != expected_responders:
                    failures.append(
                        'notice recipients did not match the observed fleet'
                    )
            responder_evidence = self._responder_evidence(discovery or {})
            missing_responses = [
                name for name, item in responder_evidence.items()
                if not item['responded']
            ]
            if missing_responses:
                failures.append(
                    'robots did not respond to the payload notice during '
                    'APPROACH: ' + ', '.join(missing_responses)
                )
            if collision_delta:
                failures.append(
                    '{} collision episodes occurred during search hand-off'.format(
                        collision_delta
                    )
                )

            return {
                'passed': not failures,
                'task_id': self.task_id,
                'robot_count': self.robot_count,
                'search_samples': self.search_samples,
                'search_observed_sample_counts': self.search_observed_samples,
                'moving_sample_counts': self.moving_samples,
                'moving_sample_ratios': {
                    name: round(ratio, 3)
                    for name, ratio in moving_ratios.items()
                },
                'search_displacement_m': {
                    name: round(distance, 4)
                    for name, distance in travel.items()
                },
                'search_path_length_m': {
                    name: round(
                        self.search_path_length.get(name, 0.0), 4
                    )
                    for name in names
                },
                'discovery': discovery,
                'notice_count': len(notices),
                'notice_received_count': len(notice_received),
                'approach_status_samples': self.approach_samples,
                'approach_observation_seconds': round(
                    approach_observation_seconds, 3
                ),
                'approach_status_span_seconds': round(
                    max(
                        0.0,
                        (self.approach_last_seen_at or 0.0)
                        - (self.approach_started_at or 0.0),
                    ),
                    3,
                ),
                'approach_observed_sample_counts': (
                    self.approach_observed_samples
                ),
                'responder_evidence': {
                    name: {
                        'responded': item['responded'],
                        'correlated_status_samples': (
                            item['correlated_status_samples']
                        ),
                        'moving_command_samples': (
                            item['moving_command_samples']
                        ),
                        'path_length_m': round(item['path_length_m'], 4),
                        'progress_toward_payload_m': round(
                            item['progress_toward_payload_m'], 4
                        ),
                    }
                    for name, item in responder_evidence.items()
                },
                'collision_count_delta': collision_delta,
                'failures': failures,
            }


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--task-id', required=True)
    parser.add_argument('--robot-count', type=int, required=True)
    parser.add_argument('--timeout', type=float, default=45.0)
    parser.add_argument(
        '--approach-window',
        type=float,
        default=4.0,
        help='seconds to observe responders after discovery (default: 4)',
    )
    return parser.parse_args()


def main():
    args = parse_args()
    if args.robot_count < 1:
        raise SystemExit('--robot-count must be positive')

    rospy.init_node('robotswarm_search_live', anonymous=True)
    probe = SearchProbe(
        args.task_id,
        args.robot_count,
        approach_window=args.approach_window,
    )
    deadline = time.monotonic() + max(1.0, args.timeout)
    while not rospy.is_shutdown() and time.monotonic() < deadline:
        if probe.complete():
            break
        time.sleep(0.05)

    result = probe.report()
    print('RESULT_JSON ' + json.dumps(result, sort_keys=True))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    sys.exit(main())
