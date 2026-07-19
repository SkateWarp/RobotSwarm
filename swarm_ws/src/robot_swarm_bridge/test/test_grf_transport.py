"""Unit tests for the ROS-independent cooperative-transport GRF kernel."""

import math
import sys
import unittest
from pathlib import Path


PACKAGE_SOURCE = Path(__file__).resolve().parents[1] / 'src'
sys.path.insert(0, str(PACKAGE_SOURCE))

from robot_swarm_bridge.algorithms import (  # noqa: E402
    GRFConfig,
    GibbsRandomFieldTransport,
    InteractionMode,
    ObjectContour,
    PotentialParameters,
    RobotSnapshot,
    Segment2,
    TransportSnapshot,
    Vec2,
    build_local_neighbor_graph,
    coulomb_buckingham,
    target_is_occluded,
)
from robot_swarm_bridge.algorithms.grf_adapter import (  # noqa: E402
    build_transport_snapshot,
    centered_object_contour,
    contact_center_distance,
    mcmc_iterations_for_fleet,
    occupied_lidar_sectors,
    oriented_box_contour,
)


def square_contour(half_extent=0.2):
    return ObjectContour(
        points=(
            Vec2(-half_extent, -half_extent),
            Vec2(half_extent, -half_extent),
            Vec2(half_extent, half_extent),
            Vec2(-half_extent, half_extent),
        )
    )


def disabled_potential():
    return PotentialParameters(
        epsilon=0.0,
        r0=1.0,
        alpha=1.0,
        q1=0.0,
        q2=0.0,
        epsilon0=1.0,
    )


class CoulombBuckinghamTests(unittest.TestCase):
    def test_equation_matches_paper_at_equilibrium_distance(self):
        parameters = PotentialParameters(
            epsilon=0.4,
            r0=0.8,
            alpha=0.2,
            q1=2.0,
            q2=-3.0,
            epsilon0=0.7,
        )
        expected = (
            -parameters.epsilon
            + parameters.q1
            * parameters.q2
            / (4.0 * math.pi * parameters.epsilon0 * parameters.r0)
        )
        actual = coulomb_buckingham(parameters.r0, parameters)
        self.assertAlmostEqual(expected, actual, places=12)

    def test_zero_and_nonfinite_distances_return_finite_bounded_energy(self):
        parameters = PotentialParameters(
            epsilon=0.04,
            r0=0.8,
            alpha=0.2,
            q1=1.0,
            q2=1.0,
            epsilon0=0.04,
        )
        for distance in (0.0, float('nan'), float('inf')):
            energy = coulomb_buckingham(
                distance, parameters, energy_limit=1234.0
            )
            self.assertTrue(math.isfinite(energy))
            self.assertLessEqual(abs(energy), 1234.0)


class NeighborGraphTests(unittest.TestCase):
    def test_filters_by_range_and_static_line_of_sight(self):
        robots = (
            RobotSnapshot('a', Vec2(0.0, 0.0)),
            RobotSnapshot('b', Vec2(0.8, 0.0)),
            RobotSnapshot('c', Vec2(0.0, 0.8)),
            RobotSnapshot('far', Vec2(3.0, 0.0)),
        )
        wall = Segment2(Vec2(0.4, -0.2), Vec2(0.4, 0.2))
        graph = build_local_neighbor_graph(
            robots, sensing_radius=1.0, occluding_segments=(wall,)
        )
        self.assertEqual(('c',), tuple(item.robot_id for item in graph['a']))
        self.assertNotIn('a', tuple(item.robot_id for item in graph['b']))
        self.assertEqual((), graph['far'])

    def test_duplicate_robot_ids_are_rejected(self):
        with self.assertRaises(ValueError):
            build_local_neighbor_graph(
                (
                    RobotSnapshot('same', Vec2()),
                    RobotSnapshot('same', Vec2(1.0, 0.0)),
                ),
                sensing_radius=2.0,
            )


class AdapterTests(unittest.TestCase):
    def test_centered_contour_and_snapshot_use_finite_gazebo_points(self):
        center = Vec2(1.0, -2.0)
        contour = centered_object_contour(center, 0.2, sample_count=8)
        self.assertEqual(8, len(contour.points))
        for point in contour.points:
            self.assertAlmostEqual(0.2, (point - center).norm(), places=12)

        robot = RobotSnapshot('tb3_0', Vec2())
        snapshot = build_transport_snapshot(
            robots=(robot,),
            object_center=center,
            target=Vec2(3.0, 3.0),
            obstacle_points=(Vec2(0.5, 0.5), Vec2(float('nan'), 1.0)),
            object_radius=0.2,
            contour_samples=8,
        )
        self.assertEqual((robot,), snapshot.robots)
        self.assertEqual((Vec2(0.5, 0.5),), snapshot.obstacle_points)
        self.assertEqual(contour, snapshot.object_contour)

    def test_oriented_box_contour_samples_rotated_perimeter_and_corners(self):
        center = Vec2(1.0, -2.0)
        half_width = 0.15
        half_height = 0.20
        yaw = math.pi / 2.0
        contour = oriented_box_contour(
            center,
            half_width,
            half_height,
            yaw,
            sample_count=16,
        )

        self.assertEqual(16, len(contour.points))
        cosine = math.cos(yaw)
        sine = math.sin(yaw)
        local_points = []
        for point in contour.points:
            offset = point - center
            local = Vec2(
                offset.x * cosine + offset.y * sine,
                -offset.x * sine + offset.y * cosine,
            )
            local_points.append(local)
            self.assertLessEqual(abs(local.x), half_width + 1.0e-12)
            self.assertLessEqual(abs(local.y), half_height + 1.0e-12)
            self.assertTrue(
                abs(abs(local.x) - half_width) <= 1.0e-12
                or abs(abs(local.y) - half_height) <= 1.0e-12
            )

        self.assertAlmostEqual(-half_width, min(p.x for p in local_points))
        self.assertAlmostEqual(half_width, max(p.x for p in local_points))
        self.assertAlmostEqual(-half_height, min(p.y for p in local_points))
        self.assertAlmostEqual(half_height, max(p.y for p in local_points))
        self.assertAlmostEqual(center.x, contour.center().x)
        self.assertAlmostEqual(center.y, contour.center().y)

    def test_box_snapshot_requires_both_half_extents(self):
        with self.assertRaisesRegex(ValueError, 'both half extents'):
            build_transport_snapshot(
                robots=(),
                object_center=Vec2(),
                target=Vec2(1.0, 0.0),
                object_half_width=0.2,
            )

    def test_large_fleet_uses_the_lower_proposal_count_at_threshold(self):
        self.assertEqual(60, mcmc_iterations_for_fleet(19, 60, 20, 24))
        self.assertEqual(24, mcmc_iterations_for_fleet(20, 60, 20, 24))
        self.assertEqual(24, mcmc_iterations_for_fleet(100, 60, 20, 24))
        self.assertEqual(60, mcmc_iterations_for_fleet(20, 60, 20, 80))

    def test_contact_readiness_uses_footprints_not_sensor_range(self):
        ready_distance = contact_center_distance(0.20, 0.11, 0.08)
        self.assertAlmostEqual(0.39, ready_distance)
        self.assertLess(ready_distance, GRFConfig().sensing_radius)

    def test_object_sectors_are_masked_only_in_the_contact_corridor(self):
        near = occupied_lidar_sectors(
            robot_position=Vec2(),
            robot_heading=0.0,
            object_center=Vec2(0.5, 0.0),
            object_radius=0.25,
            maximum_distance=0.75,
        )
        far = occupied_lidar_sectors(
            robot_position=Vec2(),
            robot_heading=0.0,
            object_center=Vec2(0.8, 0.0),
            object_radius=0.25,
            maximum_distance=0.75,
        )
        self.assertIn(0, near)
        self.assertNotIn(4, near)
        self.assertEqual((), far)


class ObjectInteractionTests(unittest.TestCase):
    def setUp(self):
        self.contour = square_contour()
        self.target = Vec2(2.0, 0.0)

    def test_object_occlusion_selects_pushing_side(self):
        self.assertTrue(
            target_is_occluded(Vec2(-1.0, 0.0), self.target, self.contour)
        )
        self.assertFalse(
            target_is_occluded(Vec2(1.0, 0.0), self.target, self.contour)
        )

    def test_square_snapshot_preserves_push_side_mode_selection(self):
        robot = RobotSnapshot(
            'contact',
            Vec2(-0.2404067827, 0.0875009130),
            heading=-0.34906585,
        )
        box_snapshot = build_transport_snapshot(
            robots=(robot,),
            object_center=Vec2(),
            target=Vec2(3.0, 3.0),
            contour_samples=16,
            object_half_width=0.2,
            object_half_height=0.2,
        )
        circle_snapshot = build_transport_snapshot(
            robots=(robot,),
            object_center=Vec2(),
            target=Vec2(3.0, 3.0),
            object_radius=0.2,
            contour_samples=16,
        )
        kernel = GibbsRandomFieldTransport(GRFConfig(sensing_radius=3.0))

        box = kernel.evaluate_velocity(box_snapshot, 'contact', Vec2())
        circle = kernel.evaluate_velocity(circle_snapshot, 'contact', Vec2())

        self.assertEqual(InteractionMode.PUSH, box.interaction_mode)
        self.assertEqual(InteractionMode.ORBIT, circle.interaction_mode)

    def test_orbiting_prefers_local_contour_tangent(self):
        zero = disabled_potential()
        config = GRFConfig(
            max_speed=0.2,
            sensing_radius=3.0,
            obstacle_potential=zero,
            object_orbit_potential=zero,
            object_push_potential=zero,
            robot_potential=zero,
            orbit_alignment_weight=20.0,
            push_alignment_weight=20.0,
            velocity_consensus_weight=0.0,
            cruise_speed_weight=0.0,
            mcmc_iterations=4,
        )
        robot = RobotSnapshot('front', Vec2(1.0, 0.3))
        snapshot = TransportSnapshot(
            robots=(robot,),
            object_contour=self.contour,
            target=self.target,
        )
        kernel = GibbsRandomFieldTransport(config)
        initial = kernel.evaluate_velocity(snapshot, 'front', Vec2())
        tangent_velocity = initial.preferred_direction * config.max_speed
        tangent = kernel.evaluate_velocity(
            snapshot, 'front', tangent_velocity
        )
        radial = kernel.evaluate_velocity(
            snapshot, 'front', Vec2(config.max_speed, 0.0)
        )
        self.assertEqual(InteractionMode.ORBIT, tangent.interaction_mode)
        self.assertGreater(abs(initial.preferred_direction.y), 0.9)
        self.assertLess(tangent.object_energy, radial.object_energy)

    def test_goal_directed_velocity_is_preferred_when_pushing(self):
        zero = disabled_potential()
        config = GRFConfig(
            max_speed=0.2,
            sensing_radius=3.0,
            obstacle_potential=zero,
            object_orbit_potential=zero,
            object_push_potential=zero,
            robot_potential=zero,
            orbit_alignment_weight=20.0,
            push_alignment_weight=20.0,
            velocity_consensus_weight=0.0,
            cruise_speed_weight=0.0,
            mcmc_iterations=4,
        )
        robot = RobotSnapshot('behind', Vec2(-1.0, 0.0))
        snapshot = TransportSnapshot(
            robots=(robot,),
            object_contour=self.contour,
            target=self.target,
        )
        kernel = GibbsRandomFieldTransport(config)
        toward_goal = kernel.evaluate_velocity(
            snapshot, 'behind', Vec2(config.max_speed, 0.0)
        )
        away_from_goal = kernel.evaluate_velocity(
            snapshot, 'behind', Vec2(-config.max_speed, 0.0)
        )
        self.assertEqual(InteractionMode.PUSH, toward_goal.interaction_mode)
        self.assertLess(
            toward_goal.object_energy, away_from_goal.object_energy
        )

    def test_default_tuning_prefers_orbiting_over_radial_attraction(self):
        robot = RobotSnapshot('side', Vec2(0.0, 1.0))
        snapshot = TransportSnapshot(
            robots=(robot,),
            object_contour=self.contour,
            target=Vec2(-2.0, 0.0),
        )
        kernel = GibbsRandomFieldTransport(GRFConfig(sensing_radius=3.0))
        idle = kernel.evaluate_velocity(snapshot, 'side', Vec2())
        preferred = kernel.evaluate_velocity(
            snapshot,
            'side',
            idle.preferred_direction * kernel.config.max_speed,
        )
        radial = kernel.evaluate_velocity(
            snapshot, 'side', Vec2(0.0, -kernel.config.max_speed)
        )
        self.assertEqual(InteractionMode.ORBIT, preferred.interaction_mode)
        self.assertLess(preferred.object_energy, radial.object_energy)

    def test_contour_resolution_does_not_multiply_object_energy(self):
        robot = RobotSnapshot('side', Vec2(0.0, 1.0))
        kernel = GibbsRandomFieldTransport(GRFConfig(sensing_radius=3.0))
        energies = []
        for sample_count in (8, 16, 64):
            snapshot = build_transport_snapshot(
                robots=(robot,),
                object_center=Vec2(),
                target=Vec2(-2.0, 0.0),
                object_radius=0.2,
                contour_samples=sample_count,
            )
            result = kernel.evaluate_velocity(
                snapshot, 'side', Vec2(kernel.config.max_speed, 0.0)
            )
            energies.append(result.object_energy)
        self.assertLess(max(energies) - min(energies), 0.15)


class SamplerTests(unittest.TestCase):
    def setUp(self):
        self.snapshot = TransportSnapshot(
            robots=(
                RobotSnapshot('tb3_10', Vec2(-0.8, 0.0), Vec2()),
                RobotSnapshot('tb3_2', Vec2(-0.8, 0.5), Vec2()),
                RobotSnapshot('tb3_far', Vec2(4.0, 4.0), Vec2()),
            ),
            object_contour=square_contour(),
            target=Vec2(2.0, 0.0),
            obstacle_points=(Vec2(-0.5, -0.5),),
        )
        self.config = GRFConfig(
            random_seed=73,
            mcmc_iterations=24,
            sensing_radius=2.0,
            max_speed=0.15,
        )

    def test_sampling_is_repeatable_bounded_and_locally_scoped(self):
        first = GibbsRandomFieldTransport(self.config).compute(
            self.snapshot, step_index=9
        )
        second = GibbsRandomFieldTransport(self.config).compute(
            self.snapshot, step_index=9
        )
        self.assertEqual(first, second)
        self.assertEqual(
            ('tb3_10', 'tb3_2', 'tb3_far'),
            tuple(result.robot_id for result in first.robots),
        )
        for result in first.robots:
            self.assertTrue(result.velocity.is_finite())
            self.assertLessEqual(
                result.velocity.norm(), self.config.max_speed + 1.0e-12
            )
            self.assertTrue(math.isfinite(result.total_energy))
            self.assertLessEqual(
                result.accepted_proposals, result.proposal_count
            )
        self.assertEqual(('tb3_2',), first.command_for('tb3_10').neighbor_ids)
        self.assertEqual((), first.command_for('tb3_far').neighbor_ids)

    def test_one_robot_box_sampler_makes_goal_directed_push_progress(self):
        robot = RobotSnapshot('solo', Vec2(-0.243, 0.0), Vec2())
        snapshot = build_transport_snapshot(
            robots=(robot,),
            object_center=Vec2(),
            target=Vec2(3.0, 0.0),
            contour_samples=16,
            object_half_width=0.2,
            object_half_height=0.2,
        )
        config = GRFConfig(
            random_seed=0,
            mcmc_iterations=60,
            sensing_radius=2.0,
            max_speed=0.15,
        )
        kernel = GibbsRandomFieldTransport(config)
        goal_direction = snapshot.target.normalized()
        progress = 0.0

        for step_index in range(10):
            command = kernel.compute(
                snapshot, step_index=step_index
            ).command_for('solo')
            self.assertEqual(InteractionMode.PUSH, command.interaction_mode)
            self.assertEqual((), command.neighbor_ids)
            self.assertTrue(command.velocity.is_finite())
            self.assertLessEqual(
                command.velocity.norm(), config.max_speed + 1.0e-12
            )
            progress += command.velocity.dot(goal_direction)

        self.assertGreater(progress, 0.5)

    def test_robot_input_order_does_not_change_results(self):
        reversed_snapshot = TransportSnapshot(
            robots=tuple(reversed(self.snapshot.robots)),
            object_contour=self.snapshot.object_contour,
            target=self.snapshot.target,
            obstacle_points=self.snapshot.obstacle_points,
        )
        kernel = GibbsRandomFieldTransport(self.config)
        self.assertEqual(
            kernel.compute(self.snapshot, step_index=4),
            kernel.compute(reversed_snapshot, step_index=4),
        )

    def test_invalid_position_fails_safe_without_poisoning_other_results(self):
        snapshot = TransportSnapshot(
            robots=(
                RobotSnapshot('invalid', Vec2(float('nan'), 0.0)),
                RobotSnapshot('valid', Vec2(-1.0, 0.0)),
            ),
            object_contour=square_contour(),
            target=Vec2(2.0, 0.0),
        )
        result = GibbsRandomFieldTransport(self.config).compute(snapshot)
        invalid = result.command_for('invalid')
        valid = result.command_for('valid')
        self.assertEqual(InteractionMode.INVALID, invalid.interaction_mode)
        self.assertEqual(Vec2(), invalid.velocity)
        self.assertTrue(valid.velocity.is_finite())

    def test_neighbor_energy_is_normalized_across_fleet_sizes(self):
        kernel = GibbsRandomFieldTransport(self.config)
        robot = RobotSnapshot('subject', Vec2(-1.0, 0.0))

        def evaluation_with_neighbors(count):
            neighbors = tuple(
                RobotSnapshot(
                    'neighbor_{}'.format(index),
                    Vec2(-0.5, 0.0),
                    Vec2(0.05, 0.0),
                )
                for index in range(count)
            )
            snapshot = TransportSnapshot(
                robots=(robot,) + neighbors,
                object_contour=square_contour(),
                target=Vec2(2.0, 0.0),
            )
            graph = {'subject': neighbors}
            return kernel.evaluate_velocity(
                snapshot,
                'subject',
                Vec2(0.1, 0.0),
                neighbor_graph=graph,
            ).neighbor_energy

        self.assertAlmostEqual(
            evaluation_with_neighbors(1),
            evaluation_with_neighbors(9),
            places=12,
        )

    def test_opposing_neighbor_velocities_do_not_cancel_consensus_cost(self):
        # Isolate velocity consensus from the position potential: neighbors
        # moving in opposite directions predict different positions, but the
        # two squared velocity mismatches should still carry the same cost.
        kernel = GibbsRandomFieldTransport(GRFConfig(
            random_seed=73,
            mcmc_iterations=24,
            sensing_radius=2.0,
            max_speed=0.15,
            robot_potential=disabled_potential(),
        ))
        subject = RobotSnapshot('subject', Vec2(-1.0, 0.0))

        def evaluate(velocities):
            neighbors = tuple(
                RobotSnapshot(
                    'neighbor_{}'.format(index),
                    Vec2(-0.5, 0.0),
                    velocity,
                )
                for index, velocity in enumerate(velocities)
            )
            snapshot = TransportSnapshot(
                robots=(subject,) + neighbors,
                object_contour=square_contour(),
                target=Vec2(2.0, 0.0),
            )
            return kernel.evaluate_velocity(
                snapshot,
                'subject',
                Vec2(),
                neighbor_graph={'subject': neighbors},
            ).neighbor_energy

        aligned = evaluate((Vec2(0.15, 0.0), Vec2(0.15, 0.0)))
        opposed = evaluate((Vec2(0.15, 0.0), Vec2(-0.15, 0.0)))

        self.assertAlmostEqual(aligned, opposed, places=12)


if __name__ == '__main__':
    unittest.main()
