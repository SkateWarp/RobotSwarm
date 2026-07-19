#!/usr/bin/env python3
"""Checks the simulation-speed contract shared by the swarm worlds."""

import math
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path


PACKAGE_DIR = Path(__file__).resolve().parents[1]
WORLDS_DIR = PACKAGE_DIR / 'worlds'
LAUNCH_DIR = PACKAGE_DIR / 'launch'
MODELS_DIR = PACKAGE_DIR / 'models'


class WorldPhysicsTests(unittest.TestCase):
    def _physics(self, world_name):
        root = ET.parse(WORLDS_DIR / world_name).getroot()
        physics = root.find('./world/physics')
        self.assertIsNotNone(physics, f'{world_name} has no physics profile')
        return physics

    def test_swarm_worlds_target_three_times_real_time(self):
        for world_name in ('swarm_arena.world', 'empty_arena.world'):
            with self.subTest(world=world_name):
                physics = self._physics(world_name)
                update_rate = float(physics.findtext('real_time_update_rate'))
                step_size = float(physics.findtext('max_step_size'))
                declared_factor = float(physics.findtext('real_time_factor'))

                self.assertAlmostEqual(3.0, update_rate * step_size)
                self.assertAlmostEqual(3.0, declared_factor)
                self.assertLessEqual(step_size, 0.003)

    def test_legacy_arena_uses_the_project_empty_world(self):
        root = ET.parse(LAUNCH_DIR / 'arena.launch').getroot()
        gazebo = root.find("./include[@file='$(find gazebo_ros)/launch/empty_world.launch']")
        self.assertIsNotNone(gazebo)

        world = gazebo.find("./arg[@name='world_name']")
        self.assertIsNotNone(world)
        self.assertEqual(
            '$(find robot_swarm_bridge)/worlds/empty_arena.world',
            world.get('value'),
        )

    def test_swarm_launch_limits_clock_and_per_robot_tf_overhead(self):
        root = ET.parse(LAUNCH_DIR / 'swarm_main.launch').getroot()
        arguments = {
            argument.get('name'): argument.get('default')
            for argument in root.findall('./arg')
        }
        self.assertEqual('100', arguments['gazebo_clock_frequency'])
        self.assertEqual('false', arguments['publish_robot_tf'])

        clock = root.find("./param[@name='/gazebo/pub_clock_frequency']")
        self.assertIsNotNone(clock)
        self.assertEqual('double', clock.get('type'))
        self.assertEqual('$(arg gazebo_clock_frequency)', clock.get('value'))

        gazebo = root.find(
            "./include[@file='$(find gazebo_ros)/launch/empty_world.launch']"
        )
        self.assertIsNotNone(gazebo)
        self.assertLess(list(root).index(clock), list(root).index(gazebo))

        fleet = root.find("./node[@name='fleet_manager']")
        self.assertIsNotNone(fleet)
        tf_setting = fleet.find("./param[@name='publish_robot_tf']")
        self.assertIsNotNone(tf_setting)
        self.assertEqual('bool', tf_setting.get('type'))
        self.assertEqual('$(arg publish_robot_tf)', tf_setting.get('value'))

    def test_ode_solver_uses_a_practical_iteration_count(self):
        physics = self._physics('swarm_arena.world')
        iterations = int(physics.findtext('./ode/solver/iters'))

        self.assertGreaterEqual(iterations, 20)
        self.assertLessEqual(iterations, 50)
        constraints = physics.find('./ode/constraints')
        self.assertAlmostEqual(
            10.0,
            float(constraints.findtext('contact_max_correcting_vel')),
        )
        self.assertAlmostEqual(
            0.001,
            float(constraints.findtext('contact_surface_layer')),
        )

    def test_transport_payload_is_supported_and_pushable(self):
        root = ET.parse(WORLDS_DIR / 'swarm_arena.world').getroot()
        payload = root.find("./world/model[@name='transport_object']")
        self.assertIsNotNone(payload)
        self.assertNotEqual('true', payload.findtext('static', 'false').lower())

        collision_size = tuple(float(value) for value in
                               payload.findtext(
                                   './link/collision/geometry/box/size'
                               ).split())
        visual_size = tuple(float(value) for value in
                            payload.findtext(
                                './link/visual/geometry/box/size'
                            ).split())
        self.assertEqual((0.4, 0.4, 0.2), collision_size)
        self.assertEqual(collision_size, visual_size)

        pose = tuple(float(value) for value in payload.findtext('pose').split())
        self.assertAlmostEqual(collision_size[2] / 2.0, pose[2])

        mass = float(payload.findtext('./link/inertial/mass'))
        self.assertEqual(0.25, mass)
        width, depth, height = collision_size
        expected_inertia = {
            'ixx': mass * (depth * depth + height * height) / 12.0,
            'iyy': mass * (width * width + height * height) / 12.0,
            'izz': mass * (width * width + depth * depth) / 12.0,
        }
        for name, expected in expected_inertia.items():
            actual = float(payload.findtext(
                './link/inertial/inertia/{}'.format(name)
            ))
            self.assertTrue(math.isclose(actual, expected, abs_tol=1.0e-6))

        friction = payload.find('./link/collision/surface/friction/ode')
        self.assertAlmostEqual(0.05, float(friction.findtext('mu')))
        self.assertAlmostEqual(0.05, float(friction.findtext('mu2')))

        contact = payload.find('./link/collision/surface/contact/ode')
        self.assertAlmostEqual(0.02, float(contact.findtext('max_vel')))
        self.assertAlmostEqual(0.001, float(contact.findtext('min_depth')))

        decay = payload.find('./link/velocity_decay')
        self.assertAlmostEqual(0.002, float(decay.findtext('linear')))
        self.assertAlmostEqual(0.005, float(decay.findtext('angular')))

    def test_loaded_transport_profile_brackets_one_and_four_burgers(self):
        root = ET.parse(
            MODELS_DIR / 'transport_crate_loaded' / 'model.sdf'
        ).getroot()
        payload = root.find("./model[@name='transport_object']")
        self.assertIsNotNone(payload)

        collision_size = tuple(float(value) for value in
                               payload.findtext(
                                   './link/collision/geometry/box/size'
                               ).split())
        visual_size = tuple(float(value) for value in
                            payload.findtext(
                                './link/visual/geometry/box/size'
                            ).split())
        self.assertEqual((0.4, 0.4, 0.2), collision_size)
        self.assertEqual(collision_size, visual_size)

        mass = float(payload.findtext('./link/inertial/mass'))
        friction = payload.find('./link/collision/surface/friction/ode')
        mu = float(friction.findtext('mu'))
        mu2 = float(friction.findtext('mu2'))
        self.assertEqual(0.75, mass)
        self.assertEqual(0.25, mu)
        self.assertEqual(mu, mu2)

        width, depth, height = collision_size
        expected_inertia = {
            'ixx': mass * (depth * depth + height * height) / 12.0,
            'iyy': mass * (width * width + height * height) / 12.0,
            'izz': mass * (width * width + depth * depth) / 12.0,
        }
        for name, expected in expected_inertia.items():
            actual = float(payload.findtext(
                './link/inertial/inertia/{}'.format(name)
            ))
            self.assertTrue(math.isclose(actual, expected, abs_tol=1.0e-6))

        gravity = 9.80665
        nominal_burger_mass = 1.0
        burger_wheel_mu = 0.1
        crate_breakaway_force = mass * mu * gravity
        one_robot_traction = nominal_burger_mass * burger_wheel_mu * gravity
        self.assertGreater(crate_breakaway_force, one_robot_traction)
        self.assertLess(crate_breakaway_force, 4.0 * one_robot_traction)

    def test_loaded_transport_model_is_installed_with_the_package(self):
        # CMake is not XML; keep this check intentionally simple and readable.
        cmake = (PACKAGE_DIR / 'CMakeLists.txt').read_text()
        install_blocks = [
            block.split(')', 1)[0]
            for block in cmake.split('install(DIRECTORY')[1:]
        ]
        self.assertTrue(any(
            'models' in block.split() for block in install_blocks
        ))


if __name__ == '__main__':
    unittest.main()
