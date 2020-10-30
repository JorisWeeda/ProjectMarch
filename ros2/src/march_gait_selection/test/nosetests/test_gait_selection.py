#!/usr/bin/env python
from copy import deepcopy
import unittest

import rclpy
from ament_index_python import get_package_share_directory, PackageNotFoundError
from march_gait_selection.dynamic_gaits.semi_dynamic_setpoints_gait import SemiDynamicSetpointsGait
from urdf_parser_py import urdf
from march_gait_selection.gait_selection import GaitSelection
from march_gait_selection.state_machine.gait_interface import GaitInterface
from march_shared_classes.gait.gait import Gait

VALID_PACKAGE = 'march_gait_selection'
VALID_DIRECTORY = 'test/resources'


class TestGaitSelection(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.robot = urdf.Robot.from_xml_file(
            get_package_share_directory('march_description') + '/urdf/march4.urdf')

    def setUp(self):
        self.gait_selection = GaitSelection(
            gait_package=VALID_PACKAGE, directory=VALID_DIRECTORY, robot=self.robot)

    # __init__ tests
    def test_init_with_wrong_package(self):
        with self.assertRaises(PackageNotFoundError):
            GaitSelection(gait_package='wrong', directory=VALID_DIRECTORY)

    def test_init_with_wrong_directory(self):
        with self.assertRaises(FileNotFoundError):
            GaitSelection(gait_package=VALID_PACKAGE, directory='wrong')

    # load gaits tests
    def test_types_in_loaded_gaits(self):
        for gait in self.gait_selection._loaded_gaits.values():
            self.assertTrue(isinstance(gait, GaitInterface))

    def test_gait_selection_positions(self):
        self.assertIn('stand', self.gait_selection.positions)

    # scan directory tests
    def test_scan_directory_top_level_content(self):
        directory = self.gait_selection.scan_directory()
        directory_gaits = ['walk_medium', 'balance_walk', 'stairs_up', 'walk_small', 'walk']
        self.assertEqual(sorted(directory.keys()), sorted(directory_gaits))

    def test_scan_directory_subgait_versions(self):
        directory = self.gait_selection.scan_directory()
        self.assertEqual(directory['walk']['left_swing'], ['MV_walk_leftswing_v2'])

    # get item tests
    def test_get_item_with_wrong_name(self):
        self.assertIsNone(self.gait_selection['wrong'])

    def test_get_item_type(self):
        self.assertIsInstance(self.gait_selection['walk'], Gait)

    def test_dynamic_gait_loaded(self):
        self.assertIsInstance(self.gait_selection['dynamic_stairs_up'],
                              SemiDynamicSetpointsGait)
