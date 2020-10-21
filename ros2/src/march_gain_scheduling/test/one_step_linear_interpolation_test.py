#!/usr/bin/env python

import unittest

import numpy as np

from march_gain_scheduling.interpolation_errors import NegativeValueError, UnequalLengthError
from march_gain_scheduling.one_step_linear_interpolation import interpolate

PKG = 'march_example_node'


class OneStepLinearInterpolationTest(unittest.TestCase):
    def test_interpolate_up(self):
        current = np.array([0, 0, 0])
        needed = np.array([1, 1, 1])
        result = interpolate(current, needed, 1, 0.1)
        self.assertTrue((result == np.array([0.1, 0.1, 0.1])).all())

    def test_interpolate_down(self):
        current = np.array([5, 5, 5])
        needed = np.array([3, 3, 3])
        result = interpolate(current, needed, 1, 0.1)
        self.assertTrue((result == np.array([4.9, 4.9, 4.9])).all())

    def test_interpolate_up_and_down(self):
        current = np.array([2, 5, 6])
        needed = np.array([4, 3, 4])
        result = interpolate(current, needed, 3, 0.1)
        self.assertTrue((result == np.array([2.3, 4.7, 5.7])).all())

    def test_interpolate_list_length(self):
        current = np.array([2, 5, 6, 8])
        needed = np.array([4, 3, 4])
        with self.assertRaises(UnequalLengthError):
            interpolate(current, needed, 1, 0.1)

    def test_negative_value_error(self):
        current = np.array([2, 5, 6])
        needed = np.array([4, 3, 4])
        with self.assertRaises(NegativeValueError):
            interpolate(current, needed, -1, 0.1)

    def test_interpolate_different_paths(self):
        current = np.array([1, 1, 1])
        needed = np.array([2, 5, 4])
        for i in range(26):  # Loop until needed is reached and a bit further
            current = interpolate(current, needed, 1, 0.2)
        self.assertTrue((current == np.array([2, 5, 4])).all())

    def test_interpolate_empty_lists(self):
        current = np.array([])
        needed = np.array([])
        result = interpolate(current, needed, 1, 0.1)
        self.assertTrue((result == np.array([])).all())
