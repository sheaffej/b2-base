import unittest
from math import pi

from b2_logic.common_functions import add_radians


class TestCommonFunctions(unittest.TestCase):

    def setUp(self):
        print()

    def test_add_radians(self):
        # a, b, expected
        tests = [
            (0, pi / 2, pi / 2),
            (pi / 4, -(pi / 2), -(pi / 4)),
            (0, pi, pi),
            (pi, -(pi), 0),
            (0, pi * 2, 0),
            (pi, -(pi * 2), pi),
            (-pi, pi * 2, pi)
        ]

        for a, b, expected in tests:
            result = add_radians(a, b)
            print("{} + {}, expected {}, actual {}".format(a, b, expected, result))
            self.assertEqual(result, expected)
