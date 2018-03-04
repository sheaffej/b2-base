#!/usr/bin/env python
from __future__ import print_function
import unittest


PKG = 'b2_base'
NAME = 'b2_ir_sensors_unittest'


class TestIRSensors(unittest.TestCase):

    def __init__(self, *args):
        super(TestIRSensors, self).__init__(*args)

    def test_dummy(self):
        self.assertEqual(1, 1)


if __name__ == "__main__":
    import rosunit
    rosunit.unitrun(PKG, NAME, TestIRSensors)
