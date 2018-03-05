#!/usr/bin/env python
from __future__ import print_function
import unittest

import b2_base

PKG = 'b2_base'
NAME = 'b2_ir_sensors_unittest'


class TestIRSensors(unittest.TestCase):

    def __init__(self, *args):
        super(TestIRSensors, self).__init__(*args)
        self.ir_sensors = 

    def test_volts_per_adc(self):
        # (vref, min_adc_reading, max_adc_reading, expected)
        tests = [
            (5.0, 0, 1023, round(5.0/1023, 3)),
            (3.3, 0, 1023, round(3.3/1023, 3))
        ]
        for vref, min_adc, max_adc, expected in tests:
            actual = round(
                        self.ir_sensors._volts_per_adc(
                            vref,
                            min_adc,
                            max_adc), 3)
            self.assertEqual(actual, expected)


if __name__ == "__main__":
    import rosunit
    rosunit.unitrun(PKG, NAME, TestIRSensors)
