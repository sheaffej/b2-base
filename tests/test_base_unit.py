#!/usr/bin/env python
from __future__ import print_function
import unittest


PKG = 'b2_base'
NAME = 'b2_base_unittest'


class TestBase(unittest.TestCase):

    def __init__(self, *args):
        super(TestBase, self).__init__(*args)

    def test_dummy(self):
        self.assertEqual(1, 1)


if __name__ == "__main__":
    import rosunit
    rosunit.unitrun(PKG, NAME, TestBase)
