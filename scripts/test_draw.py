#!/usr/bin/env python
PKG = 'pr2_draw'
import roslib; roslib.load_manifest(PKG)

import sys
import unittest

class TestDraw(unittest.TestCase):
    """Tests draw.py"""
    def __init__(self):
        super(TestDraw, self).__init__()

    def basic_test(self):
        self.assertEquals(4, 5, "4 != 5")
        self.assertEqual(2, 3, "2 != 3")

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'test_draw', TestDraw)
