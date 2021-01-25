#!/usr/bin/env python3
"""
@author: Gavin Suddrey
"""

import unittest

class TestDriver(unittest.TestCase):
    """
    Test Manipulation Driver
    """
    def test_driver_import(self):
        """
        Test driver import
        """
        from manipulation_driver import ManipulationDriver #pylint: disable=import-outside-toplevel
        self.assertIsInstance(ManipulationDriver, object)



if __name__ == '__main__':
    unittest.main()
