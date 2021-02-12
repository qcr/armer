#!/usr/bin/env python3
"""
@author: Gavin Suddrey
"""

import unittest
import subprocess
import signal
import time
import rospy

import roboticstoolbox as rtb

class TestDriver(unittest.TestCase):
    """
    Test Manipulation Driver
    """
    @classmethod
    def setUpClass(cls):
        cls.core = subprocess.Popen(['roscore'])
        rospy.init_node('test_driver')

        for _ in range(5):
            try:
                rospy.get_master()
                break
            except Exception as _:  #pylint: disable=broad-except
                time.sleep(0.5)

    @classmethod
    def tearDownClass(cls):
        cls.core.send_signal(signal.SIGINT)
        cls.core.wait()

    def tearDown(self):
        subprocess.Popen(['rosparam', 'delete', '/'])

    def test_driver_import(self):
        """
        Test driver import
        """
        from manipulation_driver import ManipulationDriver #pylint: disable=import-outside-toplevel
        self.assertIsInstance(ManipulationDriver, object)

    def test_driver_init(self):
        """
        Test initialising driver
        """
        from manipulation_driver import ManipulationDriver #pylint: disable=import-outside-toplevel
        driver = ManipulationDriver(backend=rtb.backends.Swift(display=False))
        self.assertIsInstance(driver, ManipulationDriver)
        driver.close()

if __name__ == '__main__':
    unittest.main()
