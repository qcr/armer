#!/usr/bin/env python
"""
@author: Gavin Suddrey
"""

import unittest
import subprocess
import signal
import time
import rospy

from roboticstoolbox.backends.Swift import Swift
from roboticstoolbox.models.URDF.Panda import Panda

from armer.robots.ROSRobot import ROSRobot

class TestDriver(unittest.TestCase):
    """
    Test Armer Driver
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
        from armer import Armer #pylint: disable=import-outside-toplevel
        self.assertIsInstance(Armer, object)

    def test_driver_init(self):
        """
        Test initialising driver
        """
        from armer import Armer #pylint: disable=import-outside-toplevel
        driver = Armer(robots=[ROSRobot(Panda())], backend=Swift(), backend_args={'headless': True})
        self.assertIsInstance(driver, Armer)
        driver.close()

if __name__ == '__main__':
    unittest.main()
