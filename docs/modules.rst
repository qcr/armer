
Modules
====================================

The main Armer class creates an instance of the ROSRobot class. The ROSRobot class takes a `Robotics Toolbox URDF model <https://petercorke.github.io/robotics-toolbox-python/arm_erobot.html?highlight=urdf#module-roboticstoolbox.models.URDF>`_ to create an ROS robot object.

Armer class
---------------
The Armer class is the wrapper class that launches the ROSRobot object class and backends. 

.. automodule:: armer.armer
   :members:
   :undoc-members:
   :show-inheritance:

ROSRobot module
---------------

The ROSRobot class inheirits from the RTB ERobot class to wrap ROS functionality and movement into the implementation.

.. inheritance-diagram:: armer.robots.ROSRobot
   :top-classes: roboticstoolbox.Robot
   :parts: 1

.. automodule:: armer.robots.ROSRobot
   :members:
   :undoc-members:
   :show-inheritance:


ROS Backend
-------------
Armer implements a ROS backend as a Python class for interfacing with physical arms. The class contains several functions to simplify ROS interactions.

.. automodule:: armer.backends.ROS.ROS
   :members:
   :undoc-members:
   :show-inheritance:
