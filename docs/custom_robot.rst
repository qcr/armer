Creating A Custom Robot
========================

Sometimes a manipulator's ROS drivers will not be in the format Armer expects or has features beyond Armer's standard framework.

Any discrepencies can be adjusted for by extending the Armer Robot class.

.. image:: https://github.com/qcr/armer/wiki/customrosrobot.png
  :alt: Custom robot block diagram

Possible cases include:

* The ROS drivers using different message types from Armer
* Adding functionality such as recovery functionality

To create a custom robot, the following template can be used:

.. code-block:: python

    import rospy
    import actionlib
    import roboticstoolbox as rtb

    from armer.robots import ROSRobot

    class {CUSTOM_ROBOT_NAME}ROSRobot(ROSRobot):
        def __init__(self,
                    robot: rtb.robot.Robot,
                    controller_name: str = None,
                    *args, 
                    **kwargs):

            super().__init__(robot, *args, **kwargs)
        

Any variant functionality can be declared as a function that will be called instead of the default Armer functions such as get_state.

To use the custom robot, call it in the config file. This is done by specifying the type field of the relevant robot subsection. For example:

.. code-block:: YAML

    robots:
    - name: arm 
        model: roboticstoolbox.models.{ROBOT_MODEL}
        type: {HARDWARE_PACKAGE_NAME}.robots.{CUSTOM_ROBOT_NAME}ROSRobot
    backend: 
    type: armer.backends.ROS.ROS
