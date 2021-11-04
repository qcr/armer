Creating A Custom Robot
========================

Sometimes a manipulator's ROS drivers will not be in the format Armer expects or has features beyond Armer's standard framework.

Any discrepencies can be adjusted for by extending the Armer Robot class.

.. image:: https://github.com/qcr/armer/wiki/customrosrobot.png
  :alt: Custom robot block diagram

Possible cases include:

* The ROS drivers using different message types from Armer
* Adding functionality such as recovery functionality

Create the following file structure:

.. code-block:: bash

    ├── HARDWARE_PACKAGE_NAME
    │   ├── __init__.py
    │   └── robots
    │       ├── __init__.py
    │       └── ROBOT_NAME.py
    ├── cfg
    │   ├── ROBOT_NAME_real.yaml
    │   └── ROBOT_NAME_sim.yaml
    ├── CMakeLists.txt
    ├── launch
    │   └── robot_bringup.launch
    ├── LICENSE
    ├── package.xml
    ├── README.md
    └── setup.py

To create the custom robot, the following template can be used. Save it as ROBOT_NAME.py in the robots folder:

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
        
        def the_function_to_override():
            print("cool code")
        

Any variant functionality can be declared as a function that will be called instead of the default Armer functions such as get_state.

In order for Armer and RTB to be able to find the custom robot, the ``__init__.py`` and ``setup.py`` files must also be configured correctly.

The contents ``__init__.py`` of should be as follows:

    .. code-block:: python

        from armer_ROBOT_NAME.robots.ROBOT_NAME import {CUSTOM_ROBOT_NAME}ROSRobot
        __all__ = [
            {CUSTOM_ROBOT_NAME}ROSRobot
        ]

An example of ``setup.py`` can be found in `armer_mico <https://github.com/qcr/armer_mico>`_. Modify armer_mico instances to match the name of the custom package.

To use the custom robot, call it in the config file. This is done by specifying the type field of the relevant robot subsection. For example:

.. code-block:: yaml

    robots:
    - name: arm 
        model: roboticstoolbox.models.{ROBOT_MODEL}
        type: {HARDWARE_PACKAGE_NAME}.robots.{CUSTOM_ROBOT_NAME}ROSRobot
    backend: 
    type: armer.backends.ROS.ROS
