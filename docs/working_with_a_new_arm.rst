Working With A New Arm
====================================

First install the manipulator's drivers.

If there is no existing `armer_{ROBOT_MODEL}` package in the QCR repos see:
    * `Creating a launch file <https://github.com/qcr/armer_panda/>`_
    * `Creating a yaml config <https://github.com/qcr/armer_panda/>`_
    * `Creating an Armer package <https://github.com/qcr/armer_panda/>`_

If there is no Robotic Toolbox model available see:
    * `Creating a Robotics Toolbox model <https://github.com/qcr/armer_panda/>`_



The Armer drivers load a URDF model into the Python Robotic Toolbox framework to process the kinematics and other movement related calculations.

If there is no existing `armer_{ROBOT_MODEL}` package in the QCR repos one can be  created by ` creating a launch file <https://github.com/qcr/armer_panda/>`_ that launches the manipulator drivers and the armer driver.

To configure details such as if armer is interfacing with a simulation or a physical arm, a `yaml config file <https://github.com/qcr/armer_panda/>`_ should be created. 