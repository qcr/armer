Introduction
========================================

.. image:: https://github.com/qcr/armer/wiki/armer_example.gif
  :alt: Armer example gif

.. image:: https://github.com/qcr/armer/wiki/codelink.png
  :alt: Armer's code on github
  :target: https://github.com/qcr/armer

.. image:: https://github.com/qcr/armer/wiki/PetersDiagram.png
  :alt: Armer simplified block diagram

Armer is a Python3 layer that extends the capability of a ROS-enabled robot arm. 

Armer handles requests for advanced types of motion such as: 

* Straight-line movement to a task-space pose
* Straight-line movement to a named joint configuration
* Task-space velocity with guarding compliant motion etc. 
  
Armer itself is a collection of ROS topics, services and action servers. It communicates with a ROS-enabled arm to execute the motion requests, communicating with the robot using ROS on the user’s behalf.  

Armer is implemented in Python 3 and uses the Robotics Toolbox for Python to perform all kinematic and differential kinematics functions.

Armer provides rich control modes such as:

* `Joint velocity control <set_joint_velocity.html>`_
* `Joint position control <set_joint_position.html>`_
* `End effector Cartesian velocity control <set_Cartesian_velocity.html>`_
* `End effector Cartesian position control <set_Cartesian_pose.html>`_
* `End effector Cartesian velocity control with guarded motion <guarded_motion.html>`_

Using Armer, the user is able to seamlessly switch between these modes of control.

These control modes can be used to control real or simulated robots using Swift. This gives users the ability to develop even without access to a physical manipulator.

The control instructions can be used across different robotic platforms allowing users to control robots with the same instructions.

Below is a gif of 3 different simulated arms moving with the same Cartesian velocity control commands.

.. image:: https://github.com/qcr/armer/wiki/same_code_example.gif
  :alt: Armer same code example gif

Armer is available for several popular robots including:

* Franka Panda: `https://github.com/qcr/armer_panda <https://github.com/qcr/armer_panda>`_

* Universal Robot UR3: `https://github.com/qcr/armer_ur <https://github.com/qcr/armer_ur>`_

* Universal Robot UR5: `https://github.com/qcr/armer_ur <https://github.com/qcr/armer_ur>`_

* Universal Robot UR10: `https://github.com/qcr/armer_ur <https://github.com/qcr/armer_ur>`_

* ABB IRB6700: `https://github.com/qcr/armer_abb <https://github.com/qcr/armer_abb>`_

Armer can be accessed primarily through Python3 scripts utilizing rospy to bridge Python code to the ROS framework.  

Requirements
--------------

Several ROS action servers, topics and services are set up by Armer to enable this functionality. A summary of these can be found `here <API.html>`_.

The driver is built on the `Robotics Toolbox (RTB) for Python <https://qcr.github.io/code/robotics-toolbox-python>`_ and requires a URDF loaded RTB model to calculate the required movement kinematics, RTB comes with browser based simulator `Swift <https://qcr.github.io/code/swift/>`_ which Armer uses as an out of the box simulator.

Due to these supporting packages using Armer with a manipulator will require several requirements:

Software requirements
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* `Python <https://www.python.org/>`_ >= 3.6
* `ROS Noetic <http://wiki.ros.org/noetic>`_
* `Robotics Toolbox for Python <https://pypi.org/project/roboticstoolbox-python/>`_
* `Add QCR repos to apt <https://qcr.github.io/armer/add_qcr_repos.html>`_

Robot specific requirements
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* ROS drivers with joint velocity controllers
* Robotics Toolbox model

To get started, check out the `Armer Installation <armer_installation.html>`_ page!