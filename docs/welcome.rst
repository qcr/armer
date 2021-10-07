Introduction
========================================

.. image:: https://github.com/qcr/armer/wiki/armer_example.gif
  :alt: Armer example gif

.. image:: https://github.com/qcr/armer/wiki/codelink.png
  :alt: Armer's code on github
  :target: https://github.com/qcr/armer

.. image:: https://github.com/qcr/armer/wiki/blockdiagram.png
  :alt: Armer simplified block diagram

Armer aims to provide an interface layer between the hardware drivers of a robotic arm giving the user control in several ways:

* `Joint velocity control <set_joint_velocity.html>`_
* `Joint position control <set_joint_position.html>`_
* `End effector Cartesian velocity control <set_Cartesian_velocity.html>`_
* `End effector Cartesian position control <set_Cartesian_pose.html>`_
* `End effector Cartesian velocity control with guarded motion <guarded_motion.html>`_

In addition to a multiple control method layer, Armer is designed to be a compatability layer allowing the user to use the same code across different robotic platforms. Armer supports control for physical and simulated arms giving users the ability to develop even without access to a physical manipulator.

Below is a gif of 3 different simulated arms moving with the same Cartesian velocity commands.

.. image:: https://github.com/qcr/armer/wiki/same_code_example.gif
  :alt: Armer same code example gif
  
Requirements
--------------

Several ROS action servers, topics and services are set up by Armer to enable this functionality. A summary of these can be found `here <API.html>`_.

The driver is built off the `Python Robotics Toolbox (RTB) <https://qcr.github.io/code/robotics-toolbox-python>`_ and requires a URDF loaded RTB model to calculate the required movement kinematics, RTB comes with browser based simulator `Swift <https://qcr.github.io/code/swift/>`_ which Armer uses as an out of the box simulator.

Due to these supporting packages using Armer with a manipulator will require several requirements:

Software requirements
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* `Python <https://www.python.org/>`_ >= 3.6
* `ROS Noetic <http://wiki.ros.org/noetic>`_
* `Robotics Toolbox for Python <https://pypi.org/project/roboticstoolbox-python/>`_
* `QCR repos <https://qcr.github.io/armer/add_qcr_repos.html>`_

Robot specific requirements
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* ROS drivers with joint velocity controllers
* Robotics Toolbox model

To get started, check out the `Armer Installation <armer_installation.html>`_ page!