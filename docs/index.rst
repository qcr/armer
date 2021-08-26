.. icon-api documentation master file, created by
   sphinx-quickstart on Mon Mar  1 18:12:12 2021.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

Welcome to Armer Driver's documentation!
========================================

.. image:: https://github.com/qcr/armer/wiki/armer_example.gif
  :alt: Armer example gif

`Armer's code on github: https://github.com/qcr/armer <https://github.com/qcr/armer>`_

.. image:: https://github.com/qcr/armer/wiki/blockdiagram.png
  :alt: Armer example gif

Armer aims to provide an interface layer between the hardware drivers of a robotic arm giving the user control in several ways:

* `Joint velocity control <set_joint_velocity.html>`_
* `Joint position control <set_joint_position.html>`_
* `End effector cartesian velocity control <set_cartesian_velocity.html>`_
* `End effector cartesian position control <set_cartesian_pose.html>`_

In addition to a multiple control method layer, Armer is designed to also be a compatability layer allowing the user to use the same code across different robotic platforms. Armer supports control for physical and simulated arms giving users the ability to develop even without access to a physical manipulator.

Requirements
--------------

Several ROS action servers, topics and services are set up by this package to enable this functionality. A summary of these can be found `here <API.html>`_.

The driver is built off the `Python Robotics Toolbox (RTB) <https://qcr.github.io/code/robotics-toolbox-python>`_ and requires a URDF loaded RTB model to calculate the required movement kinematics, RTB comes with browser based simulator `Swift <https://qcr.github.io/code/swift/>`_ which Armer uses as an out of the box simulator.

Due to these supporting packages using Armer with a manipulator will require several requirements:

Software requirements
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* `Python <https://www.python.org/>`_ >= 3.6
* `ROS Noetic <http://wiki.ros.org/noetic>`_
* `Robotics Toolbox for Python <https://pypi.org/project/roboticstoolbox-python/>`_

Robot specific requirements
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* Joint velocity ROS controllers
* Robotics Toolbox model

To get started, check out the `Armer Installation <armer_installation.html>`_ page!

.. toctree::
   :maxdepth: 2
   :caption: Getting Started:

   armer_installation
   supported_arms
   working_with_a_new_arm  

.. toctree::
   :maxdepth: 3
   :caption: Tutorials:
   
   creating_a_hardware_package
   create_an_RTB_model
   set_joint_velocity
   set_joint_position
   set_cartesian_velocity
   set_cartesian_position


.. toctree::
   :maxdepth: 3
   :caption: Code:

   API
   modules
   


 