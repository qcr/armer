.. icon-api documentation master file, created by
   sphinx-quickstart on Mon Mar  1 18:12:12 2021.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

Welcome to Armer Driver's documentation!
========================================

.. image:: https://github.com/qcr/armer/wiki/armer_example.gif
  :alt: Armer example gif


The Armer driver provides a simple mechanism for building high-level configurations to seamlessly control a broad range of manipulators under different actuation modes (simulation or physical).

Several ROS action servers, topics and services are set up by this package to enable this functionality. A summary of these can be found [here](#driver-component-summary). 

Additionally, the driver is built off the `Python Robotics Toolbox <https://qcr.github.io/code/robotics-toolbox-python>`_ and uses `Swift <https://qcr.github.io/code/swift/>`_ as a backend to simulate supported manipulators.

To get started, check out the `Armer Installation <armer_installation>`_ page!

.. toctree::
   :maxdepth: 2
   :caption: Getting Started:

   armer_installation   
   working_with_a_new_arm  

.. toctree::
   :maxdepth: 3
   :caption: Tutorials:
   
   creating_a_hardware_package
   create_an_RTB_model
   set_joint_velocity
   set_cartesian_velocity
   set_joint_position


.. toctree::
   :maxdepth: 3
   :caption: Code:

   API
   modules
   


 