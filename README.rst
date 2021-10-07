Armer Driver
========================================

.. image:: https://github.com/qcr/armer/wiki/armer_example.gif
  :alt: Armer example gif

`Armer documentation can be found here <https://qcr.github.io/armer>`_

.. image:: https://github.com/qcr/armer/wiki/blockdiagram.png
  :alt: Armer example gif

Armer aims to provide an interface layer between the hardware drivers of a robotic arm giving the user control in several ways:

* `Joint velocity control <https://qcr.github.io/armer/set_joint_velocity.html>`_
* `Joint position control <https://qcr.github.io/armer/set_joint_position.html>`_
* `End effector Cartesian velocity control <https://qcr.github.io/armer/set_joint_position.html>`_
* `End effector Cartesian position control <https://qcr.github.io/armer/set_cartesian_position.html>`_

In addition to a multiple control method layer, Armer is designed to also be a compatability layer allowing the user to use the same code across different robotic platforms. Armer supports control for physical and simulated arms giving users the ability to develop even without access to a physical manipulator.

Requirements
--------------

Several ROS action servers, topics and services are set up by this package to enable this functionality. A summary of these can be found `here <https://qcr.github.io/armer/API.html>`_.

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

Installation
--------------------------------

Copy and paste the following code snippet into a terminal to create a new catkin workspace and install the Armer drivers to it. Note this script will also add the workspace to be sourced every time a bash terminal is opened.

    .. code-block:: bash
        
        sudo apt install python3-pip 
        mkdir -p ~/armer_ws/src && cd ~/armer_ws/src 
        git clone https://github.com/qcr/armer.git && git clone https://github.com/qcr/armer_msgs 
        cd .. && rosdep install --from-paths src --ignore-src -r -y 
        catkin_make 
        echo "source ~/armer_ws/devel/setup.bash" >> ~/.bashrc 
        source ~/armer_ws/devel/setup.bash
        echo "Installation complete!"
        
For more information and examples see the `Armer documentation <https://qcr.github.io/armer/>`_