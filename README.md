Armer Driver
============

[![QUT Centre for Robotics Open Source](https://github.com/qcr/qcr.github.io/raw/master/misc/badge.svg)](https://qcr.github.io)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Build Status](https://github.com/qcr/armer/workflows/Build/badge.svg?branch=master)](https://github.com/qcr/armer/actions?query=workflow%3ABuild)
[![Language grade: Python](https://img.shields.io/lgtm/grade/python/g/qcr/armer.svg?logo=lgtm&logoWidth=18)](https://lgtm.com/projects/g/qcr/armer/context:python)
[![codecov](https://codecov.io/gh/qcr/armer/branch/master/graph/badge.svg?token=GERHR5QTOF)](https://codecov.io/gh/qcr/armer)

![image](https://github.com/qcr/armer/wiki/armer_example.gif)

[![Armer documentation can be found here](https://github.com/qcr/armer/wiki/doclink.png)](https://qcr.github.io/armer/welcome.html)

![image](https://github.com/qcr/armer/wiki/blockdiagram.png)

Armer aims to provide an interface layer between the hardware drivers of
a robotic arm giving the user control in several ways:

-   [Joint velocity
    control](https://qcr.github.io/armer/set_joint_velocity.html)
-   [Joint position
    control](https://qcr.github.io/armer/set_joint_position.html)
-   [End effector Cartesian velocity
    control](https://qcr.github.io/armer/set_joint_position.html)
-   [End effector Cartesian position
    control](https://qcr.github.io/armer/set_cartesian_position.html)

In addition to a multiple control method layer, Armer is designed to
be a compatability layer allowing the user to use the same code
across different robotic platforms. Armer supports control for physical
and simulated arms giving users the ability to develop even without
access to a physical manipulator.

Below is a gif of 3 different simulated arms moving with the same cartesian velocity commands.

![image](https://github.com/qcr/armer/wiki/same_code_example.gif)

Requirements
------------

Several ROS action servers, topics and services are set up by Armer 
to enable this functionality. A summary of these can be found
[here](https://qcr.github.io/armer/API.html).

Armer is built on the [Python Robotics Toolbox
(RTB)](https://qcr.github.io/code/robotics-toolbox-python) and requires
a URDF loaded RTB model to calculate the required movement kinematics,
RTB comes with browser based simulator
[Swift](https://qcr.github.io/code/swift/) which Armer uses as an out of
the box simulator.

Due to these supporting packages using Armer with a manipulator will
require several requirements:

### Software requirements

-   [Python](https://www.python.org/) \>= 3.6
-   [ROS Noetic](http://wiki.ros.org/noetic)
-   [Robotics Toolbox for
    Python](https://pypi.org/project/roboticstoolbox-python/)
-   [QCR repos](https://qcr.github.io/armer/add_qcr_repos.html)
- Chrome browser (for using Swift sim)

### Robot specific requirements

-   ROS drivers with joint velocity controllers
-   Robotics Toolbox model

Installation
------------

Copy and paste the following code snippet into a terminal to create a
new catkin workspace and install Armer to it. Note this
script will also add the workspace to be sourced every time a bash
terminal is opened.

> ```sh
> sudo apt install python3-pip 
> pip install git+https://github.com/petercorke/spatialmath-python.git
> pip install git+https://github.com/jhavl/spatialgeometry.git
> pip install git+https://github.com/jhavl/swift.git
> pip install git+https://github.com/petercorke/robotics-toolbox-python.git@v1.0.2
> mkdir -p ~/armer_ws/src && cd ~/armer_ws/src 
> git clone https://github.com/qcr/armer.git && git clone https://github.com/qcr/armer_msgs 
> cd .. && rosdep install --from-paths src --ignore-src -r -y 
> catkin_make 
> echo "source ~/armer_ws/devel/setup.bash" >> ~/.bashrc 
> source ~/armer_ws/devel/setup.bash
> echo "Installation complete!"
> ```

Supported Arms
---------------
Armer relies on the manipulator's ROS driver to communicate with the low level hardware so the the ROS drivers must be started along side Armer. ***NOTE: the below packages are required for control of a real robot - see below for simulation usage instructions***

Currently Armer driver has packages that launches Armer and the target manipulator's drivers are bundled together. If your arm model has a hardware package, control should be a fairly plug and play experience. (An experience we are still working on so please let us know if it isn't.). Below are the github pages to arms with hardware packages. Install directions can be found on their respective pages.

* Franka Panda: [https://github.com/qcr/armer_panda](https://github.com/qcr/armer_panda)

* Universal Robot UR3: [https://github.com/qcr/armer_ur](https://github.com/qcr/armer_ur)

* Universal Robot UR5: [https://github.com/qcr/armer_ur](https://github.com/qcr/armer_ur)

* Universal Robot UR10: [https://github.com/qcr/armer_ur](https://github.com/qcr/armer_ur)

* ABB IRB6700: [https://github.com/qcr/armer_abb](https://github.com/qcr/armer_abb)

For more information on setting up manipulators not listed here see the Armer documentation, [Supported Arms](https://qcr.github.io/armer/supported_arms.html).

Usage
-------

The Armer interface can be launched with the following command for simulation:

> ``` {.sourceCode .bash}
> roslaunch armer armer.launch config:={PATH_TO_CONFIG_YAML_FILE}
> ```

Alternatively, the Armer interface can be launched for a real robot using the following command (Note that this can also support simulation if you wish via the ***sim*** parameter):

> ``` {.sourceCode .bash}
> roslaunch armer_{ROBOT_MODEL} robot_bringup.launch config:={PATH_TO_CONFIG_YAML_FILE} sim:={true/false}
> ```

After launching, an arm can be controlled in several ways. Some quick tutorials can be referenced below:

-   [Joint velocity
    control](https://qcr.github.io/armer/set_joint_velocity.html)
-   [Joint position
    control](https://qcr.github.io/armer/set_joint_position.html)
-   [End effector Cartesian velocity
    control](https://qcr.github.io/armer/set_joint_position.html)
-   [End effector Cartesian position
    control](https://qcr.github.io/armer/set_Cartesian_position.html)
-   [End effector Cartesian velocity control with guarded motion](https://qcr.github.io/armer/guarded_motion.html)

For more information and examples see the [Armer
documentation](https://qcr.github.io/armer/)
