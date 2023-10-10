Armer Driver
============

[![QUT Centre for Robotics Open Source](https://github.com/qcr/qcr.github.io/raw/master/misc/badge.svg)](https://qcr.github.io)
[![License](https://img.shields.io/github/license/qcr/armer)](./LICENSE.txt)
[![Build Status](https://github.com/qcr/armer/workflows/Build/badge.svg?branch=master)](https://github.com/qcr/armer/actions?query=workflow%3ABuild)
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
-   [ROS Noetic](http://wiki.ros.org/noetic) - [RoboStack](https://robostack.github.io/) can also be used (see below)
-   [Robotics Toolbox for
    Python](https://pypi.org/project/roboticstoolbox-python/)
-   [QCR repos](https://qcr.github.io/armer/add_qcr_repos.html)
- Chrome browser (for using Swift sim)
- [pointcloud_utils](https://github.com/qcr/pointcloud_utils) ROS package from QCR

### Robot specific requirements

-   ROS drivers with joint group velocity controllers (ros_control)
-   Robotics Toolbox model or URDF (loaded as a robot_description parameter)

Installation
------------
### Linux (Ubuntu 20.04)
Copy and paste the following code snippet into a terminal to create a
new catkin workspace and install Armer to it. Note this
script will also add the workspace to be sourced every time a bash
terminal is opened. If [RoboStack](https://robostack.github.io/) is preferred, please follow the steps in the next section

```bash
# Install pip 
sudo apt install python3-pip

# Make the workspace and clone armer, armer_msgs, and pointcloud_utils packages
mkdir -p ~/armer_ws/src && cd ~/armer_ws/src 
git clone https://github.com/qcr/armer.git && git clone https://github.com/qcr/armer_msgs && git clone https://github.com/qcr/pointcloud_utils.git

# Install all required packages
pip install -r ~/armer_ws/src/armer/requirements.txt
cd .. && rosdep install --from-paths src --ignore-src -r -y 

# Make and source the workspace 
catkin_make 
echo "source ~/armer_ws/devel/setup.bash" >> ~/.bashrc 
source ~/armer_ws/devel/setup.bash
echo "Installation complete!"
```

###  macOS and Windows (10/11)
To enable easy use of ROS on these operating systems, it is recommended to use [RoboStack](https://robostack.github.io/); note that ROS 1 (noetic) is recommended at this stage. Please ensure you have [mamba](https://mamba.readthedocs.io/en/latest/installation.html) installed before proceeding. Please follow all required steps for the [RoboStack](https://robostack.github.io/) install (as per their instructions) to enable the smoothest setup on your particular OS.
```bash
# --- Mamba Environment Setup --- #
# Create and activate a new robostack (ros-env) environment
mamba create -n ros-env ros-noetic-desktop python=3.9 -c robostack-staging -c conda-forge --no-channel-priority --override-channels
mamba activate ros-env

# Install some compiler packages
mamba install compilers cmake pkg-config make ninja

# FOR WINDOWS: Install the Visual Studio command prompt - if you use Visual Studio 2022
mamba install vs2022_win-64

# --- ARMer Setup --- #
# Make the armer workspace and clone in armer and armer_msgs packages
# FOR LINUX/MACOS
mkdir -p ~/armer_ws/src && cd ~/armer_ws/src 
# FOR WINDOWS: Assumes you are in the home folder
mkdir armer_ws\src && cd armer_ws\src
# Clone in armer, armer_msgs, and pointcloud_utils
git clone https://github.com/qcr/armer.git && git clone https://github.com/qcr/armer_msgs && git clone https://github.com/qcr/pointcloud_utils.git
# Install all required packages (into ros-env) - from current directory
# FOR LINUX/MACOS
pip install -r armer/requirements.txt
# FOR WINDOWS
pip install -r armer\requirements.txt
# Enter armer_ws folder and run rosdep commands
cd .. && rosdep init && rosdep update && rosdep install --from-paths src --ignore-src -r -y 

# Make and source the workspace (including environment)
catkin_make 

# --- Default Activation of Environment --- #
# FOR LINUX 
echo "mamba activate ros-env" >> ~/.bashrc

# FOR MACOS
echo "mamba activate ros-env" >> ~/.bash_profile

# --- Workspace Source --- #
source ~/armer_ws/devel/setup.bash
```

Supported Arms
---------------
Armer relies on the manipulator's ROS driver to communicate with the low level hardware so the the ROS drivers must be started along side Armer. ***NOTE: the below packages are required for control of a real robot - see below for simulation usage instructions***

Currently Armer driver has packages that launches Armer and the target manipulator's drivers are bundled together. If your arm model has a hardware package, control should be a fairly plug and play experience. (An experience we are still working on so please let us know if it isn't.). Below are the github pages to arms with hardware packages. Install directions can be found on their respective pages.

* [Available] Franka Panda: [https://github.com/qcr/armer_panda](https://github.com/qcr/armer_panda)

* [Available] Universal Robot (UR3/UR5/UR10/UR10e): [https://github.com/qcr/armer_ur](https://github.com/qcr/armer_ur)

* [In Progress] UFactory XArm (6): [https://github.com/qcr/armer_xarm](https://github.com/qcr/armer_xarm)

* [In Progress] ABB IRB6700: [https://github.com/qcr/armer_abb](https://github.com/qcr/armer_abb)

For more information on setting up manipulators not listed here see the Armer documentation, [Supported Arms](https://qcr.github.io/armer/supported_arms.html).

Usage
-------

The Armer interface can be launched with the following command for simulation (note, please replace USER with your own username):

> ``` {.sourceCode .bash}
> # Example is using the panda_sim.yaml. Note, please update the below path if the install directory is different
> roslaunch armer armer.launch config:=/home/$USER/armer_ws/src/armer/cfg/panda_sim.yaml
> ```

Alternatively, the Armer interface can be launched for a real robot using the following command (Note that this can also support simulation if you wish via the ***sim*** parameter):

> ``` {.sourceCode .bash}
> # Note this example launches the panda model in simulation mode (assumes you have this package cloned, see above)
> roslaunch armer_panda robot_bringup.launch sim:=true
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
