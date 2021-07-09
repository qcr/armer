# Armer
[![QUT Centre for Robotics Open Source](https://github.com/qcr/qcr.github.io/raw/master/misc/badge.svg)](https://qcr.github.io)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Build Status](https://github.com/suddrey-qut/armer/workflows/Build/badge.svg?branch=master)](https://github.com/suddrey-qut/armer/actions?query=workflow%3ABuild)
[![Language grade: Python](https://img.shields.io/lgtm/grade/python/g/suddrey-qut/armer.svg?logo=lgtm&logoWidth=18)](https://lgtm.com/projects/g/suddrey-qut/armer/context:python)
[![Coverage](https://codecov.io/gh/suddrey-qut/armer/branch/master/graph/badge.svg)](https://codecov.io/gh/suddrey-qut/armer)

## Overview
This project provides a simple unified mechanism for building high-level interface configurations for seamlessly controlling a broad range of manipulators under different actuation modes.

More concretly, the Armer driver provides a mechanism for advertising high-level topics, services and action_servers for controlling a manipulator, which when called, not only actuate the manipulator based on the provided command, but also hot-swaps in the relevant low-level controllers for that actuation mode. 


## Installation
Requires ROS noetic preinstalled

1. Create a catkin workspace 
```sh
mkdir ~/armer_ws && cd ~/armer_ws
catkin_make
```
2. Clone this repository and https://github.com/qcr/armer_msgs into the armer_ws/src folder
```sh
cd src && git clone https://github.com/qcr/armer.git && git clone https://github.com/qcr/armer_msgs 
```
3. Install the required dependencies
```sh
cd .. && rosdep install --from-paths src --ignore-src -r -y 
```
4. Build the packages by running 
```sh
catkin_make 
```
5. Source the workspace
```sh
source devel/setup.sh
```
To make this automatically occur in any terminal that is launched, run 
```sh
echo "source ~/armer_ws/devel/setup.bash" >> ~/.bashrc

```
6. Run 
```sh
roslaunch armer armer.launch
```
 to start the simulation. By default the Panda model sim will be launched

## Usage

Configurations are specified using the YAML file format and should be placed in the cfg folder. 

An example configuration for the Franka-Emika Panda simulation can be seen below:
```
robots:
  - name: arm 
    model: roboticstoolbox.models.Panda 
backend: 
  type: roboticstoolbox.backends.Swift.Swift 
```
The key parameters to define here are:
* name: namespace to be used
* model: robotics toolbox robot model to use
* backend type: roboticstoolbox.backends.Swift.Swift will launch the Swift simulator. Use armer.backends.ROS.ROS to use a physical model

Other parameters can also be set:
| Field Name | Description | Example |
| --------| --------| --------|
| robots: joint_state_topic | topic to listen to joint states on | `"/joint_states"` |
| robots: joint_velocity_topic | topic to listen to velocity on | `"/joint_velocity_node_controller/joint_velocity"` |
| robots: origin | Set a different origin for the robot | [-1, 0, 0, 0, 0, 0] |
| robots: gripper | Specify the end effector link | `"tool0"` | 
| logging: frequency | Sets the frequency of logging | false |


Certain arms (such as the UR3) have multiple end effectors so specifying the link should be done by adding a "gripper" field to the robots section with the link name as a string.

### Panda in Swift
```sh
roslaunch armer armer.launch config:=panda_sim
```

### UR5 in Swift
```sh
roslaunch armer armer.launch config:=ur5_sim
```

## Examples
An example for Panda can be run from the workspace main directory via the following command after roslaunching the Panda model sim

```sh
python3 armer/examples/panda_example.py
```

An example for UR5 can be run from the workspace main directory via the following command after roslaunching the UR5 model sim

```sh
python3 armer/examples/ur5_example.py
```
## Driver Component Summary

### Subscribed Topics

- **/arm/cartesian/velocity** ([geometry_msgs/TwistStamped](https://docs.ros.org/api/geometry_msgs/html/msg/Twist.html))
Moves the end-effector in cartesian space w.r.t. the target frame_id (base frame if no frame_id is set).

- **/arm/joint/velocity** ([armer_msgs/JointVelocity](https://github.com/qcr/armer_msgs/blob/main/msg/JointVelocity.html))
Moves the joints of the manipulator at the requested velocity.

### Published Topics

- **/arm/state**  ([armer_msgs/ManipulatorState](https://github.com/qcr/armer_msgs/blob/main/msg/ManipulatorState.msg))
Provides information on the current state of the manipulator including the pose of the end-effector w.r.t. to the base link, whether the manipulator is experiencing a cartesian contact and collision as a bit-wised error state flag.

### Services

- **/arm/home** ([std_srvs/Empty](http://docs.ros.org/jade/api/std_srvs/html/srv/Empty.html))
Moves the robot back to its initial ready pose.

- **/arm/recover** ([std_srvs/Empty](http://docs.ros.org/jade/api/std_srvs/html/srv/Empty.html))
Recovers from collision or limit violation error states that will put the robot into a non-operable state.

- **/arm/stop** ([std_srvs/Empty](http://docs.ros.org/jade/api/std_srvs/html/srv/Empty.html))
Stops the current motion of the current.

- **/arm/get_named_poses** ([armer_msgs/GetNamesList](https://github.com/qcr/armer_msgs/blob/main/srv/GetNamesList.srv))
Gets a list of currently stored named poses (includes both moveit and driver stored named poses).

- **/arm/set_named_pose** ([armer_msgs/SetNamedPose](https://github.com/qcr/armer_msgs/blob/main/srv/SetNamedPose.srv))
Saves the current joint configuration of the robot with the provided pose name.

- **/arm/remove_named_pose** ([armer_msgs/RemoveNamedPose](https://github.com/qcr/armer_msgs/blob/main/srv/RemoveNamedPose.srv))
Removes the joint configuration of the provided pose name.

- **/arm/set_cartesian_impedance** ([armer_msgs/SetCartesianImpedance](https://github.com/qcr/armer_msgs/blob/main/srv/SetCartesianImpedance.srv)
Adjusts the impedance of the end-effector position in cartesian space.

- **/arm/add_named_pose_config** ([armer_msgs/SetNamedPoseConfig](https://github.com/qcr/armer_msgs/blob/main/srv/SetNamedPoseConfig.srv))
Instructs the driver to load named poses stored in the indicated config file.

- **/arm/get_named_pose_configs** ([armer_msgs/GetNamedPoseConfigs](https://github.com/qcr/armer_msgs/blob/main/srv/GetNamedPoseConfigs.srv))
Gets the list of config files to check for named poses.

- **/arm/remove_named_pose_config** ([armer_msgs/SetNamedPoseConfig](https://github.com/qcr/armer_msgs/blob/main/srv/SetNamedPoseConfig.srv))
Instructs the driver to remove named poses stored in the indicated config file.


### Action API

- **/arm/cartesian/pose** ([armer_msgs/MoveToPose.action](https://github.com/qcr/armer_msgs/blob/main/action/MoveToPose.action))
Moves the end-effector to the requested goal pose w.r.t. the indicated frame id.

- **/arm/cartesian/servo_pose** ([armer_msgs/ServoToPose.action](https://github.com/qcr/armer_msgs/blob/main/action/ServoToPose.action))
Servos the end-effector to the requested goal pose with real time object avoidance.

- **/arm/joint/named** ([armer_msgs/MoveToNamedPose.action](https://github.com/qcr/armer_msgs/blob/main/action/MoveToNamedPose.action))
Moves the end-effector to a pre-defined joint configuration.

- **/arm/joint/pose** ([armer_msgs/MoveToJointPoseAction](https://github.com/qcr/armer_msgs/blob/main/action/MoveToJointPose.action))
Moves the joints of the robot to the indicated positions (radians).