# Armer
[![QUT Centre for Robotics Open Source](https://github.com/qcr/qcr.github.io/raw/master/misc/badge.svg)](https://qcr.github.io)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Build Status](https://github.com/suddrey-qut/armer/workflows/Build/badge.svg?branch=master)](https://github.com/suddrey-qut/armer/actions?query=workflow%3ABuild)
[![Language grade: Python](https://img.shields.io/lgtm/grade/python/g/suddrey-qut/armer.svg?logo=lgtm&logoWidth=18)](https://lgtm.com/projects/g/suddrey-qut/armer/context:python)
[![Coverage](https://codecov.io/gh/suddrey-qut/armer/branch/master/graph/badge.svg)](https://codecov.io/gh/suddrey-qut/armer)



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
    joint_state_topic: /joint_states # topic to publish joint states to
    joint_velocity_topic:  /joint_velocity_node_controller # topic to publish joint velocity to
    origin: [0, 0, 0, 0, 0, 0] #default (xyzrpy) # desired origin of robot
backend: 
  type: roboticstoolbox.backends.Swift.Swift 
logging: 
  frequency: false 
```
The key parameters to define here are:
* model: robotics toolbox robot model to load
* backend type: roboticstoolbox.backends.Swift.Swift will launch the Swift simulator. Use armer.backends.ROS.ROS to use a physical model

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
