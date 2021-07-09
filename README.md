# Armer
[![QUT Centre for Robotics Open Source](https://github.com/qcr/qcr.github.io/raw/master/misc/badge.svg)](https://qcr.github.io)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Build Status](https://github.com/suddrey-qut/armer/workflows/Build/badge.svg?branch=master)](https://github.com/suddrey-qut/armer/actions?query=workflow%3ABuild)
[![Language grade: Python](https://img.shields.io/lgtm/grade/python/g/suddrey-qut/armer.svg?logo=lgtm&logoWidth=18)](https://lgtm.com/projects/g/suddrey-qut/armer/context:python)
[![Coverage](https://codecov.io/gh/suddrey-qut/armer/branch/master/graph/badge.svg)](https://codecov.io/gh/suddrey-qut/armer)


## Installation
Requires ROS noetic preinstalled

1. Create a catkin workspace (this example will assume the workspace location ~/armer_ws)
2. Clone this repository and https://github.com/qcr/armer_msgs into the armer_ws/src folder
3. In the main workspace folder (armer_ws) run 
```sh
rosdep install --from-paths src --ignore-src -r -y 
```
4. Build the packages by running 
```sh
catkin_make 
```
also in the main workspace folder (armer_ws)
5. Source the workspace
```sh
source devel/setup.sh
```
To make this automatically occur in any terminal that is launched, run 
```sh
source ~/armer_ws/devel/setup.bash
```
6. Run 
```sh
roslaunch armer armer.launch
```
 to start the simulation. By default the Panda model sim will be launched

## Usage

### Panda in Swift
```sh
roslaunch armer armer.launch config:=panda_sim
```

### UR5 in Swift
```sh
roslaunch armer armer.launch config:=ur5_sim
```
An example for Panda can be run from the workspace main directory via the following command after roslaunching the Panda model sim

```sh
python3 armer/examples/panda_example.py
```

An example for UR5 can be run from the workspace main directory via the following command after roslaunching the UR5 model sim

```sh
python3 armer/examples/ur5_example.py
```
