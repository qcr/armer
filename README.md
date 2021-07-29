# Armer Driver
[![QUT Centre for Robotics Open Source](https://github.com/qcr/qcr.github.io/raw/master/misc/badge.svg)](https://qcr.github.io)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Build Status](https://github.com/suddrey-qut/armer/workflows/Build/badge.svg?branch=master)](https://github.com/suddrey-qut/armer/actions?query=workflow%3ABuild)
[![Language grade: Python](https://img.shields.io/lgtm/grade/python/g/suddrey-qut/armer.svg?logo=lgtm&logoWidth=18)](https://lgtm.com/projects/g/suddrey-qut/armer/context:python)
[![Coverage](https://codecov.io/gh/suddrey-qut/armer/branch/master/graph/badge.svg)](https://codecov.io/gh/suddrey-qut/armer)

![armer_example](https://github.com/qcr/armer/wiki/armer_example.gif)

## Introduction
The Armer driver provides a simple mechanism for building high-level configurations to seamlessly control a broad range of manipulators under different actuation modes (simulation or physical).

Several ROS action servers, topics and services are set up by this package to enable this functionality. A summary of these can be found [here](#driver-component-summary). 

Additionally, the driver is built off the [Python Robotics Toolbox](https://qcr.github.io/code/robotics-toolbox-python/) and uses [Swift](https://qcr.github.io/code/swift/) as a backend to simulate supported manipulators.

## Installation
*Requires ROS Noetic preinstalled*

1. Create a catkin workspace.
```sh
mkdir -p ~/armer_ws/src && cd ~/armer_ws/src
```

2. Clone this repository and https://github.com/qcr/armer_msgs into the armer_ws/src folder.
```sh
git clone https://github.com/qcr/armer.git && git clone https://github.com/qcr/armer_msgs
```
3. Install the required dependencies.
```sh
cd .. && rosdep install --from-paths src --ignore-src -r -y
```

4. Build the packages.
```sh
catkin_make 
```

5. Source the workspace.
```sh
source devel/setup.sh
```
To make this automatically occur in any terminal that is launched, run the follwing line.
```sh
echo "source ~/armer_ws/devel/setup.bash" >> ~/.bashrc
```

6. Run the following code block to start the simulation. By default the configuration for the Panda model sim will be launched.
```sh
roslaunch armer armer.launch
```


## Usage

The driver can be configured use with different arms, a simulation or a physical enviroment. To define these settings and allow for easy reuse, settings can be saved and loaded from a YAML file in the cfg folder. 

A simple example configuration for the Franka-Emika Panda simulation can be seen below:
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
* backend type: roboticstoolbox.backends.Swift.Swift will launch the Swift simulator. Use armer.backends.ROS.ROS to use a physical system

Other parameters can also be set:

| Field Name | Description | Example |
| :--------| :--------| :--------|
| robots: joint_state_topic | topic to listen to joint states on | `"/joint_states"` |
| robots: joint_velocity_topic | topic to listen to velocity on | `"/joint_group_velocity_controller/joint_velocity"` |
| robots: origin | Set a different origin for the robot | [-1, 0, 0, 0, 0, 0] |
| robots: gripper | Specify the end effector link | `"tool0"` | 
| logging: frequency | Sets the frequency of logging | false |


Certain arms (such as the UR3) have multiple end effectors so specifying the link must be done by adding a "gripper" field to the robots section with the link name as a string.

To launch a driver instance with a preset config, the config parameter is added to the roslaunch command with the name of the desired YAML config.

The following example shows how driver instance can be launched with a [saved Panda sim config](https://github.com/qcr/armer/blob/master/cfg/panda_sim.yaml):
```
roslaunch armer armer.launch config:=/path/to/cfg/panda_sim.yaml
```

## Examples

Examples of interfacing with the driver can be found in the [examples folder](https://github.com/qcr/armer/tree/master/examples). 

An example[(panda_example.py)](https://github.com/qcr/armer/blob/master/examples/panda_example.py) for interacting with a Panda sim can be run from the workspace main directory via the following command after roslaunching the Panda model sim config.

```
python3 armer/examples/panda_example.py
```

Alternatively, an example[(ur5_example.py)](https://github.com/qcr/armer/blob/master/examples/ur5_example.py) for interacting with a UR5 sim can be run from the workspace main directory via the following command after roslaunching the UR5 model sim config.

```
python3 armer/examples/ur5_example.py
```

## Code Examples

### Move a manipulator to a specified pose

Both the MoveToPose action server and the ServoToPose action server will move a manipulator to a specific pose. 

To communicate with the driver, a ROS action client must be created to communicate with the driver.

To connect to the MoveToPose action server, initialize an action client with the following code.
```
pose_cli = actionlib.SimpleActionClient('/arm/cartesian/pose', MoveToPoseAction)
pose_cli.wait_for_server()
```

In this example the pose of a simulated Panda arm will be set. The MoveToPose action server uses the PoseStamped message type from the geometry messages package. 

```
# Create an empty PoseStamped message
target = PoseStamped()
```
To request a pose the frame_id of the header field should be filled out with the target robot's base link.
```
target.header.frame_id = 'panda_link0'
```
 The desired pose's position and orientation should be filled out in the corresponding fields.
```
target.pose.position.x = 0.307
target.pose.position.y = 0.400
target.pose.position.z = 0.490

target.pose.orientation.x = -1.00
target.pose.orientation.y =  0.00
target.pose.orientation.z =  0.00
target.pose.orientation.w =  0.00
```

Create a goal from the target pose.
```
goal = MoveToPoseGoal()
goal.pose_stamped=target
```

Set the movement speed if desired. The field types can be seen in the action message declaration ([armer_msgs/MoveToPose.action](https://github.com/qcr/armer_msgs/blob/main/action/MoveToPose.action)).
```
goal.speed=1
```

The desired pose is then sent to the action server as a goal and waits for it to finish. The server will return true if it has been successful and false if not.
```
pose_cli.send_goal(goal)
pose_cli.wait_for_result()
```
### Move a manipulator to a specified pose with obstacle avoidance

The key difference between the two pose action servers is ServoToPose will move to the specified pose with real time object avoidance.

To connect to the ServoToPose action server, initialize an action client with the following code.
```
servo_cli = actionlib.SimpleActionClient('/arm/cartesian/servo_pose', ServoToPoseAction)
servo_cli.wait_for_server()
```
Again, the pose of a simulated Panda arm will be set. The ServoToPose action server also uses the PoseStamped message type from the geometry messages package. 

```
# # Create a target pose
target = PoseStamped()
target.header.frame_id = 'panda_link0'

# Populate with target position/orientation 
target.pose.position.x = 0.307
target.pose.position.y = 0.400
target.pose.position.z = 0.490

target.pose.orientation.x = -1.00
target.pose.orientation.y =  0.00
target.pose.orientation.z =  0.00
target.pose.orientation.w =  0.00

# Create goal from target pose
goal = ServoToPoseGoal()
goal.pose_stamped=target
```
As with MoveToPose, parameters can be specified, here the threshold and gain can be set as follows. Additionally, the expected types can be seen in the action message declaration ([armer_msgs/ServoToPose.action](https://github.com/qcr/armer_msgs/blob/main/action/ServoToPose.action)).
```
goal.threshold=0.5
goal.gain=0.5
```
ServoToPose will also return a success boolean.
```
# Send goal and wait for it to finish
servo_cli.send_goal(goal)
servo_cli.wait_for_result()
servo_cli.get_result()

```
As can be seen, the ServoToPose process to have the driver move a manipulator to a specific pose is very similar between MoveToPose.

### Move a manipulator via specific joint settings
If the user has a desired joint configuration, the joints of a manipulator can manually be set with the the MoveToJointPoseAction server.

As always, a connection with the server must be created via an action client.

```
servo_cli = actionlib.SimpleActionClient('/arm/joint/pose', MoveToJointPoseAction)
servo_cli.wait_for_server()
```
The desired joint positions should be sent through as an array. Other field types can be seen in the action message declaration ([armer_msgs/MoveToJointPose.action](https://github.com/qcr/armer_msgs/blob/main/action/MoveToJointPose.action)). 
```
# Desired joint settings in radians
desired_joints=[0, -pi/2, pi/4, 0, 0, 0 0]
# Create goal from target pose
goal = MoveToJointPoseGoal()
goal.joints=desired_joints

# Send goal and wait for it to finish
servo_cli.send_goal(goal)
servo_cli.wait_for_result()
servo_cli.get_result()
```

### Move a manipulator's end effector w.r.t the target frame's ID
A target robot can also be manipulated by publishing directly to the cartesian velocity topic. This topic uses the message type [TwistStamped](https://docs.ros.org/api/geometry_msgs/html/msg/Twist.html).

Set up the publisher pointed to the velocity topic.
```
vel_pub = rospy.Publisher('/arm/cartesian/velocity', TwistStamped, queue_size=1)
```
Create an empty message.
```
ts = TwistStamped()
```
Populate the message with desired variables.
```
ts.twist.linear.z = 0.1
```
Publish the message to the velocity topic.
```
vel_pub.publish(ts)
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

- **/arm/home** ([std_srvs/Empty](http://docs.ros.org/noetic/api/std_srvs/html/srv/Empty.html))
Moves the robot back to its initial ready pose.

- **/arm/recover** ([std_srvs/Empty](http://docs.ros.org/noetic/api/std_srvs/html/srv/Empty.html))
Recovers from collision or limit violation error states that will put the robot into a non-operable state.

- **/arm/stop** ([std_srvs/Empty](http://docs.ros.org/noetic/api/std_srvs/html/srv/Empty.html))
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