Creating a Hardware Package 
====================================
Armer drivers provide a high level interface to command a manipulator. Armer relies on the manipulator's ROS driver implementation to communicate with the low level hardware.

For convenience, a hardware package should be created to launch the drivers for an arm or arm group. This consists of:

* A `ROS launch file <http://wiki.ros.org/roslaunch/XML>`_ which launches the manipulator drivers and then the Armer drivers
* A `yaml configuration file <http://wiki.ros.org/YAML%20Overview>`_ which sets run time parameters such as the model being launched and the type of backend 

These files are packaged togther as a hardware package. To skip to folder structure and naming convention, see the `Putting it All Together <creating_a_hardware_package.html#putting-it-all-together>`_ section of this guide.

Examples of hardware packages can be seen with the `armer_panda <https://github.com/qcr/armer_panda/>`_ or the `armer_ur <https://github.com/qcr/armer_ur/>`_ packages.

.. note::
    At this point in time, only manipulators with joint velocity controllers are supported by Armer. If an arm does not support this, it cannot be used with the Armer drivers.

Creating a Launch File
---------------------------

This is a ROS launch file which starts the manipulator's drivers as well as the Armer driver. This is achieved by combining the contents of the ROS driver launch file with the `armer.launch <https://github.com/qcr/armer/blob/master/launch/armer.launch/>`_ file found in the Armer driver package.

#. If they have not already been installed, install the ROS drivers for the manipulator. This can generally be done by googling ``{ROBOT_MODEL} ROS drivers`` and following their readme instructions. 

#. After the drivers have been installed to the relevant workspace, read the ROS driver's documentation to find the launch file that starts the driver. 

    Copy and paste the manipulator driver's launch file contents into the template below and name the file ``robot_bringup.launch``:

    .. code-block:: xml

        <?xml version="1.0" ?>
        <launch>
            <arg name="config"/>
            <arg name="sim" default="false" />

            <!-- Physical Robot -->
            <group unless="$(arg sim)">   
                
                <!-- Manipulator driver -->  
                <!--INCLUDE OR COPY THE CONTENTS OF THE MANIPULATOR'S ROS DRIVER LAUNCH FILE HERE -->  

                <!-- Launch armer driver -->
                <include file="$(find armer)/launch/armer.launch">
                    <arg name="config" value="$(arg config)" />
                </include>
            </group>

            <!-- Simulated Robot -->
            <group if="$(arg sim)">   
                <include file="$(find armer)/launch/armer.launch">
                    <arg name="config" value="$(arg config)" />
                </include>
            </group>
        </launch>


Creating a Configuration File
---------------------------------

To configure parameters such as if armer is interfacing with a simulation or a physical arm, a yaml config file should be created. 

Armer uses the Robotics Toolbox to perform calculations and this config file will point to the Robotics toolbox ERobot object so movement can be processed. If the manipulator does not have an existing RTB model or if unsure, see `Creating a Robotics Toolbox model <create_an_RTB_model.html#creating-a-robotics-toolbox-model/>`_.

Create a config for launching the physical robot backend using the following as an example template and saved as ``{ROBOT_MODEL}_{BACKEND_MODE}.yaml`` where backend mode is `"real"` for a physical robot config and `"sim"` for a Swift sim robot:

    .. code-block:: yaml

        robots:
        - name: {DESIRED_ROBOT_NAMESPACE}
            model: roboticstoolbox.models.{ROBOTIC_TOOLBOX_MODEL_NAME}
        backend: 
            type: {DESIRED_BACKEND_TYPE}

    Most parameters can be left as defaults, but these are the essential parameters that should be changed for basic functionality.

    .. list-table:: Key configuration parameters
        :widths: 25 25 25 25
        :header-rows: 1

        *   - Field Name
            - Description
            - Example 
            - Default
        *   - name 
            - namespace to be used
            - `"my_cool_robot"`
            - ``None``
        *   - model 
            - robotics toolbox model to use
            - roboticstoolbox.models.Panda
            - ``None``
        *   - backend 
            - backend to be used
            - roboticstoolbox.backends.Swift.Swift
            - ``None``

    The two current options for backend are: 
            * ``roboticstoolbox.backends.Swift.Swift`` (Swift simulation robot)
            * ``armer.backends.ROS.ROS`` (Physical robot)

    Multiple robots can be launched at a time and parameters for each individual instance can be set under the corresponding namespace. For example: 

    .. code-block:: yaml

        robots:
        - name: panda
            model: roboticstoolbox.models.Panda
            origin: [0, 0, 0, 0, 0, -1] 
        - name: ur5
            model: roboticstoolbox.models.UR5
            origin: [-1, 0, 0, 0, 0, 0] 
        backend:
        - type: roboticstoolbox.backends.Swift.Swift

    In this example, a Panda and a UR5 arm are being launched with different origin settings. The options for each different robot section is signaled with the ``-`` symbol before the name parameter.

    The following parameters are available for setting in multi or single robot operations.

            
    .. list-table:: Robot parameters
        :widths: 20 20 10 50
        :header-rows: 1

        *   - Field Name
            - Description
            - Example 
            - Default
        *   - joint_state_topic 
            - topic to listen to joint states on 
            - `"/my_joint_states"`
            - `"/joint_states"`
        *   - joint_velocity_topic
            - topic to listen to velocity on
            - `"/my_controller/joint_velocity"` 
            - `"/joint_group_velocity_controller/command"`
        *   - origin 
            - set a different origin for the robot
            - `[-1, 0, 0, 0, 0, 0]`
            - `[0, 0, 0, 0, 0, 0]`
        *   - gripper
            - specify the end effector link
            - `"tool0"` 
            - ``None``
        *   - logging: frequency
            - sets the frequency of logging 
            - ``false`` 
            - ``None``

    
    Certain arms (such as the UR3) have multiple end effectors so specifying the link must be done by adding a "gripper" field to the robots section with the link name as a string.

Putting it All Together
------------------------------

For ease of deployment and use, the launch and config file should be packaged into a ROS package. The overall file structure is as follows:

.. code-block:: bash

    armer_{ROBOT_MODEL}/
        ├─ launch/
        │   ├─ robot_bringup.launch
        ├─ cfg/
        │   ├─ {ROBOT_MODEL}_{BACKEND_MODE}.yaml


 
#. The name of the package should be ``armer_{ROBOT_MODEL}``. 

#. The launch file should be placed in the ``armer_{ROBOT_MODEL}/launch`` directory. 

#. Relevant config files should be placed in ``armer_{ROBOT_MODEL}/cfg``. 

#. The package can be created by running ``catkin_create_pkg armer_{ROBOT_MODEL}/`` in the directory above ``armer_{ROBOT_MODEL}``.

#. The package can be built by running ``catkin_make`` in the main workspace directory

#. If all has gone well, the ROS drivers and the Armer drivers should be started after running:

    .. code-block:: bash

        roslaunch armer_{ROBOT_MODEL} robot_bringup.launch

.. note::

    For further details on creating a ROS package see http://wiki.ros.org/ROS/Tutorials/CreatingPackage.

..
    This helper is also good github.com/qcr/qcr_templates https://github.com/qcr/code_templates?
