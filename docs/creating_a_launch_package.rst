Creating a Hardware Package 
====================================
Armer drivers provide a high level interface to command a manipulator. Armer relies on the manipulator's ROS driver implementation to communicate with the low level hardware.

For convenience a hardware package should be created to launch the drivers for an arm or arm group. This consists of:

* A launch file which launches the manipulator drivers and then the Armer drivers
* A yaml configuration file which sets run time parameters such as the model being launched and the type of backend 

Examples of a hardware packages can be seen with the `armer_panda <https://github.com/qcr/armer_panda/>`_ or the `armer_ur <https://github.com/qcr/armer_ur/>`_ packages.

.. 
    TODO There is probably a step before this which is what are the requirements for the hardware driers. For example what ROS controller style, naming convention, urdf etc are needed.

Creating a Launch File
-----------------------
A ROS launch file should be created which launches the manipulator's drivers as well as the Armer driver. This is achieved by combining the contents of the ROS driver launch file with the `armer.launch <https://github.com/qcr/armer/blob/master/launch/armer.launch/>`_ file found in the Armer driver package.

#. Find, download and install the manufacturer's drivers. This can generally be done by googling ``{ROBOT_MODEL} ROS drivers`` and following their readme instructions. 

#. After the drivers have been installed to the relevant workspace, read the ROS drivers documentation to find the launch file that starts the driver nodes. 

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
-----------------------

Create a config for launching the physical robot backend using the following as an example template:

    .. code-block:: yaml

        robots:
        - name: arm 
            model: roboticstoolbox.models.{ROBOTIC_TOOLBOX_MODEL_NAME}
            .. 
                TODO what if the model doesnt exist in the toolbox?
        backend: 
        type: roboticstoolbox.backends.ROS.ROS

    The key parameters to define here are:
        * name: namespace to be used
        * model: robotics toolbox robot model to use
        * backend type: 
            * roboticstoolbox.backends.Swift.Swift will launch the Swift simulator
            * armer.backends.ROS.ROS to use a physical system

    Optional parameters can also be set:

    ..
        TODO What are the defaults if these aren't applied

    .. list-table:: Configuration parameters
        :widths: 25 25 50
        :header-rows: 1

        *   - Field Name
            - Description
            - Example
        *   - robots: joint_state_topic 
            - topic to listen to joint states on 
            - `"/joint_states"`
        *   - robots: joint_velocity_topic
            - topic to listen to velocity on
            - `"/joint_group_velocity_controller/joint_velocity"` 
        *   - robots: origin 
            - Set a different origin for the robot
            - `[-1, 0, 0, 0, 0, 0]`
        *   - robots: gripper
            - Specify the end effector link
            - `"tool0"` 
        *   - logging: frequency
            - Sets the frequency of logging 
            - `false` 

    Certain arms (such as the UR3) have multiple end effectors so specifying the link must be done by adding a "gripper" field to the robots section with the link name as a string.

Package Structure
--------------------

For ease of deployment and use, the launch and config file should be packaged into a ROS package.
 
#. The name of the package should be ``armer_{ROBOT_MODEL}``. 

#. The launch file should be placed in the ``armer_{ROBOT_MODEL}/launch`` directory. 

#. Relevant config files should be placed in ``armer_{ROBOT_MODEL}/cfg``. 

#. The package can be created by running ``catkin_create_pkg armer_{ROBOT_MODEL}/`` in the directory above ``armer_{ROBOT_MODEL}``.

#. The package can be built by running ``catkin_make`` in the main workspace directory

#. If all has gone well, the ROS drivers and the Armer drivers should be started after running:

    .. code-block:: sh

        roslaunch armer_{ROBOT_MODEL} robot_bringup.launch

.. note::

    For further details on creating a ROS package see http://wiki.ros.org/ROS/Tutorials/CreatingPackage.

..
    This helper is also good github.com/qcr/qcr_templates