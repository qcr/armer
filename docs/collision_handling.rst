Collision Handling
=====================

Armer implements collision handling features that extend the capabilities provided by the roboticstoolbox-python and spatialgeometry libraries. 
Upon launch of Armer, the default robot_description (i.e., robot model) is parsed, with each collision shape associated with each link captured in a collision dictionary for checking.
If a valid collision is found in every step of the Armer driver, a preemption occurs to safely stop the robot.

The following high-level features are now available:
- Cython build of the global checking method for fast execution
- A slice of target links (named slice links) that can be configured by user input for limiting the number of links to be evaluated per time step.
- K-Dimensional (KD) tree creation is conducted each step to: (a) capture the distance between all links to target slice links; and (b) find the closest 4 links per each slice link to confirm for collision. This ensures that the collision checking process remains stable even with an increase in the number of shapes in the scene. 
- Multiple arm instances per Armer driver (two tested) can be conducted with the global collision checking method, where each individual arm checks and preempts upon collision in a common scene.
- A new recover_move service to move the arm back to a pre-defined safe joint state along its recently conducted trajectory. This ensures the arm comes free of collision preemption for fast recovery.
- The dynamic addition/removal of collision objects (simple primitives, such as spheres, cuboids, or cylinders) during runtime for custom collision object setup
- Each added collision object is interactive through RVIZ for easy positioning/orienting based on the use case.
- A collision scene (list of added dynamic collision shapes) can be saved to a config yaml file for easy reload on a new run 

URDF Parsed Collision Objects
------------------------------

Armer supports loading in a robot_description; and as such, will parse and extract collision shapes to conduct collision checking.

Note that, these collision shapes will not be interactive and are a part of the robot's collision tree. This is especially useful when considering
the end-effector's collision shapes (in a custom configuration for example). Please refer to https://wiki.ros.org/urdf/Tutorials for an indepth explanation on
what a URDF is and how to create a custom version.

Adding Collision Objects
------------------------

Collision primitives (i.e., spheres, cylinders, and cuboids) can be added in at runtime via the ``/arm/add_collision_object`` service. 

This can be called via Python:

.. code-block:: python

    from armer_msgs.srv import AddCollisionObject
    import rospy

    rospy.init_node('armer_example', disable_signals=True)
    service = rospy.ServiceProxy('/arm/add_collision_object', AddCollisionObject)
    # Create a pose object and populate
    pose = Pose()
    pose.position.x = 0.3
    pose.position.y = 0
    pose.position.z = 0.15
    pose.orientation.w = 0
    pose.orientation.x = 1
    pose.orientation.y = 0
    pose.orientation.z = 0
    
    # Configure Shape Properties
    name="test_object"
    shape_type='cylinder'
    length=0.3
    radius=0.05

    # Send via service
    service(name, shape_type, length, radius, pose)  

Or via Bash:

.. code-block:: bash

    rosservice call /arm/add_collision_object "name: 'test_object'
    type: 'cylinder'
    radius: 0.05
    length: 0.3
    scale_x: 0.0
    scale_y: 0.0
    scale_z: 0.0
    pose:
        position: {x: 0.3, y: 0.0, z: 0.15}
        orientation: {x: 1.0, y: 0.0, z: 0.0, w: 0.0}
    overwrite: false" 

Getting Collision Object Details
----------------------------------

Added collision object details (specifically the ``name``, its ``type``, and the ``x,y,z translation``) can be requested through the ``/arm/get_collision_objects`` service.

This can be called via Bash:

.. code-block:: bash

    rosservice call /arm/get_collision_objects "{}"

which will return a list of information. As an example, you may see the following:

.. code-block:: bash

    rosservice call /arm/get_collision_objects "{}"
    objects: 
        - 'block_1 -> (shape: cylinder, pose (x,y,z): [0.43330675 0.         0.16029578])'
        - 'ceil -> (shape: cuboid, pose (x,y,z): [0.90051776 0.         0.55649769])'
        - 'floor -> (shape: cuboid, pose (x,y,z): [3.72529030e-09 0.00000000e+00 2.11520195e-02]

Interacting with Collision Objects
----------------------------------

With RVIZ open, simply add the ``RobotModel`` and the ``InteractiveMarkers`` modules to view currently configured shapes.

Removing Collision Objects
--------------------------

Added objects (excluding those that are a part of the robot_description) can be done through the ``/arm/remove_collision_object`` service.

This can be called via Python:

.. code-block:: python

    from armer_msgs.srv import RemoveCollisionObject
    import rospy

    rospy.init_node('armer_example', disable_signals=True)
    service = rospy.ServiceProxy('/arm/remove_collision_object', RemoveCollisionObject)
    
    # Set the name of the object to be removed
    name="test_object"

    # Send via service
    service(name)  

Or via Bash:

.. code-block:: bash

    rosservice call /arm/remove_collision_object "name: 'test_object'"

NOTE: that the name must be within the existing dictionary of collision objects added. If this name is not in this dictionary, then you will receive an error.

Saving a Collision Scene
------------------------

Loading a Collision Scene
-------------------------


