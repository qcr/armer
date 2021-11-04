Creating a Robotics Toolbox Model
==================================
Armer loads a URDF model into the Python Robotic Toolbox framework to process the kinematics and other movement related calculations. 

.. note:: 
    For more information on the Robotics Toolbox see their `documentation page <https://petercorke.github.io/robotics-toolbox-python/index.html>`_

The Robotics Toolbox contains a wide `selection of arms <https://petercorke.github.io/robotics-toolbox-python/_modules/index.html>`_ so always check first before setting out to creating your own.

Create the following file structure:


.. code-block:: bash

    ├── HARDWARE_PACKAGE_NAME
    │   ├── __init__.py
    │   └── robots
    │       ├── __init__.py
    │       └── ROBOT_NAME.py
    ├── cfg
    │   ├── ROBOT_NAME_real.yaml
    │   └── ROBOT_NAME_sim.yaml
    ├── CMakeLists.txt
    ├── data
    │   └── xacro
    │       ├── meshes
    │       └── robots
    │           ├── RELEVANT_XACRO_FILES.xacro
    ├── launch
    │   └── robot_bringup.launch
    ├── LICENSE
    ├── package.xml
    ├── README.md
    └── setup.py

The relevant meshes should be placed in ``data/xacro/meshes``

Relevant URDF files should be placed in ``data/xacro/robots``

Fill in the following template and paste into the Python file in ``armer_ROBOT_NAME/robots/ROBOT_NAME.py``

    .. code-block:: python

        import numpy as np
        from roboticstoolbox.robot.ERobot import ERobot
        from rospkg import RosPack

        class ROBOT_MODEL_NAME(ERobot):

            def __init__(self):

                links, name, urdf_string, urdf_filepath = self.URDF_read("robots/ROBOT_URDF.xacro", tld=RosPack().get_path('HARDWARE_PACKAGE_NAME') + '/data/xacro')
                    
                super().__init__(
                    links,
                    name=name,
                    urdf_string=urdf_string,
                    urdf_filepath=urdf_filepath,
                    manufacturer="ROBOT_MANUFACTURER_NAME", 
                    gripper_links=links[7]
                )
                
                self.addconfiguration(
                    "qr", np.array([0, 0, 0, 0, 0, 0])
                )

            if __name__ == "__main__":  # pragma nocover

                robot = ROBOT_MODEL_NAME()
                print(robot)

To verify the model has been created correctly, ``ROBOT_NAME.py`` can be run and will print out details of the model

In order for Armer and RTB to be able to find the custom robot model file, the ``__init__.py`` and ``setup.py`` files must also be configured correctly.

The contents ``__init__.py`` of should be as follows:

    .. code-block:: python

        from armer_ROBOT_NAME.models.ROBOT_NAME import ROBOT_NAME
        __all__ = [
            'ROBOT_NAME'
        ]

An example of ``setup.py`` can be found in `armer_mico <https://github.com/qcr/armer_mico>`_. Modify armer_mico instances to match the name of the custom package.

Your model can now be called by RTB and can be loaded from a config file for Armer to use.