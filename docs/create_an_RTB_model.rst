Creating a Robotics Toolbox model
==================================
The Armer drivers loads a URDF model into the Python Robotic Toolbox framework to process the kinematics and other movement related calculations. 

.. note:: 
    For more information on the Robotics Toolbox see their `documentation page <https://petercorke.github.io/robotics-toolbox-python/index.html>`_

The Robotics Toolbox contains a reasonable `selection of arms <https://petercorke.github.io/robotics-toolbox-python/_modules/index.html>`_  so always check first before setting out to creating your own.

#. Find the xarco URDF model of your robot 
#. Fill in the following template 

    .. code-block:: Python

        import numpy as np
        from roboticstoolbox.robot.ERobot import ERobot


        class ROBOT_MODEL_NAME(ERobot):
            """
            Class that imports a ROBOT_MODEL_NAME URDF model

            Defined joint configurations are:

            - qz, zero joint angle configuration, 'L' shaped configuration
            - qr, vertical 'READY' configuration

            """
            def __init__(self):

                links, name = self.URDF_read(
                    "{LOCATION_OF_URDF_XARCO}")

                super().__init__(
                        links,
                        name=name,
                        manufacturer='{MANUFACTURER_NAME}'
                    )

                #These arrays should contain an element for each joint
                self.addconfiguration(
                    "qz", np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0]))
                self.addconfiguration(
                    "qr", np.array([0, 45, 60, 0, 0, 0, 0, 0, 0, 0]) * np.pi/180)

        if __name__ == '__main__':   # pragma nocover

            robot = ROBOT_MODEL_NAME()
            print(robot)

    To verify the model has been created correctly, the script can be run and will print out details of the model

#. Save the template to your local RTB models folder

#. Your model can now be called by RTB and can be loaded from a config file for Armer to use