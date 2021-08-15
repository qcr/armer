Working With A New Arm
====================================

.. note::
    Make sure the arm's ROS drivers and the main Armer driver package are installed!

If a ``armer_{YOUR_ROBOT_MODEL}`` package is in the QCR repos, you're in luck! Simply install it and run 
    .. code-block:: bash
        
        roslaunch armer_ur robot_bringup.launch 

If your arm model doesn't have a hardware package in the QCR repos see:
    *  `Creating a Hardware Package <creating_a_hardware_package.html#creating-a-hardware-package>`_.
    
If you want to set custom parameters:
    * `Creating a Launch File <creating_a_hardware_package.html#creating-a-launch-file>`_
    * `Creating a Config File <creating_a_hardware_package.html#creating-a-launch-file>`_

If there is no Robotic Toolbox model available see:
    * `Creating a Robotics Toolbox model <create_an_RTB_model.html#creating-a-robotics-toolbox-model/>`_




