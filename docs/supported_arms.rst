Supported Arms
===============
Armer provides a high level interface to command a manipulator and relies on the manipulator's ROS driver implementation to communicate with the low level hardware.

For convenience, hardware packages consisting of a config file for setting `run parameters <creating_a_hardware_package.html#creating-a-launch-file>`_ and a `launch file <creating_a_hardware_package.html#creating-a-launch-file>`_ that launches Armer and the target manipulator's drivers are bundled together.


Arms with Armer hardware packages
----------------------------------
Currently Armer driver has packages for a few arms. If your arm model has a hardware package, control should be a fairly plug and play experience. (An experience we are still working on so please let us know if it isn't.)

* Franka Panda: `https://github.com/qcr/armer_panda <https://github.com/qcr/armer_panda>`_

* Universal Robot UR3: `https://github.com/qcr/armer_ur <https://github.com/qcr/armer_ur>`_

* Universal Robot UR5: `https://github.com/qcr/armer_ur <https://github.com/qcr/armer_ur>`_

* Universal Robot UR10: `https://github.com/qcr/armer_ur <https://github.com/qcr/armer_ur>`_

* ABB IRB6700: `https://github.com/qcr/armer_abb <https://github.com/qcr/armer_abb>`_


Arms with Robotic Toolbox Models
----------------------------------

Armer uses the kinematic model from the Robotics Toolbox (RTB) to calculate motion for an arm. If your arm exists as an RTB, it is farily simple to point Armer to launch the model from the config file. See `Creating a Config File <creating_a_hardware_package.html#creating-a-launch-file>`_. 

To check if your arm model is supported by RTB out of the box, see the list of modules here for if they have the URDF suffix: https://petercorke.github.io/robotics-toolbox-python/_modules/index.html 

It is recommended to follow the `Creating a Hardware Package <creating_a_hardware_package.html#creating-a-hardware-package>`_ tutorial to help setup a hardware package for your arm if one does not exist.

Unsupported Arms
-----------------------------
In the case of your manipulator of interest not having an Armer hardware package or an RTB model, a URDF model can be loaded into an RTB model so procedure with creating a hardware package can continue normally from there. See `Creating a Robotics Toolbox model <create_an_RTB_model.html#creating-a-robotics-toolbox-model/>`_

