Armer Installation
========================================

Requirements
-------------

* `Python <https://www.python.org/>`_ >= 3.6
* `ROS Noetic <http://wiki.ros.org/noetic>`_
* `Robotics Toolbox for Python <https://pypi.org/project/roboticstoolbox-python/>`_
* `QCR repos <https://qcr.github.io/armer/add_qcr_repos.html>`_

.. note::

        Please refer to the requirements.txt (in the armer package) for easy dependency install

Robot Specific Requirements
-------------

* ROS drivers with joint group velocity controllers (ros_control)
* Robotics Toolbox model or URDF (loaded as a robot_description parameter)

Quickstart Installation (Native Linux)
--------------------------------

Copy and paste the following code snippet into a terminal to create a new catkin workspace and install the Armer drivers to it. Note this script will also add the workspace to be sourced every time a bash terminal is opened.

.. code-block:: bash
        
       # Install pip 
       sudo apt install python3-pip
       
       # Make the workspace and clone armer and armer_msgs packages
       mkdir -p ~/armer_ws/src && cd ~/armer_ws/src 
       git clone https://github.com/qcr/armer.git && git clone https://github.com/qcr/armer_msgs 
       
       # Install all required packages
       pip install -r ~/armer_ws/src/armer/requirements.txt
       cd .. && rosdep init && rosdep update && rosdep install --from-paths src --ignore-src -r -y 
       
       # Make and source the workspace 
       catkin_make 
       echo "source ~/armer_ws/devel/setup.bash" >> ~/.bashrc 
       source ~/armer_ws/devel/setup.bash
       echo "Installation complete!"


.. note::

    If the ``rosdep`` command is failing to find dependencies, make sure the QCR Robotic Vision repos have been added. See `Adding QCR Robotic Vision Repos <add_qcr_repos.html>`_
        
RoboStack Installation (Linux, Mac, Windows)
--------------------------------

To enable easy use of ROS on these operating systems, it is recommended to use `RoboStack <https://robostack.github.io/>`_; note that ROS 1 (noetic) is recommended at this stage. Please ensure you have `mamba <https://mamba.readthedocs.io/en/latest/installation.html>`_ installed before proceeding. Please follow all required steps for the RoboStack install (as per their instructions) to enable the smoothest setup on your particular OS.

.. note::
    This is still in progress. See here for updates.

.. code-block:: bash

        # --- Mamba Environment Setup --- #
        # Create and activate a new robostack (ros-env) environment
        mamba create -n ros-env ros-noetic-desktop python=3.9 -c robostack-staging -c conda-forge --no-channel-priority --override-channels
        mamba activate ros-env

        # Install some compiler packages
        mamba install compilers cmake pkg-config make ninja

        # FOR WINDOWS: Install the Visual Studio command prompt - if you use Visual Studio 2022
        mamba install vs2022_win-64

        # --- ARMer Setup --- #
        # Make the armer workspace and clone in armer and armer_msgs packages
        # FOR LINUX/MACOS
        mkdir -p ~/armer_ws/src && cd ~/armer_ws/src 
        # FOR WINDOWS: Assumes you are in the home folder
        mkdir armer_ws\src && cd armer_ws\src
        # Clone in armer and armer_msgs
        git clone https://github.com/qcr/armer.git && git clone https://github.com/qcr/armer_msgs 
        # Install all required packages (into ros-env) - from current directory
        # FOR LINUX/MACOS
        pip install -r armer/requirements.txt
        # FOR WINDOWS
        pip install -r armer\requirements.txt
        # Enter armer_ws folder and run rosdep commands
        cd .. && rosdep init && rosdep update && rosdep install --from-paths src --ignore-src -r -y 

        # Make and source the workspace (including environment)
        catkin_make 

        # --- Default Activation of Environment --- #
        # FOR LINUX 
        echo "mamba activate ros-env" >> ~/.bashrc

        # FOR MACOS
        echo "mamba activate ros-env" >> ~/.bash_profile

        # --- Workspace Source --- #
        source ~/armer_ws/devel/setup.bash
        

If the script has completed with no errors, continue on to `Supported Arms <supported_arms.html>`_.
