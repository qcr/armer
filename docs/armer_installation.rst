Armer Installation
========================================

.. note::
    Requires ROS Noetic preinstalled

Quickstart Installation Script
--------------------------------

Copy and paste the following script into a terminal to create a new catkin workspace and install the Armer drivers to it. Note this script will also add the workspace to be sourced every time a bash terminal is opened.

    .. code-block:: bash
        
        sudo apt install python3-pip &&
        mkdir -p ~/armer_ws/src && cd ~/armer_ws/src &&
        git clone https://github.com/qcr/armer.git && git clone https://github.com/qcr/armer_msgs &&
        cd .. && rosdep install --from-paths src --ignore-src -r -y &&
        catkin_make &&
        echo "source ~/armer_ws/devel/setup.bash" >> ~/.bashrc &&
        source ~/armer_ws/devel/setup.bash
        
If the script has completed with no errors, continue on to `Working With a New Arm <working_with_a_new_arm/>`_.