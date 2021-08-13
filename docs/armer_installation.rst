Armer Installation
========================================

.. note::
    Requires ROS Noetic preinstalled

Quickstart Installation Script
--------------------------------

Copy and paste the following script into a terminal to create a new catkin workspace and install the Armer drivers to it. Note this script will also add the workspace to be sourced every time a bash terminal is opened.

For a breakdown of the script flick to 
`Installation Breakdown <armer_installation.html#installation_breakdown/>`_.

    .. code-block:: bash
        
        mkdir -p ~/armer_ws/src && cd ~/armer_ws/src &&
        git clone https://github.com/qcr/armer.git && git clone https://github.com/qcr/armer_msgs &&
        cd .. && rosdep install --from-paths src --ignore-src -r -y &&
        catkin_make &&
        echo "source ~/armer_ws/devel/setup.bash" >> ~/.bashrc &&


If the script has completed with no errors, continue on to `Working With a New Arm <working_with_a_new_arm/>`_.

Installation Breakdown
--------------------------------

1. Creates a catkin workspace.

    .. code-block:: sh

        mkdir -p ~/armer_ws/src && cd ~/armer_ws/src

2. Clones this repository and https://github.com/qcr/armer_msgs into the armer_ws/src folder.

    .. code-block:: sh

        git clone https://github.com/qcr/armer.git && git clone https://github.com/qcr/armer_msgs

3. Installs the required dependencies.

    .. code-block:: sh
        
        cd .. && rosdep install --from-paths src --ignore-src -r -y


4. Builds the downloaded packages.

    .. code-block:: sh

        catkin_make 


5. Sources the workspace automatically.

    .. code-block:: sh

        echo "source ~/armer_ws/devel/setup.bash" >> ~/.bashrc