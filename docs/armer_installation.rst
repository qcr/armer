Armer Installation
========================================

Requirements
-------------

* `Python <https://www.python.org/>`_ >= 3.6
* `ROS Noetic <http://wiki.ros.org/noetic>`_
* `Robotics Toolbox for Python <https://pypi.org/project/roboticstoolbox-python/>`_

.. note::

    You will probably need to add the QCR Robotic Vision repos so ``rosdep`` won't fail. See `Adding QCR Robotic Vision Repos <add_qcr_repos.html>`_

Quickstart Installation
--------------------------------

Copy and paste the following code snippet into a terminal to create a new catkin workspace and install the Armer drivers to it. Note this script will also add the workspace to be sourced every time a bash terminal is opened.

    .. code-block:: bash
        
        sudo apt install python3-pip 
        mkdir -p ~/armer_ws/src && cd ~/armer_ws/src 
        git clone https://github.com/qcr/armer.git && git clone https://github.com/qcr/armer_msgs 
        cd .. && rosdep install --from-paths src --ignore-src -r -y 
        catkin_make 
        echo "source ~/armer_ws/devel/setup.bash" >> ~/.bashrc 
        source ~/armer_ws/devel/setup.bash
        echo "Installation complete!"
        
If the script has completed with no errors, continue on to `Working With a New Arm <working_with_a_new_arm.html/>`_.