Armer Installation
========================================

Requirements
-------------

* `Python <https://www.python.org/>`_ >= 3.6
* `ROS Noetic <http://wiki.ros.org/noetic>`_
* `Robotics Toolbox for Python <https://pypi.org/project/roboticstoolbox-python/>`_
* `QCR repos <https://qcr.github.io/armer/add_qcr_repos.html>`_

Quickstart Installation (Native Linux)
--------------------------------

Copy and paste the following code snippet into a terminal to create a new catkin workspace and install the Armer drivers to it. Note this script will also add the workspace to be sourced every time a bash terminal is opened.

.. code-block:: bash
        
        sudo apt install python3-pip 
        pip install git+https://github.com/petercorke/spatialmath-python.git
        pip install git+https://github.com/jhavl/spatialgeometry.git
        pip install git+https://github.com/jhavl/swift.git
        pip install git+https://github.com/petercorke/robotics-toolbox-python.git@v1.0.2
        mkdir -p ~/armer_ws/src && cd ~/armer_ws/src 
        git clone https://github.com/qcr/armer.git && git clone https://github.com/qcr/armer_msgs 
        cd .. && rosdep install --from-paths src --ignore-src -r -y 
        catkin_make 
        echo "source ~/armer_ws/devel/setup.bash" >> ~/.bashrc 
        source ~/armer_ws/devel/setup.bash
        echo "Installation complete!"


.. note::

    If the ``rosdep`` command is failing to find dependencies, make sure the QCR Robotic Vision repos have been added. See `Adding QCR Robotic Vision Repos <add_qcr_repos.html>`_
        
RoboStack Installation (Linux, Mac, Windows)
--------------------------------

Alternatively, Armer can be installed in a RoboStack environment. The supported ROS 1 distributions is currently Noetic ONLY.

.. note::
    This is still in progress. See here for updates.


If the script has completed with no errors, continue on to `Supported Arms <supported_arms.html>`_.
