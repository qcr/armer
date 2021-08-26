Add QCR Robotic Vision Repo
====================================
``rosdep`` provides a useful mechanism for retrieving system dependencies required to build ROS packages from source. However, the list of packages maintained by the official ROS index is rather limited.

As part of developing our Robotic Vision libraries we have frequently discovered dependencies that were simply not available via rosdep. To resolve this issue we have created our own index compliment that provided by ROS.

Import the GPG Key using the following command

.. code-block:: bash

    sudo -E apt-key adv --keyserver hkp://keyserver.ubuntu.com --recv-key 5B76C9B0

Add the Robotic Vision repository to the apt sources list directory

.. code-block:: bash

    sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] https://packages.qcr.ai $(lsb_release -sc) main" > /etc/apt/sources.list.d/qcr-latest.list'

Update your packages list

.. code-block:: bash

    sudo apt update

To use rosdep to install depencencies needed for building the Robotic Vision libraries from source, please run the following command:

.. code-block:: bash

    sudo sh -c 'sed -i "1iyaml https:\/\/raw.githubusercontent.com\/qcr\/package_lists\/master\/$(lsb_release -sc)\/sources.yaml" /etc/ros/rosdep/sources.list.d/20-default.list'
