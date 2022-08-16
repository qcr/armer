cmake_minimum_required(VERSION 3.5)
project(armer)

if($ENV{ROS_VERSION} STREQUAL "2")
  find_package(ament_cmake REQUIRED)

else()
  find_package(catkin REQUIRED)

  catkin_python_setup()

  catkin_package()

  catkin_install_python(PROGRAMS
    scripts/armer
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

  install(DIRECTORY cfg/ examples/ launch/ 
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

endif()

