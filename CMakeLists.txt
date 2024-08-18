cmake_minimum_required(VERSION 3.5)
project(pro_bridge)

set(ROS_VERSION $ENV{ROS_VERSION})

if (ROS_VERSION STREQUAL "1")
  find_package(catkin REQUIRED COMPONENTS
    rospy
  )
elseif (ROS_VERSION STREQUAL "2")
  find_package(ament_cmake REQUIRED)
  find_package(ament_cmake_python REQUIRED)
  find_package(rclpy REQUIRED)

  install(
  DIRECTORY config ${PROJECT_NAME}
  DESTINATION share/${PROJECT_NAME}
  )
  install(PROGRAMS
    ${PROJECT_NAME}/bridge.py
    DESTINATION lib/${PROJECT_NAME}
  )
  ament_package()
endif()