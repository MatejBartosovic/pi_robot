cmake_minimum_required(VERSION 2.8.3)
project(pi_robot_driver)

 add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  pi_robot_description
  controller_manager
  diagnostic_updater
  dynamic_reconfigure
  hardware_interface
  roscpp
  sensor_msgs
  serial
  std_msgs
  joint_limits_interface
  urdf
)

generate_dynamic_reconfigure_options(
        cfg/dynamic_reconfigure.cfg
)

catkin_package()

include_directories(
  include
        ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node    src/Communication.cpp
                                       src/Messages.cpp
                                       src/Hardware.cpp
                                       src/Robot.cpp
                                       src/main.cpp)

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_gencfg)

target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
 )
