cmake_minimum_required(VERSION 2.8.3)
project(joystick_turtle)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  turtlesim
  message_generation
)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Cmd_feedback.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES DynamixelSimpleAPI.h
  CATKIN_DEPENDS roscpp std_msgs message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

#add_executable(tmp src/tmp.cpp)
#target_link_libraries(tmp ${catkin_LIBRARIES})
#add_dependencies(tmp joystick_turtle_generate_messages_cpp)

add_executable(turtlesim_teleop_joystick src/turtlesim_teleop_joystick.cpp src/SmartServoFramework-master/src/SerialPort.cpp src/SmartServoFramework-master/src/SerialPortLinux.cpp src/SmartServoFramework-master/src/SerialPortMacOS.cpp src/SmartServoFramework-master/src/SerialPortWindows.cpp src/SmartServoFramework-master/src/ControlTables.cpp src/SmartServoFramework-master/src/Utils.cpp src/SmartServoFramework-master/src/ControllerAPI.cpp src/SmartServoFramework-master/src/Servo.cpp src/SmartServoFramework-master/src/Dynamixel.cpp src/SmartServoFramework-master/src/DynamixelTools.cpp src/SmartServoFramework-master/src/DynamixelSimpleAPI.cpp src/SmartServoFramework-master/src/DynamixelController.cpp src/SmartServoFramework-master/src/ServoDynamixel.cpp src/SmartServoFramework-master/src/ServoAX.cpp src/SmartServoFramework-master/src/ServoEX.cpp src/SmartServoFramework-master/src/ServoMX.cpp src/SmartServoFramework-master/src/ServoXL.cpp src/SmartServoFramework-master/src/HerkuleX.cpp src/SmartServoFramework-master/src/HerkuleXTools.cpp src/SmartServoFramework-master/src/HerkuleXSimpleAPI.cpp src/SmartServoFramework-master/src/HerkuleXController.cpp src/SmartServoFramework-master/src/ServoHerkuleX.cpp src/SmartServoFramework-master/src/ServoDRS.cpp)
target_link_libraries(turtlesim_teleop_joystick ${catkin_LIBRARIES})
add_dependencies(turtlesim_teleop_joystick joystick_turtle_generate_messages_cpp)

add_executable(joystick_node src/joystick_node.cpp src/SmartServoFramework-master/src/SerialPort.cpp src/SmartServoFramework-master/src/SerialPortLinux.cpp src/SmartServoFramework-master/src/SerialPortMacOS.cpp src/SmartServoFramework-master/src/SerialPortWindows.cpp src/SmartServoFramework-master/src/ControlTables.cpp src/SmartServoFramework-master/src/Utils.cpp src/SmartServoFramework-master/src/ControllerAPI.cpp src/SmartServoFramework-master/src/Servo.cpp src/SmartServoFramework-master/src/Dynamixel.cpp src/SmartServoFramework-master/src/DynamixelTools.cpp src/SmartServoFramework-master/src/DynamixelSimpleAPI.cpp src/SmartServoFramework-master/src/DynamixelController.cpp src/SmartServoFramework-master/src/ServoDynamixel.cpp src/SmartServoFramework-master/src/ServoAX.cpp src/SmartServoFramework-master/src/ServoEX.cpp src/SmartServoFramework-master/src/ServoMX.cpp src/SmartServoFramework-master/src/ServoXL.cpp src/SmartServoFramework-master/src/HerkuleX.cpp src/SmartServoFramework-master/src/HerkuleXTools.cpp src/SmartServoFramework-master/src/HerkuleXSimpleAPI.cpp src/SmartServoFramework-master/src/HerkuleXController.cpp src/SmartServoFramework-master/src/ServoHerkuleX.cpp src/SmartServoFramework-master/src/ServoDRS.cpp)
target_link_libraries(joystick_node ${catkin_LIBRARIES})
add_dependencies(joystick_node joystick_turtle_generate_messages_cpp)

add_executable(joystick_turtlesim_server src/joystick_turtlesim_server.cpp)
target_link_libraries(joystick_turtlesim_server ${catkin_LIBRARIES})
add_dependencies(joystick_turtlesim_server joystick_turtle_generate_messages_cpp)

## Declare a cpp library
# add_library(DynamixelSimpleAPI
#   src/${PROJECT_NAME}/DynamixelSimpleAPI.cpp
# )

## Declare a cpp executable
# add_executable(joystick_turtle_node src/joystick_turtle_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(joystick_turtle_node joystick_turtle_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(joystick_turtle_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS joystick_turtle joystick_turtle_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_joystick_turtle.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
