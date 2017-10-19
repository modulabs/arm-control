cmake_minimum_required(VERSION 2.8.3)
project(acrobot_control)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg cmake_modules message_generation)
find_package(Eigen REQUIRED)

## add_message_files(FILES MsgControl.msg)

## Generate added messages and services
## generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package(
LIBRARIES ${PROJECT_NAME}
  DEPENDS Boost Eigen)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

## Build talker and listener
include_directories(include
    ${catkin_INCLUDE_DIRS}
    ${Eigen_INCLUDE_DIRS})

add_executable(acrobot_control src/acrobot_control_PID.cpp)
target_link_libraries(acrobot_control ${catkin_LIBRARIES})
add_dependencies(acrobot_control acrobot_control_generate_messages_cpp)
