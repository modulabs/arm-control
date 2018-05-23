cmake_minimum_required(VERSION 2.8.3)
project(arm_controllers)

find_package(catkin REQUIRED COMPONENTS
  angles
  controller_interface
  control_msgs
  geometry_msgs
  urdf
  kdl_parser
  realtime_tools
)

include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

catkin_package(
  CATKIN_DEPENDS
    angles
    controller_interface
    control_msgs
    geometry_msgs
    urdf
    kdl_parser
    realtime_tools
  LIBRARIES ${PROJECT_NAME}
)

add_library(${PROJECT_NAME}
  src/time_delay_controller.cpp
  src/sliding_mode_controller.cpp
  src/adaptive_variable_impedance_controller.cpp
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

