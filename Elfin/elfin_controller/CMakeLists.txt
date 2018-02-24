cmake_minimum_required(VERSION 2.8.3)
project(elfin_controller)

find_package(catkin REQUIRED COMPONENTS
  controller_interface
  control_msgs
  urdf
)

include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

catkin_package(
  CATKIN_DEPENDS
    controller_interface
    control_msgs
    urdf
  LIBRARIES ${PROJECT_NAME}
)

add_library(${PROJECT_NAME}
  src/Time_Delay_Controller.cpp
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

