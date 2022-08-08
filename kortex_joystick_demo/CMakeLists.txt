cmake_minimum_required(VERSION 3.0.2)
project(kortex_joystick_demo)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  sensor_msgs
  joy
  kortex_driver
  control_msgs
  actionlib
  actionlib_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES kortex_joystick_demo
  CATKIN_DEPENDS geometry_msgs roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node
  src/main.cpp
  src/kortex_joystick_node.cpp
)

add_dependencies(${PROJECT_NAME}_node
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
