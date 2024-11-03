cmake_minimum_required(VERSION 3.0.2)
project(slambot_sdk)

## Compile as C++17, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

## Declare a Catkin package
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES encoder_motor
  CATKIN_DEPENDS roscpp std_msgs
)

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(encoder_motor
  src/encoder_motor.cpp
)

## Declare a C++ executable
add_executable(encoder_motor_node
  src/main.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(encoder_motor_node
  encoder_motor
  ${catkin_LIBRARIES}
)

## Mark executables and/or libraries for installation
install(TARGETS encoder_motor encoder_motor_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark header files for installation
install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
