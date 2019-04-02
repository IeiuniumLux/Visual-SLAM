cmake_minimum_required(VERSION 2.8.3)
project(cereal_port)
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)


catkin_package(
INCLUDE_DIRS include
LIBRARIES CerealPort
CATKIN_DEPENDS roscpp)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(CerealPort src/CerealPort.cpp)


install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)


