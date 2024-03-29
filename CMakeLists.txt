cmake_minimum_required(VERSION 3.0.2)
project(speaking_face)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslib
  std_msgs
  message_generation
)

# OpenGL
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(
  FILES
  ShapeConfig.msg
  FaceConfig.msg
  FaceParam.msg
  MouseTouch.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES speaking_face
 CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
)

###########
## Build ##
###########

# Add source files
file(GLOB_RECURSE SOURCE_FILES 
	src/*.cpp)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(face ${SOURCE_FILES})
target_link_libraries(face ${catkin_LIBRARIES} ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} -lglut) # explicitly ass -lglut to address awkward issue for building in conda env
add_dependencies(face ${catkin_EXPORTED_TARGETS})
