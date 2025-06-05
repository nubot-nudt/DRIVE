cmake_minimum_required(VERSION 3.0.2)
project(innfos_control_7r)

SET(INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
SET(LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lib/linux_x86_64)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib/linux_x86_64)
link_directories(${LIB_DIR})

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
add_compile_options(-std=c++11)
add_compile_options(-fPIC)
#add_compile_options(-DQT_NO_VERSION_TAGGING)
add_compile_options(-pthread)
add_compile_options(-DASIO_STANDALONE)
add_compile_options(-DASIO_HAS_STD_CHRONO)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  roscpp
  rospy
  std_msgs
  message_generation
  geometry_msgs
  sensor_msgs
  nubot_msgs
)

 add_message_files(
   FILES
   euler.msg
   eso.msg
   u.msg
   w.msg
 )
 generate_messages(
   DEPENDENCIES
   geometry_msgs
   std_msgs  # Or other packages containing msgs
 )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES innfos_control_7r
  CATKIN_DEPENDS roscpp rospy std_msgs
  #DEPENDS system_lib
  #message_runtime
)

include_directories(
	include
	${INCLUDE_DIR}
	${catkin_INCLUDE_DIRS}
	${INCLUDE_DIR}/serial/include	
)
add_library(ftsensorPerceptron SHARED ${CMAKE_CURRENT_SOURCE_DIR}/src/ftsensorPerceptron.cpp)
add_library(canthread SHARED ${CMAKE_CURRENT_SOURCE_DIR}/src/canthread.cpp)
 
add_executable(innfos_control_7r ${CMAKE_CURRENT_SOURCE_DIR}/src/innfos_control_7r.cpp )
target_link_libraries(innfos_control_7r ${catkin_LIBRARIES} ftsensorPerceptron ActuatorController canthread controlcan pthread)
