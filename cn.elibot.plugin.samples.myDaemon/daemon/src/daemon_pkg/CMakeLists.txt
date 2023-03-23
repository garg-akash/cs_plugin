cmake_minimum_required(VERSION 3.0.2)
project(daemon_pkg)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  cs_ros_msgs
  message_generation
  actionlib
  actionlib_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES daemon_pkg
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  CATKIN_DEPENDS actionlib actionlib_msgs
#  DEPENDS system_lib
#  DEPENDS cs_rtsi
)

include_directories(
  include
  /home/ak/cs_catkin_ws/src/cs_ros_pkg/cs_ros/include
  /home/ak/cs_catkin_ws/devel/include/
  ${catkin_INCLUDE_DIRS}
  # ${cs_ros_msgs_INCLUDE_DIRS}
)

add_subdirectory(src)

add_executable(HelloWorld main.cpp)
target_include_directories(HelloWorld PUBLIC include)
target_link_libraries(HelloWorld ${catkin_LIBRARIES} DaemonPlugin xmlrpc_server_abyss++ xmlrpc_server++ pthread)
add_dependencies(HelloWorld 
${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})