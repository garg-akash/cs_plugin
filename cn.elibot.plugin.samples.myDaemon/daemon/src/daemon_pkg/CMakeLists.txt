cmake_minimum_required(VERSION 3.0.2)
project(daemon_pkg)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES daemon_pkg
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_subdirectory(src)

add_executable(HelloWorld main.cpp)
target_include_directories(HelloWorld PUBLIC include)
target_link_libraries(HelloWorld ${catkin_LIBRARIES} DaemonPlugin xmlrpc_server_abyss++ xmlrpc_server++ pthread)
