cmake_minimum_required(VERSION 3.5)
project(my_daemon)

add_subdirectory(src)

add_executable(HelloWorld main.cpp)
target_include_directories(HelloWorld PUBLIC include)
target_link_libraries(HelloWorld DaemonPlugin xmlrpc_server_abyss++ xmlrpc_server++ pthread)
