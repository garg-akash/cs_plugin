include_directories(${PROJECT_SOURCE_DIR}/include)

add_library(DaemonPlugin SHARED Data.cpp MyXmlRpcServerMethods.cpp webServer.cpp xmlrpcserver.cpp interpolation.cpp) 
target_link_libraries(DaemonPlugin csrtsi) 

add_executable(inter interpolation.cpp)
target_link_libraries(inter ${catkin_LIBRARIES} csrtsi)
add_dependencies(inter 
${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
