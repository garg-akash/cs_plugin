include_directories(${PROJECT_SOURCE_DIR}/include)
add_library(DaemonPlugin SHARED Data.cpp MyXmlRpcServerMethods.cpp webServer.cpp xmlrpcserver.cpp)
target_link_libraries(DaemonPlugin xmlrpc)
