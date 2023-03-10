#ifndef XMLRPCSERVER_H
#define XMLRPCSERVER_H
 
#include "xmlrpc-c/base.hpp"
#include "xmlrpc-c/registry.hpp"
#include "xmlrpc-c/server_abyss.hpp"
#include "Data.h"
 
class XmlRpcServer
{
 
private:

 
private:
    xmlrpc_c::registry serviceRegistry;
    xmlrpc_c::serverAbyss* AbyssServer;
    int port;
    int socket_fd;
    Data* data;
 
public:
    void regester_xml_method();
    int setupServer();
    int setupSocket();
 
public:
    XmlRpcServer(int _port,Data* _data);
};
 
#endif // XMLRPCSERVER_H