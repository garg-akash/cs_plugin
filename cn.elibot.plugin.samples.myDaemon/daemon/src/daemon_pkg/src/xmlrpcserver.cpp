#include "xmlrpcserver.h"
 
#include "xmlrpc-c/base.hpp"
#include "xmlrpc-c/registry.hpp"
#include "xmlrpc-c/server_abyss.h"
 
#include <sys/signal.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <cstring>
#include <iostream>
#include "MyXmlRpcServerMethods.h"
 
using namespace std;

XmlRpcServer::XmlRpcServer(int _port,Data* _data):socket_fd(-1){
    port = _port;
    data = _data;
} 

void XmlRpcServer::regester_xml_method()
{
    serviceRegistry.addMethod("get_message", new GetMessage(data));
    serviceRegistry.addMethod("set_message", new SetMessage(data));
    serviceRegistry.addMethod("get_moveit_traj", new GetMoveitTraj(data));
}
 
int XmlRpcServer::setupServer()
{
    AbyssServer = new xmlrpc_c::serverAbyss(xmlrpc_c::serverAbyss::constrOpt().registryP(&serviceRegistry).socketFd(socket_fd));
    cout<<"Xmlrpc server is running......"<<endl;
    AbyssServer->run();
    cout<<"Xmlrpc server stopped"<<endl;
    return 0;
}
 
int XmlRpcServer::setupSocket()
{
    int rc;
    int yes=1;
    struct sockaddr_in rm_add;
 
    socket_fd = socket(AF_INET, SOCK_STREAM, 0);
    if(-1 == socket_fd)
    {
        cout<<"Can not open server socket: "<<endl;
        return -1;
    }
 
    rc = setsockopt(socket_fd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(int));
    if(-1 == rc)
    {
        cout<<"Can not set socket options: "<<endl;
        return -1;
    }
 
    rm_add.sin_family = AF_INET;
    rm_add.sin_port = htons(port);
    rm_add.sin_addr.s_addr = INADDR_ANY;
 
    rc = bind(socket_fd, (struct sockaddr*)&rm_add, sizeof(struct sockaddr));
    if(-1 == rc)
    {
        cout<<"Can not bind to port "<<port<<endl;
        close(socket_fd);
        return -1;
    }
 
    return 0;
}