#include <iostream>
#include "xmlrpcserver.h"
#include "Data.h"
#include "webServer.h"
#include "ros/ros.h"

using namespace std;

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "main_node");
    ros::NodeHandle nh;

    Data data(& nh);

    cout << data.getMessage() << endl;
    XmlRpcServer myServer(4444, &data);
    WebServer webServer(5555, &data);

    if (myServer.setupSocket() != 0)
    {
        return -1;
    }

    myServer.regester_xml_method();

    pthread_t thread_id;
    if (pthread_create(&thread_id, NULL, webServer.run, &webServer))
    {
        cerr << "Couldn't create pthread" << endl;
        return EXIT_FAILURE;
    }

    if (myServer.setupServer() != 0)
    {
        cout << "cannot set RPC Server" << endl;
        return -1;
    }

    ros::spin();
    return 0;
}