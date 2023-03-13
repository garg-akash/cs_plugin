#ifndef WEBSERVER_H
#define WEBSERVER_H

#include <stdio.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/sendfile.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <assert.h>
#include <unistd.h>
#include <string.h>
#include "Data.h"

using namespace std;
class WebServer
{
private:
    int port;
    Data *data;
    const string htmlStr = "<html> <body> <hr> <p> %s <//p> <//body> <//html> ";

public:
    WebServer(int _port, Data *_data);
    static void *run(void *webServer);
    static string &replace_all(string &str, const string &old_value, const string &new_value);
};

#endif // WEBSERVER_H