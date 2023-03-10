#include <iostream>
#include "webServer.h"

using namespace std;
WebServer::WebServer(int _port, Data *_data) : port(_port),
                                               data(_data)
{
}

string &WebServer::replace_all(string& str, const string& old_value, const string& new_value)
{
    while (true)
    {
        string::size_type pos(0);
        if ((pos = str.find(old_value)) != string::npos)
            str.replace(pos, old_value.length(), new_value);
        else
            break;
    }
    return str;
}

void *WebServer::run(void *server)
{
    WebServer *web = static_cast<WebServer *>(server);

    int sock;
    int connfd;
    struct sockaddr_in sever_address;
    bzero(&sever_address, sizeof(sever_address));
    sever_address.sin_family = PF_INET;
    sever_address.sin_addr.s_addr = htons(INADDR_ANY);
    sever_address.sin_port = htons(web->port);

    sock = socket(AF_INET, SOCK_STREAM, 0);

    assert(sock >= 0);

    int ret = bind(sock, (struct sockaddr *)&sever_address, sizeof(sever_address));
    assert(ret != -1);

    ret = listen(sock, 1);
    assert(ret != -1);
    cout<<"web server is running......"<<endl;
    while (1)
    {
        struct sockaddr_in client;
        socklen_t client_addrlength = sizeof(client);
        connfd = accept(sock, (struct sockaddr *)&client, &client_addrlength);
        if (connfd < 0)
        {
            printf("errno\n");
        }
        else
        {
            char request[1024];
            recv(connfd, request, 1024, 0);
            request[strlen(request) + 1] = '\0';
            printf("%s\n", request);
            printf("successeful!\n");
            //HTTP响应
            string agreement = "HTTP/1.1 200 ok\r\nconnection: close\r\n\r\n";
            string _str = web->htmlStr;
            string _lin = "%s";
            _str = WebServer::replace_all(_str,_lin,web->data->getMessage());
            agreement = agreement + _str;
            // int fd = open("", O_RDONLY); //消息体
            // sendfile(connfd, fd, NULL, 2500);      //零拷贝发送消息体
            send(connfd,agreement.c_str(),agreement.size(),0);
            // close(fd);
            close(connfd);
        }
    }
    return NULL;
}
