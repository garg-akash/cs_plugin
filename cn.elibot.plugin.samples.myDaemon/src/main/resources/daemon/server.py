#!/usr/bin/env python3

import time
import sys
import _thread
import re

from xmlrpc.server import SimpleXMLRPCServer
from http.server import BaseHTTPRequestHandler, HTTPServer

message ='Hello, world'


def set_message(mes):
    global message
    message = mes
    print ("message from client: " + mes)
    return message

class RequestHandler(BaseHTTPRequestHandler):
    '''处理请求并返回页面'''
    global message

    # 页面模板
    Page = '''\
        <html>
        <body>
        <p>{message} </p>
        </body>
        </html>
    '''

    # 处理一个GET请求
    def do_GET(self):
        print ("do_GET")
        print ("do_GET: " + message)
        mes = re.sub(r'\{message\}', self.Page, message)
        self.send_response(200)
        self.send_header("Content-Type", "text/html")
        self.send_header("Content-Length", str(len(mes)))
        self.end_headers()
        self.wfile.write(mes.encode('utf-8'))

def rpc_server(threadName, delay):
    sys.stdout.write("MyDaemon daemon started")
    sys.stderr.write("MyDaemon daemon started")
    server = SimpleXMLRPCServer(("127.0.0.1", 4444))
    server.register_function(set_message, "set_message")
    server.serve_forever()

def http_server(threadName, delay):
    serverAddress = ('127.0.0.1', 5555)
    server = HTTPServer(serverAddress, RequestHandler)
    server.serve_forever()


if __name__ == '__main__':
    # 创建两个线程
    try:
        _thread.start_new_thread( http_server, ("Thread-1", 1, ) )
        _thread.start_new_thread( rpc_server, ("Thread-2", 2, ) )
    except:
        print ("Error: 无法启动线程")
    while 1:
        pass





