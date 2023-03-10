#!/usr/bin/python3.6
# -*- coding:utf-8 -*-
import os
import time
import socket
import threading
# import lbGlobalDef as Gbl


class IB_UDP(threading.Thread):
    def __init__(self, ip="127.0.0.1", port=8001):                 #目标端口
        threading.Thread.__init__(self)
        self.thread_stop = False
        self.hostIP = ip
        self.hostPort = int(port)
        self.server = None
        self.DEL_BUS_PORT = 33001
        self.MCU_BUS_PORT = 33002
        # interface is for broadcast receive
        try:
            self.server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
            try:
                self.server.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
                self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            except AttributeError:
                pass
            self.server.bind((self.hostIP, self.hostPort))
        except Exception as e:
            print("####ERROR: IB_UDP->socket:" + str(e) + "\n")
    
    def send_msg(self, data, port=8000, ip="127.0.0.1"):                       #发送端口
        try:
            if not isinstance(data, bytes):
                if isinstance(data, bytearray):
                    data = bytes(data)
                elif isinstance(data, str):
                    data = data.encode()
            return self.server.sendto(data, (ip, port))
        except Exception as e:
            print("####ERROR: IB_UDP->send_msg:" + str(e) + "\n")
            return 0
    
    def on_receive_msg(self, data):
        try:
            print(data)
        except Exception as e:
            print("####ERROR: IB_UDP->on_receive_msg_rec:" + str(e) + "\n")

    def run(self):
        while (self.thread_stop is False) and (self.server is not None):
            try:
                data, addr = self.server.recvfrom(65000)
                if (data is not None) and (addr is not None):
                    self.on_receive_msg(data)
            except Exception as e:
                print("####ERROR: IB_UDP->run:" + str(e) + "\n")

    def stop(self):
        # set this flag to stop thread here
        self.thread_stop = True
        time.sleep(0.2)


if __name__ == '__main__':
    ibus = IB_UDP()
    ibus.start()
    # 8个编码器值
    alm_info = str(1.1) + "," + str(2.2) + str(3.3) + "," + str(4.4) + str(5.5) + "," + str(6.6) + str(7.7) + "," + str(8.8) 
    while(True):
        time.sleep(1)
        ibus.send_msg(alm_info)
        print("have sened msg")



