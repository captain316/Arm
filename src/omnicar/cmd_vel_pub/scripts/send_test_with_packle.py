#!/usr/bin/python3.6
# -*- coding:utf-8 -*-
import socket
import struct

import pickle
s=socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
data1 = ('0.2222').encode('utf-8')
data2 = ('0.55522').encode('utf-8')

# data = arr.tobyte
# data = arr.tostring()
# data = pickle.dumps(arr)
while True:
    s.sendto(data1, ('192.168.1.103', 8000))
    s.sendto(data2, ('192.168.1.103', 8000))
    print(data1)

s.close()
