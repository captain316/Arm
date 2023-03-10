#!/usr/bin/python3.6
# -*- coding:utf-8 -*-
import socket
import struct

import json
s=socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
arr = [0.2222, 0.2335]
# data = arr.tobyte
# data = arr.tostring()
data = json.dumps(arr)
#bytes(data, encoding = "utf8")
while True:
    s.sendto(data.encode('utf-8'), ('192.168.1.103', 8000))
    print(data)

s.close()
