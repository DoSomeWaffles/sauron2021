'''
 Copyright (c) 2021 University of Applied Sciences Western Switzerland / Fribourg

 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in all
 copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 SOFTWARE.

 Project: HEIA-FR / Sauron2021 Localisation system based on computer vision

 Purpose: This module is used to communicate with an external server

 Author:  Denis Rosset
 Date:    January 2021
'''


import socket
import select
import struct
import binascii
import traceback
import time
import Configuration

MILIS_TO_SECOND=1000

# The Serverc class is used to communicate with the webtools and the robots
class Server():
    JSON_FILENAME="configuration.json"
    connected_clients = dict()
    debug_connection_count = 0

    # sends package containing id and message to peer 
    @classmethod
    def send_to(cls, peer, msg_id, msg):
        data = bytearray(3)
        data.extend(msg)
        struct.pack_into('<HB', data, 0, len(msg)+1, msg_id)
        curr_time = round(time.time() * MILIS_TO_SECOND)

        try:
            if peer not in cls.connected_clients or curr_time - cls.connected_clients[peer][1] > 5000:
                if peer in cls.connected_clients:
                    cls.connected_clients[peer][0].close()
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.connect(peer)
                sock.settimeout(3)
                cls.connected_clients[peer] = [sock, round(time.time() * MILIS_TO_SECOND)]

            conn = cls.connected_clients[peer]
            conn[0].send(data)
            conn[1] = curr_time
        
        except ConnectionRefusedError as e:
            if cls.debug_connection_count > Configuration.SERVER_MAX_TRIES_BEFORE_TIMEOUT:
                print("connection refused for peer:")
                print(peer)
                cls.debug_connection_count = 0
            cls.debug_connection_count += 1
        except Exception as e:
            print("network error:")
            print(e)
            traceback.print_exc()
            print("for connection:")
            print(peer)

    # sends discovered points to webtools
    @classmethod
    def webtools_draw_points(cls, points, color, id):
        msg = bytearray(Configuration.SERVER_WEBTOOLS_OFFSET + Configuration.SERVER_FLOAT_OFFSET*len(points))
        struct.pack_into('<BBBBH', msg, 0, id, color[0], color[1], color[2], len(points))

        offset = Configuration.SERVER_WEBTOOLS_OFFSET
        for point in points:
            struct.pack_into('<hh', msg, offset, round(point[0]), round(point[1]))
            offset += Configuration.SERVER_FLOAT_OFFSET
            Server.send_to((Configuration.SERVER_WEBTOOLS_MESSAGE_ID, Configuration.SERVER_WEBTOOLS_MESSAGE_ID), Configuration.SERVER_WEBTOOLS_MESSAGE_ID, msg)
    
    # sends robot id, x and y coordinates to robot 1 and robot 2
    @classmethod
    def send_robot_position(cls, robot_id, x, y):
        msg = bytearray(Configuration.SERVER_TCP_PACKET_SIZE)
        struct.pack_into('<Bff', msg, 0, int(robot_id), x, y)
        Server.send_to((Configuration.SERVER_ROBOT1_ADDR, Configuration.SERVER_ROBOT1_PORT), Configuration.SERVER_ROBOT_MESSAGE_ID, msg)
        #Server.send_to((Configuration.SERVER_ROBOT2_ADDR, Configuration.SERVER_ROBOT2_PORT), Configuration.SERVER_ROBOT_MESSAGE_ID, msg)
