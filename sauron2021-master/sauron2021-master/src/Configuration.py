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

 Purpose: This module is used to configure each options of the program 

 Author:  Denis Rosset
 Date:    January 2021
'''

SERVER_WEBTOOLS_ADDR="192.168.0.200"
SERVER_WEBTOOLS_PORT=5500
SERVER_ROBOT1_ADDR="192.168.0.41"
SERVER_ROBOT1_PORT=5000
SERVER_ROBOT2_ADDR="192.168.0.43"
SERVER_ROBOT2_PORT=5000
SERVER_ROBOT_MESSAGE_ID=205
SERVER_WEBTOOLS_MESSAGE_ID=104
SERVER_MAX_TRIES_BEFORE_TIMEOUT=10
SERVER_ID_OFFSET=2
SERVER_FLOAT_OFFSET=4
SERVER_TCP_PACKET_SIZE=9
SERVER_WEBTOOLS_OFFSET=6

CAMERA_RESOLUTION_X=3296
CAMERA_RESOLUTION_Y=2464
CAMERA_FRAMERATE=15

UNWANTED_ARUCOS = [11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 43, 44, 45, 46, 47, 48, 49, 50]
ARUCO_TOP_SIZE = 70
ARUCO_ALLY_SIDE_SIZE = 70
ARUCO_ENNEMY_SIDE_SIZE = 50
ARUCO_CENTRAL_MARKER_SIZE = 100
TEAM_CHANGE_ID_OFFSET = [0,20]
LOGGED_PICTURES_SIZE_RATIO = 0.5
SAMPLE_FREQUENCY = 10 # position is logged and sent every n frames processed
JSON_FILENAME="aruco_translation.json"
CENTRAL_POSITON_MARKER_X=1250
CENTRAL_POSITON_MARKER_Y=1500
ALLY_ROBOT_1_ID=1
ALLY_ROBOT_2_ID=2
ENNEMY_ROBOT_1_ID=3
ENNEMY_ROBOT_2_ID=4
CENTRAL_ARUCO_ID=42


