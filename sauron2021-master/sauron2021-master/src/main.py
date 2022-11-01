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

 Purpose: This module calculate the position of an ArUco marker relative to the camera position

 Author:  Denis Rosset
 Date:    January 2021
'''

import numpy as np
import cv2
import nanocamera
import math
import time
from datetime import datetime
from Server import Server
from Camera import Camera
import json
import sys
import Configuration

# used to assign params entered when starting the program
def checkParams(params):
    # checks if the user entered enough params
    if len(sys.argv) < 5:
        print ("Not enough arguments.")
        print("Arguments are : ")
        print("Team : 0 for left or 1 for right")
        print("Robot 1 ArUco id")
        print("Robot 2 ArUco id")
        print("Robot 3 ArUco id")
        print("Robot 4 ArUco id")
        exit()

    team = int(sys.argv[1])
    aruco_random_ids = [int(sys.argv[2]),int(sys.argv[3]),int(sys.argv[4]),int(sys.argv[5])]
    return team, aruco_random_ids

# gets every marker found on the current frame
def getEveryMarkers(cap, aruco_dict, aruco_params):
    # copy the current frame into frame_color
    _, frame_color = cap.read()
    # gets corners and ids from every ArUco found on the frame with the detection parameters
    corners, ids, _ = cv2.aruco.detectMarkers(frame_color, aruco_dict, parameters=aruco_params)
    # draw every ArUcos found on the frame
    frame_draw = cv2.aruco.drawDetectedMarkers(np.copy(frame_color), corners)
    return corners, ids, frame_draw

# gets the cerntral marker position and translate it to find the camera position in the EuRobot frame of reference
def getCentralMarker(corners, ids, frame_draw):
    # transformation to apply to the central ArUco to find camera position
    TRANSFORMATION_CENTRAL_TO_WORLD = np.array([
        [math.cos(math.tau/4), -math.sin(math.tau/4), 0, Configuration.CENTRAL_POSITON_MARKER_X],
        [math.sin(math.tau/4), math.cos(math.tau/4), 0, Configuration.CENTRAL_POSITON_MARKER_Y],
        [0, 0, 1, 0],
        [0, 0, 0, 1],
    ])
    is_central_marker_found = False
    # seek if central ArUco is found
    for i in range(len(corners)):
        aruco_id = ids[i, 0]
        aruco_corners = np.array(corners)[i, 0, :, :]
        # if the aruco is the central one
        if aruco_id == Configuration.CENTRAL_ARUCO_ID:
            is_central_marker_found = True
            # gets 3d position of the ArUco coordinate system in the camera coordinate system
            r_vecs, t_vecs, _ = cv2.aruco.estimatePoseSingleMarkers(np.expand_dims(aruco_corners, axis=0), Configuration.ARUCO_CENTRAL_MARKER_SIZE, Camera.CAMERA_MATRIX, Camera.DIST_COEFFS)
            # draw the axis of the ArUco on the frame (this is only used to log the frame)
            frame_draw = cv2.aruco.drawAxis(frame_draw, Camera.CAMERA_MATRIX, Camera.DIST_COEFFS, r_vecs[0], t_vecs[0], 50)
            # gives rotation matrix from another representation of the angle
            rotation_matrix, _ = cv2.Rodrigues(r_vecs[0])
            # gets the transformation to go from the central coordinate system to the camera coordinate system
            transformation_central_to_camera = np.concatenate(
                (np.concatenate((rotation_matrix, t_vecs.reshape(3, 1)), axis=1),
                np.array([0, 0, 0, 1]).reshape(1, 4)),
                axis=0)
            # invert the matrix
            transformation_camera_to_central = np.linalg.inv(transformation_central_to_camera)
            # multiply the camera to central matrix with the central to world matrix to get the camera to world matrix
            transformation_camera_to_world = np.matmul(TRANSFORMATION_CENTRAL_TO_WORLD, transformation_camera_to_central)
            # get the camera X Y Z position in the Eurobot frame of reference
            camera_position = np.matmul(transformation_camera_to_world, np.array([0, 0, 0, 1]).reshape(4, 1))
            # divide by the homogeneous coordinate
            camera_position /= camera_position[3, 0]
            return is_central_marker_found, transformation_camera_to_world
    return False, []

# get every ArUco markers and transform them into Eurobot frame of reference
def getRobotMarkers(corners, ids, frame_draw, team, aruco_random_ids, transformation_camera_to_world, camera_position, aruco_positions):
    for i in range(len(corners)):
        aruco_size = Configuration.ARUCO_TOP_SIZE
        aruco_id = ids[i, 0]
        aruco_corners = np.array(corners)[i, 0, :, :]
        if not aruco_id in Configuration.UNWANTED_ARUCOS and transformation_camera_to_world is not None:
            # get the robot id if ArUco found is on the top of the robot 
            if(aruco_id in aruco_random_ids):
                if(aruco_random_ids[0] == aruco_id):
                    robot_id = 1
                if(aruco_random_ids[1] == aruco_id):
                    robot_id = 2
                if(aruco_random_ids[2] == aruco_id):
                    robot_id = 3
                if(aruco_random_ids[3] == aruco_id):
                    robot_id = 4
                aruco_translation, robot_id = getTranslationFromId(aruco_id, robot_id, team)
                aruco_size = Configuration.ARUCO_TOP_SIZE
            # get the robot id if ArUco found is on the side of the robot 
            else:
                crt_id = aruco_id-Configuration.TEAM_CHANGE_ID_OFFSET[team]
                aruco_translation, robot_id = getTranslationFromId(crt_id, -1, team)
                # set the right size for the side of the robot if it's ally or ennemy
                if(robot_id == Configuration.ENNEMY_ROBOT_1_ID or robot_id == Configuration.ENNEMY_ROBOT_2_ID):
                    aruco_size = Configuration.ARUCO_ENNEMY_SIDE_SIZE
                elif(robot_id == Configuration.ALLY_ROBOT_1_ID or robot_id == Configuration.ALLY_ROBOT_2_ID):
                    aruco_size = Configuration.ARUCO_ALLY_SIDE_SIZE

            # gets 3d position of the ArUco coordinate system in the camera coordinate system
            r_vecs, t_vecs, _ = cv2.aruco.estimatePoseSingleMarkers(np.expand_dims(aruco_corners, axis=0), aruco_size, Camera.CAMERA_MATRIX, Camera.DIST_COEFFS)
            # draw axis of the ArUco on the frame
            frame_draw = cv2.aruco.drawAxis(frame_draw, Camera.CAMERA_MATRIX, Camera.DIST_COEFFS, r_vecs, t_vecs, 0.05)
            # gives rotation matrix from another representation of the angle
            rotation_matrix, _ = cv2.Rodrigues(r_vecs[0])
            # gets the transformation to go from the robot coordinate system to the camera coordinate system
            transformation_robot_to_camera = np.concatenate(
                (np.concatenate((rotation_matrix, t_vecs.reshape(3, 1)), axis=1),
                np.array([0, 0, 0, 1]).reshape(1, 4)),
                axis=0)
            # multiply the matrix to get the transformation to go from the robot coordinate system to the world coordinate system
            transformation_robot_to_world = np.matmul(transformation_camera_to_world, transformation_robot_to_camera)
            # divide by the homogeneous coordinate
            transformation_robot_to_world /= transformation_robot_to_world[3, 3]
            # get the robot coordinates from the Eurobot frame of reference
            robot_position = np.matmul(transformation_robot_to_world, np.array(aruco_translation).reshape(4, 1))
            
        # create a new dictionnary and adds every robot position found with the id of the robot and the area of the correponding ArUco 
        if not aruco_id in Configuration.UNWANTED_ARUCOS:
            if (str(robot_id) not in aruco_positions):
                aruco_positions[str(robot_id)]={}

            if str(aruco_id) not in aruco_positions[str(robot_id)]:
                aruco_positions[str(robot_id)][str(aruco_id)]= {}
                aruco_positions[str(robot_id)][str(aruco_id)]['x']=[]
                aruco_positions[str(robot_id)][str(aruco_id)]['y']=[]
                aruco_positions[str(robot_id)][str(aruco_id)]['z']=[]
                aruco_positions[str(robot_id)][str(aruco_id)]['area']=[]
                aruco_positions[str(robot_id)][str(aruco_id)]['robot_id']=[robot_id]

            robot_position /= robot_position[3, 0]
            aruco_positions[str(robot_id)][str(aruco_id)]['x'].append(robot_position[0, 0])
            aruco_positions[str(robot_id)][str(aruco_id)]['y'].append(robot_position[1, 0])
            aruco_positions[str(robot_id)][str(aruco_id)]['z'].append(robot_position[2, 0])
            aruco_positions[str(robot_id)][str(aruco_id)]['area'].append(cv2.contourArea(corners[i]))
    return aruco_positions

# gets the translation from the camera to the ArUco for each ArUco detected and returns it grouped by robot id
def getTranslationFromId(aruco_id, robot_id, team):  
    # open the json containing every ArUco translation relative to the center of the robot
    with open('aruco_translation.json') as translation_file:
        aruco_translation_list = json.load(translation_file)
        # sets the right translation to the top oriented ArUcos
        if robot_id==Configuration.ALLY_ROBOT_1_ID:
            translation = aruco_translation_list['translation_list']['aruco_dessus_grand_robot']
            return translation, robot_id
        elif robot_id==Configuration.ALLY_ROBOT_2_ID :
            translation = aruco_translation_list['translation_list']['aruco_dessus_petit_robot']
            return translation, robot_id
        elif robot_id==Configuration.ALLY_ROBOT_1_ID or robot_id==Configuration.ALLY_ROBOT_2_ID :
            translation = aruco_translation_list['translation_list']['aruco_dessus_robot_ennemi']
            return translation, robot_id
        any_aruco_found = False

        # sets the right translation to side oriented ArUcos
        for aruco_translation in aruco_translation_list['aruco_list']:
            crt_id = aruco_id-Configuration.TEAM_CHANGE_ID_OFFSET[team]
            if(int(aruco_translation['id']) == crt_id):
                translation = aruco_translation_list['translation_list'][aruco_translation['translation']]
                any_aruco_found = True
                robot_id = aruco_translation['robot']
                break

        if(not any_aruco_found):
            # set robot id to -1 and null translation to log when ArUcos are detected but not wanted
            robot_id = -1
            translation = [0,0,0,1]
    return translation, robot_id

# gets the ArUco with the wider area for each robot ID 
def getBestPositionsFromArucoList(aruco_positions):
    best_arucos_by_robot = []
    # for each different robot detected
    for robot_id in aruco_positions:
        # concerns only robot id 1 to 4 (not 99)
        if int(robot_id) <1 or int(robot_id) >4:
            continue
        best_aruco_area = -1
        best_aruco_id = -1
        # gets the best ArUco based on its area
        for aruco_id in aruco_positions[robot_id]:
            # checks the median to filter the bad values
            if np.median(aruco_positions[robot_id][aruco_id]['area'])>best_aruco_area:
                best_aruco_id = aruco_id
        best_arucos_by_robot.append(aruco_positions[robot_id][best_aruco_id])
    return best_arucos_by_robot

# logs every ArUcos from every robots currently in aruco_positions and saves the frame
def logDetectedArucos(aruco_positions, frame_draw):
    t0 = time.time()
    log_file = open("pictures/logs.txt", "a")
    for robot_id in aruco_positions:
        for aruco_id in aruco_positions[str(robot_id)]:
            log_file.write('frame id: '+str(t0))
            log_file.write(' trust factor: '+str(int(np.median(aruco_positions[str(robot_id)][aruco_id]['area']))))
            log_file.write(' robot : '+str(aruco_positions[str(robot_id)][aruco_id]['robot_id'][0]))
            log_file.write(' aruco id: ' +str(aruco_id)+ ' position: (')
            log_file.write(' x: '+str(np.median(aruco_positions[str(robot_id)][aruco_id]['x'])))
            log_file.write(' y: '+str(np.median(aruco_positions[str(robot_id)][aruco_id]['y'])))
            log_file.write(' z: '+str(np.median(aruco_positions[str(robot_id)][aruco_id]['z'])))
            log_file.write(')')
            log_file.write('\n')
            csv_file = open("pictures/aruco_"+str(aruco_id)+".txt", "a+")
            csv_file.write(str(int(np.median(aruco_positions[str(robot_id)][aruco_id]['x'])))
                        +','+str(int(np.median(aruco_positions[str(robot_id)][aruco_id]['y'])))
                        +','+str(int(np.median(aruco_positions[str(robot_id)][aruco_id]['z'])))+'\n')
            csv_file.close()
    log_file.close()
    cv2.imwrite("pictures/frame_draw_" + str(t0) + ".jpg", cv2.resize(frame_draw, (0,0),fx=Configuration.LOGGED_PICTURES_SIZE_RATIO, fy=Configuration.LOGGED_PICTURES_SIZE_RATIO))

# Entry point of the programm. Instanciate variables and opens the video capture. For each frame (last frame available) 
# check for central ArUcos, for robot ArUcos, logs the positions found and sends the position to the robots.
def main():
    team, aruco_random_ids = checkParams(sys.argv)
    aruco_positions = {}
    transformation_camera_to_world = None
    camera_position = None
    is_central_marker_found = False
    cap = cv2.VideoCapture(Camera.gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)
    frame_number = 0

    if cap.isOpened():
        # set detection parameters with subpixel refinement
        aruco_params = cv2.aruco.DetectorParameters_create()
        aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_100)
        # loop that stops only when a keyboard interruption is detected or when the program is killed
        tstart = round(time.time()*1000)
        while True:
            
            print('##### new frame')
            # gets every marker
            corners, ids, frame_draw = getEveryMarkers(cap, aruco_dict, aruco_params)
            # gets central ArUco marker and transformation
            is_central_marker_found, transformation_camera_to_world = getCentralMarker(corners, ids, frame_draw)
            if not is_central_marker_found:
                print("CENTRAL MARKER NOT FOUND")
                continue
            # gets every robot ArUco markers positions
            aruco_positions = getRobotMarkers(corners, ids, frame_draw, team, aruco_random_ids, transformation_camera_to_world, camera_position, aruco_positions)
            
            # logs and sends position to robot every nth frame
            if frame_number%Configuration.SAMPLE_FREQUENCY == Configuration.SAMPLE_FREQUENCY-1:
                logDetectedArucos(aruco_positions, frame_draw)
                best_arucos = getBestPositionsFromArucoList(aruco_positions)
                for key in best_arucos:
                    print('robot : ' + str(key['robot_id'][0]))
                    print('x : ' +  str(np.median(key['x'])))
                    print('y : ' +  str(np.median(key['y'])))
                    print('z : ' +  str(np.median(key['z'])))
                    Server.send_robot_position(key['robot_id'][0], np.median(key['x']), np.median(key['y']))
                tend = round(time.time()*1000)
                
                #print(str(tend-tstart))
                tstart=round(time.time()*1000)
                cv2.destroyAllWindows()
                # reset the dictionnary with every coordinate
                for robot_id in aruco_positions:
                    aruco_positions[str(robot_id)]={}
            frame_number+=1
    cv2.destroyAllWindows()
if __name__== "__main__":
    main()

