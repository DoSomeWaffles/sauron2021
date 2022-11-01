
import numpy as np
import cv2
import picamera
import math
from enum import Enum
from datetime import datetime
import statistics



CAMERA_MATRIX = np.array([
    [ 1.1642185371830324e+03, 0., 1.6625098807682714e+03],
    [ 0., 1.1645036678113613e+03, 1.2170786800318006e+03],
    [ 0., 0., 1. ],
])

DIST_COEFFS = np.array([ -2.4181769938627334e-01, 5.8653995083099326e-02, 8.4650077651566061e-04, 2.7466061813867905e-04, -6.1956679930553812e-03 ])

TRANSFORMATION_CENTRAL_TO_WORLD = np.array([
    [math.cos(math.tau/4), -math.sin(math.tau/4), 0, 1250],
    [math.sin(math.tau/4), math.cos(math.tau/4), 0, 1500],
    [0, 0, 1, 0],
    [0, 0, 0, 1],
])

transformation_camera_to_world = None
camera_position = None
frame = np.empty((2464, 3296, 3), dtype=np.uint8)

with picamera.PiCamera() as camera:
    camera.resolution = (3280, 2464)
    camera.framerate = 15
    camera.shutter_speed = 15000
    aruco_params = cv2.aruco.DetectorParameters_create()
    aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_100)
    i = 0
    while True:
        print('##### new frame')
        camera.capture(frame, format='rgb')
        frame_color = cv2.cvtColor(frame[:2464, :3280, :], cv2.COLOR_RGB2BGR)

        corners, ids, rejected_points = cv2.aruco.detectMarkers(frame_color, aruco_dict, parameters=aruco_params)

        frame_draw = cv2.aruco.drawDetectedMarkers(np.copy(frame_color), corners)

        for i in range(len(corners)):
            aruco_id = ids[i, 0]
            aruco_corners = np.array(corners)[i, 0, :, :]

            if aruco_id == 42:
                print('--- found central marker (' + str(aruco_id) + ')')
                r_vecs, t_vecs, _ = cv2.aruco.estimatePoseSingleMarkers(np.expand_dims(aruco_corners, axis=0), 100, CAMERA_MATRIX, DIST_COEFFS)
                frame_draw = cv2.aruco.drawAxis(frame_draw, CAMERA_MATRIX, DIST_COEFFS, r_vecs[0], t_vecs[0], 50)

                rotation_matrix, _ = cv2.Rodrigues(r_vecs[0])
                transformation_central_to_camera = np.concatenate(
                    (np.concatenate((rotation_matrix, t_vecs.reshape(3, 1)), axis=1),
                     np.array([0, 0, 0, 1]).reshape(1, 4)),
                    axis=0)
                transformation_camera_to_central = np.linalg.inv(transformation_central_to_camera)
                transformation_camera_to_world = np.matmul(TRANSFORMATION_CENTRAL_TO_WORLD, transformation_camera_to_central)

                camera_position = np.matmul(transformation_camera_to_world, np.array([0, 0, 0, 1]).reshape(4, 1))
                camera_position /= camera_position[3, 0]
                print('camera position: (')
                print('    x: ' + str(camera_position[0, 0]))
                print('    y: ' + str(camera_position[1, 0]))
                print('    z: ' + str(camera_position[2, 0]))
                print(')')
        
        for i in range(len(corners)):
            aruco_id = ids[i, 0]
            aruco_corners = np.array(corners)[i, 0, :, :]

            if aruco_id >= 1 and aruco_id <= 10 and transformation_camera_to_world is not None:
                print('--- found robot marker (' + str(aruco_id) + ')')
                r_vecs, t_vecs, _ = cv2.aruco.estimatePoseSingleMarkers(np.expand_dims(aruco_corners, axis=0), 70, CAMERA_MATRIX, DIST_COEFFS)
                frame_draw = cv2.aruco.drawAxis(frame_draw, CAMERA_MATRIX, DIST_COEFFS, r_vecs[0], t_vecs[0], 35)

                rotation_matrix, _ = cv2.Rodrigues(r_vecs[0])
                transformation_robot_to_camera = np.concatenate(
                    (np.concatenate((rotation_matrix, t_vecs.reshape(3, 1)), axis=1),
                     np.array([0, 0, 0, 1]).reshape(1, 4)),
                    axis=0)
                transformation_robot_to_world = np.matmul(transformation_camera_to_world, transformation_robot_to_camera)
                transformation_robot_to_world /= transformation_robot_to_world[3, 3]

                robot_position = np.matmul(transformation_robot_to_world, np.array([0, 0, 0, 1]).reshape(4, 1))
                robot_position /= robot_position[3, 0]
                print('robot ' + str(aruco_id) + ' position: (')
                print('    x: ' + str(robot_position[0, 0]))
                print('    y: ' + str(robot_position[1, 0]))
                print('    z: ' + str(robot_position[2, 0]))
                print(')')
            else:
                print('Central marker not found or missing from camera pov')
        
        cv2.imshow('camera', frame_draw)
        w = cv2.waitKey(1)
        if w & 0XFF == ord('p'):
            print("writing image : " + str(i) + str(datetime.now()))
            cv2.imwrite("pictures/frame_" + str(i) + str(datetime.now()) + ".png", frame_color)
            i+=1
        if w & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()
