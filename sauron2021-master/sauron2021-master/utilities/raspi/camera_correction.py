import numpy as np
import cv2
import picamera
import math
from enum import Enum
from datetime import datetime
import statistics

<<<<<<< HEAD
CAMERA_MATRIX = np.array([
    [1.0496023192800999e+03, 0., 1.4245418705857289e+03],
    [0., 1.0518927019464054e+03, 9.5655621119318903e+02],
    [0., 0., 1.],
])

DIST_COEFFS = np.array([-3.1732988929049799e-01, 9.7092364944771095e-02, -7.2089682561056164e-04, -1.2574689566626838e-03, -1.3026411006679277e-02 ])

TRANSFORMATION_CENTRAL_TO_WORLD = np.array([
    [math.cos(math.tau/4), -math.sin(math.tau/4), 0, 1250],
    [math.sin(math.tau/4), math.cos(math.tau/4), 0, 1500],
    [0, 0, 1, 0],
    [0, 0, 0, 1],
])

transformation_camera_to_world = None
camera_position = None
=======
>>>>>>> b557a8a1f468941990e91b21beece19b69523a95
frame = np.empty((2464, 3296, 3), dtype=np.uint8)

with picamera.PiCamera() as camera:
    camera.resolution = (3280, 2464)
    camera.framerate = 15
    camera.shutter_speed = 15000

    i = 0

    while True:

        print('##### new frame')
        camera.capture(frame, format='rgb')
        frame_color = cv2.cvtColor(frame[:2464, :3280, :], cv2.COLOR_RGB2BGR)

        cv2.imshow('camera', cv2.resize(frame_color, (0, 0), fx=0.3, fy=0.3))
        w = cv2.waitKey(1)
        if w & 0XFF == ord('p'):
            print("writing image : " + str(i) + str(datetime.now()))
            cv2.imwrite("pictures/frame_" + str(i) + str(datetime.now()) + ".png", frame_color)
            i+=1
        if w & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()
