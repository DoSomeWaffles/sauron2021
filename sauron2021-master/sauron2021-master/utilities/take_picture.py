
import numpy as np
import cv2
import nanocamera
import math
import time
from enum import Enum
from datetime import datetime
import statistics
import cProfile

frame = np.empty((2464, 3296, 3), dtype=np.uint8)

def gstreamer_pipeline(
    capture_width=3296,
    capture_height=2464,
    display_width=3296,
    display_height=2464,
    framerate=15,
    flip_method=0,
): 
    return (
        "nvarguscamerasrc ! "
        "video/x-raw(memory:NVMM), "
        "width=(int)%d, height=(int)%d, "
        "format=(string)NV12, framerate=(fraction)%d/1 ! "
        
        "queue max-size-buffers=1 leaky=downstream ! "
        
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink max-buffers=1 drop=True"
        % (
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )

transformation_camera_to_world = None
camera_position = None

cap = cv2.VideoCapture(gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)
i = 0
while(True):
    input("Press any key to take picture "+str(i)+"...")
    _, frame = cap.read()
    print('################# image saved ####################')
    t0 = time.time()
    cv2.imwrite("calibration/picture_" + str(t0) + ".png", frame)
    i+=1
