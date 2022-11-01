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

 Purpose: This module is used to configure the camera

 Author:  Denis Rosset
 Date:    January 2021
'''

import numpy as np
import Configuration

# Camera class is used to set the camera parameters for the Jetson Nano
class Camera():
    # camera matrix used to undistort the lens of the camera
    CAMERA_MATRIX = np.array([
        [ 1.1642185371830324e+03, 0., 1.6625098807682714e+03],
        [ 0., 1.1645036678113613e+03, 1.2170786800318006e+03],
        [ 0., 0., 1. ],
    ])

    DIST_COEFFS = np.array([ -2.4181769938627334e-01, 5.8653995083099326e-02, 8.4650077651566061e-04, 2.7466061813867905e-04, -6.1956679930553812e-03 ])

    # gets every camera parameters
    @classmethod
    def gstreamer_pipeline(cls,
        capture_width=Configuration.CAMERA_RESOLUTION_X,
        capture_height=Configuration.CAMERA_RESOLUTION_Y,
        display_width=Configuration.CAMERA_RESOLUTION_X,
        display_height=Configuration.CAMERA_RESOLUTION_Y,
        framerate=Configuration.CAMERA_FRAMERATE,
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