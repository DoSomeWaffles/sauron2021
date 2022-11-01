import numpy as np
import cv2
import math

CAMERA_MATRIX = np.array([
    [ 1.2072034331538910e+03, 0., 1.6977711085073261e+03],
    [ 0., 1.1902329073724356e+03, 1.2664290386828982e+03],
    [ 0., 0., 1. ],
])

DIST_COEFFS = np.array([ -1.8839782083586495e-01, 2.8879939266968510e-02, -1.8918117889283245e-03, 6.5565242429574931e-04, -1.6932160665967502e-03 ])
mtx = CAMERA_MATRIX
dist = DIST_COEFFS
# Getting the new optimal camera matrix
img0 = cv2.imread('image0.png')
print(img0.shape[:2])
h, w = img0.shape[:2]
h1, w1 = 2*h, 2*w
newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w,h), 0, (w1, h1))
mapx, mapy = cv2.initUndistortRectifyMap(mtx, dist, None, newcameramtx, (w1, h1), 5)
dst = cv2.remap(img0, mapx, mapy, cv2.INTER_LINEAR)
# Checking to make sure the new camera materix was properly generated

# Undistorting
#dst0 = cv2.undistort(img0, CAMERA_MATRIX, DIST_COEFFS, None, newcameramtx)
# Cropping the image
x,y,w,h = roi
print(roi)
print(dst.shape)
#dst0 = dst0[y:+y+h, x:x+w]
cv2.imwrite('calibresult0.png',dst)