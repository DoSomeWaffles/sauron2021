import cv2
import time
import numpy as np
import cProfile
def cpu():
    for i in range(5,49,2):
        img1 = cv.medianBlur(img1,i)
        
    print(time.time() - t0)

print('Doing CPU')
cProfile.run('cpu()')
exit()