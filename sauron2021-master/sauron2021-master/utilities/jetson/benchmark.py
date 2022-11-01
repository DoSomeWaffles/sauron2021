import cv2
import time
import numpy as np
import cProfile
def cpu():
    frame = np.empty((3000, 3000, 3), dtype=np.uint8)
    print(frame.shape)
    t0 = time.time()
    for i in range(1):
        kernel = np.ones((5,5),np.float32)/25
        window = np.outer(kernel, kernel.transpose())

        mu1 = cv2.filter2D(frame, -1, window)[5:-5, 5:-5]  # valid

    print(time.time() - t0)

def gpu():
    frame_device = cv2.cuda_GpuMat()
    frame = np.empty((3000, 3000, 3), dtype=np.uint8)
    frame_device.upload(frame)
    print(frame.shape)
    t0 = time.time()
    for i in range(1):

        clahe = cv2.cuda.createCLAHE(clipLimit=5.0, tileGridSize=(8, 8))
        dst = clahe.apply(src, cv2.cuda_Stream.Null())

        result = dst.download()
    print(time.time() - t0)


print('Doing CPU')
cProfile.run('cpu()')
print('Doing GPU')
cProfile.run('gpu()')
exit()