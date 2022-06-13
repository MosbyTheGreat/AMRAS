import os
import time
import cv2
import numpy as np
import matplotlib.pyplot as plt
import timeit


# values
camera_matrix = np.array([[1.52244073e+03,  0.00000000e+00, 1.18803566e+03]
                          [0.00000000e+00,  1.50426034e+03, 7.84669829e+02]
                          [0.00000000e+00,  0.00000000e+00, 1.00000000e+00]])
dist_coefs = np.array([-0.31086836,  0.10644732,  0.0030072,  -0.0012158,  -0.01891462])

print("camera matrix:\n", camera_matrix)
print("distortion coefficients: ", dist_coefs.ravel())

# load images
img_names = os.listdir('./Images/')
for img in img_names:
    print(img)

images = [ cv2.imread('./Images/' + fn.__str__(),cv2.IMREAD_GRAYSCALE) for fn in img_names ]
images = [ x for x in images if x is not None ] # filter empty images

for img in images:
    start = timeit.default_timer()
    # remove the the lens distortions
    dst = cv2.undistort(img, camera_matrix, dist_coefs, None, camera_matrix)

    stop = timeit.default_timer()
    print('Time: ', stop - start)  

    # show image
    cv2.imshow("AMRAS Viewfinder", dst)
    cv2.waitKey(1)
    plt.show()
    time.sleep(1)