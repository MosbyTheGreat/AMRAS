import os
import cv2
import numpy as np
import matplotlib.pyplot as plt

def processImage(img):
    if img is None:
        print("Image is None!")
        return None


    # parameters of the checkerboard
    square_size = 1.0

    pattern_size = (7, 7)
    pattern_points = np.zeros((np.prod(pattern_size), 3), np.float32)
    pattern_points[:, :2] = np.indices(pattern_size).T.reshape(-1, 2)
    pattern_points *= square_size

    found, corners = cv2.findChessboardCorners(img, pattern_size)
    if found: # refine corner with sub-pixel accuracy
        term = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_COUNT, 30, 0.1)
        corners = cv2.cornerSubPix(img, corners, (5, 5), (-1, -1), term)

    if not found:
        print('chessboard not found')
        return None
    else:
        print('chessboard found')

    return (corners.reshape(-1, 2), pattern_points)

# process all images
img_names = os.listdir('./Images/')
for img in img_names:
    print(img)

images = [ cv2.imread('./Images/' + fn.__str__(),cv2.IMREAD_GRAYSCALE) for fn in img_names ]
images = [ x for x in images if x is not None ] # filter empty images
h, w = images[0].shape[:2]

# find corners for all images
chessboards = [processImage( img ) for img in images] # process all images
chessboards = [x for x in chessboards if x is not None] # filter images that did not work!
img_points, obj_points = [],[]
for (corners, pattern_points) in chessboards:
    img_points.append(corners)
    obj_points.append(pattern_points)

# calculate camera distortion
rms, camera_matrix, dist_coefs, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points, (w, h), None, None)

print("\nRMS:", rms)
print("camera matrix:\n", camera_matrix)
print("distortion coefficients: ", dist_coefs.ravel())