import numpy as np
import cv2 as cv
import glob

# Straightens an image given camera calibration parameters.

name = "melissa" # Params to use
target = "photos/brandon/target2.jpg" # Picture to straighten
output = "result_brandon.png" # Output file

# Read camera distortion params

ret = np.load("parameters/" + name + "/ret.npy")
mtx = np.load("parameters/" + name + "/mtx.npy")
dist = np.load("parameters/" + name + "/dist.npy")
rvecs = np.load("parameters/" + name + "/rvecs.npy")
tvecs = np.load("parameters/" + name + "/tvecs.npy")

# Take a new image

img = cv.imread(target)
h,  w = img.shape[:2]
newcameramtx, roi = cv.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))

# Now, undistort

# undistort
dst = cv.undistort(img, mtx, dist, None, newcameramtx)

# crop the image
#x, y, w, h = roi
#dst = dst[y:y+h, x:x+w]
cv.imwrite(output, dst)