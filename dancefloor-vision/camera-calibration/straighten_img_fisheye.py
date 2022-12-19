import numpy as np
import cv2 as cv
import glob

# Straightens an image given camera calibration parameters.

name = "melissa" # Params to use
target = "photos/leo/target.jpg" # Picture to straighten
output = "result_melissa2.png" # Output file

# Get params
DIM = np.load("parameters/" + name + "/DIM.npy")
K = np.load("parameters/" + name + "/K.npy")
D = np.load("parameters/" + name + "/D.npy")

img = cv.imread(target)
h,w = img.shape[:2]

# See https://medium.com/@kennethjiang/calibrate-fisheye-lens-using-opencv-part-2-13990f1b157f

balance = 1 # Set to 1 to show black space. Set to 0 to crop
new_K = cv.fisheye.estimateNewCameraMatrixForUndistortRectify(K, D, DIM, np.eye(3), balance=balance) # Need this step if we don't want to crop

map1, map2 = cv.fisheye.initUndistortRectifyMap(K, D, np.eye(3), new_K, DIM, cv.CV_16SC2)
undistorted_img = cv.remap(img, map1, map2, interpolation=cv.INTER_LINEAR, borderMode=cv.BORDER_CONSTANT)

# Show image
cv.imshow("undistorted", undistorted_img)
cv.waitKey(0)
cv.destroyAllWindows()

cv.imwrite(output, undistorted_img) # Save it