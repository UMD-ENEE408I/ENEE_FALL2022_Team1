import numpy as np
import cv2 as cv
import glob

# Outputs camera calibration parameters from photos.

name = "melissa" # Pictures to process

# Get corners from pictures of checkerboard

# termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 50, 0.001)
# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,6,0)
objp = np.zeros((7*7,3), np.float32)
objp[:,:2] = np.mgrid[0:7,0:7].T.reshape(-1,2)
# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.
images = glob.glob('photos/' + name + '/*.jpg')

i = 0 # Iteration number

for fname in images:
    img = cv.imread(fname)
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    # Find the chess board corners
    ret, corners = cv.findChessboardCorners(gray, (7,7), None)
    # If found, add object points, image points (after refining them)

    # Print iteration number
    i = i + 1
    print("Image", i, end=" ")

    if ret == True:
        print("(SUCCESS!)") # Indicate if that picture was processed successfully

        objpoints.append(objp)
        corners2 = cv.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
        imgpoints.append(corners)
        # Draw and display the corners
        cv.drawChessboardCorners(img, (7,7), corners2, ret)
        cv.imshow('img', img)
        cv.waitKey(500)
    else:
        print("(FAILED)") # Indicate failure
cv.destroyAllWindows()

# Calibrate camera

ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

# Save outputs to files

np.save("parameters/" + name + "/ret", ret)
np.save("parameters/" + name + "/mtx", mtx)
np.save("parameters/" + name + "/dist", dist)
np.save("parameters/" + name + "/rvecs", rvecs)
np.save("parameters/" + name + "/tvecs", tvecs)