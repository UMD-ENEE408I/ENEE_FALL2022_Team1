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

_img_shape = None

i = 0 # Iteration number

for fname in images:
    img = cv.imread(fname)

    if _img_shape == None:
        _img_shape = img.shape[:2]

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
# See https://medium.com/@kennethjiang/calibrate-fisheye-lens-using-opencv-333b05afa0b0

calibration_flags = cv.fisheye.CALIB_RECOMPUTE_EXTRINSIC+cv.fisheye.CALIB_FIX_SKEW

N_OK = len(objpoints)
K = np.zeros((3, 3))
D = np.zeros((4, 1))
rvecs = [np.zeros((1, 1, 3), dtype=np.float64) for i in range(N_OK)]
tvecs = [np.zeros((1, 1, 3), dtype=np.float64) for i in range(N_OK)]
rms, _, _, _, _ = \
    cv.fisheye.calibrate(
        np.expand_dims(np.asarray(objpoints), -2),
        imgpoints,
        gray.shape[::-1],
        K,
        D,
        rvecs,
        tvecs,
        calibration_flags,
        (cv.TERM_CRITERIA_EPS+cv.TERM_CRITERIA_MAX_ITER, 30, 1e-6)
    )
print("Found " + str(N_OK) + " valid images for calibration.")
DIM = _img_shape[::-1]
K = K.tolist()
D = D.tolist()

# Save outputs to files
np.save("parameters/" + name + "/DIM", DIM)
np.save("parameters/" + name + "/K", K)
np.save("parameters/" + name + "/D", D)