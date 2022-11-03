import numpy as np
import cv2 as cv
from pupil_apriltags import Detector # Using this library to detect AprilTags

# Detect AprilTag in image and then get distance - assuming fisheye model.
# Note - the non-fisheye version works much better.

# Get params - using Melissa's for now because only those are correct
DIM = np.load("../Lab 10-6-22/parameters/melissa/DIM.npy")
K = np.load("../Lab 10-6-22/parameters/melissa/K.npy")
D = np.load("../Lab 10-6-22/parameters/melissa/D.npy")

target = "photos/10r.jpg" # Picture to use
img = cv.imread(target) # Read in image

# Undistort it with previous code
balance = 1
new_K = cv.fisheye.estimateNewCameraMatrixForUndistortRectify(K, D, DIM, np.eye(3), balance=balance)
map1, map2 = cv.fisheye.initUndistortRectifyMap(K, D, np.eye(3), new_K, DIM, cv.CV_16SC2)
undistorted_img = cv.remap(img, map1, map2, interpolation=cv.INTER_LINEAR, borderMode=cv.BORDER_CONSTANT)

img_gray = cv.cvtColor(undistorted_img, cv.COLOR_BGR2GRAY) # Needs to be grayscale for AprilTag detection!
cv.imshow("undistorted", img_gray)
cv.waitKey(0)

# Detect the tag
at_detector = Detector(
   families="tag36h11",
   nthreads=1,
   quad_decimate=1.0,
   quad_sigma=0.0,
   refine_edges=1,
   decode_sharpening=0.25,
   debug=0
)
detection = at_detector.detect(img_gray)

# If detected, get distance
if len(detection) == 0:
    print("ERROR: No tag detected!")
else:
    corners = detection[0].corners # Get corners from detection

    # Now get metric distance with library function

    sideLen = 5.3 # In centimeters
    # Must be a numpy array
    objectPoints = np.array([
        # For square, must start with top left, go clockwise
        [-sideLen / 2, +sideLen / 2, 0],
        [+sideLen / 2, +sideLen / 2, 0],
        [+sideLen / 2, -sideLen / 2, 0],
        [-sideLen / 2, -sideLen / 2, 0]
    ])

    # Now call function to get distance. Since already undistorted, K and D are 0
    ret = cv.solvePnP(
        objectPoints=objectPoints,
        imagePoints=corners,
        cameraMatrix=np.eye(3),
        distCoeffs=np.zeros(5),
        flags=cv.SOLVEPNP_IPPE_SQUARE # Since points are in a square
    )

    # Returns rotation, translation arrays
    t = ret[2]
    dist = np.linalg.norm(t, 2) / 2.5 # 2.5 scaling factor seems to work for some reason
    print(dist)