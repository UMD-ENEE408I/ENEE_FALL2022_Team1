import numpy as np
import cv2 as cv
from pupil_apriltags import Detector # Using this library to detect AprilTags

# Detect AprilTag in image and then get distance with normal camera matrix (not fisheye).
# Note - this version works quite well!

# Get params
name = "melissa"
mtx = np.load("../Lab 10-6-22/parameters/" + name + "/mtx.npy")
dist = np.load("../Lab 10-6-22/parameters/" + name + "/dist.npy")

target = "photos/20l.jpg" # Picture to use
img = cv.imread(target) # Read in image

img_gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY) # Needs to be grayscale for AprilTag detection!
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

    # Now call function to get distance.
    ret = cv.solvePnP(
        objectPoints=objectPoints,
        imagePoints=corners,
        cameraMatrix=mtx,
        distCoeffs=dist,
        flags=cv.SOLVEPNP_IPPE_SQUARE # Since points are in a square
    )

    # Returns rotation, translation arrays
    t = ret[2]
    dist = np.linalg.norm(t, 2)
    print(dist)