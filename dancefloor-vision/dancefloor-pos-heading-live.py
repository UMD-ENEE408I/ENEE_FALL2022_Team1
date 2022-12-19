import cv2 as cv
import numpy as np
from pupil_apriltags import Detector # Using this library to detect AprilTags

# Get heading and position live, using fisheye undistort.
# Run save-dancefloor-info.py first to get dancefloor info.

sideLen = 17.3 # Tag side len in cm
# For solvePnP call
objectPoints = np.array([
    # For square, must start with top left, go clockwise
    [-sideLen / 2, +sideLen / 2, 0],
    [+sideLen / 2, +sideLen / 2, 0],
    [+sideLen / 2, -sideLen / 2, 0],
    [-sideLen / 2, -sideLen / 2, 0]
])

ft_to_cm = 30.48 * np.array([1, 1, 1]) # Number of cm in a foot

# Stores data for each tag's position and orientation. Note: z points out of
# back of tag, x right, y down for its frame.
tag_data = {}
tag_data[6] = np.load("T_tag6_to_wrld.npy")
tag_data[40] = np.load("T_tag40_to_wrld.npy")
tag_data[45] = np.load("T_tag45_to_wrld.npy")
tag_data[48] = np.load("T_tag48_to_wrld.npy")
print(tag_data)
  
# Define a video capture object
vid = cv.VideoCapture(0)

# Get params
name = "brandon"
#mtx = np.load("../Lab 10-6-22/parameters/" + name + "/mtx.npy")
#dist = np.load("../Lab 10-6-22/parameters/" + name + "/dist.npy")
K = np.load("../Lab 10-6-22/parameters/melissa/K.npy")
D = np.load("../Lab 10-6-22/parameters/melissa/D.npy")
DIM = np.load("../Lab 10-6-22/parameters/melissa/DIM.npy")
  
while(True):
    # Capture the video frame
    ret, img = vid.read()

    # Undistort it with previous code
    balance = 1
    new_K = cv.fisheye.estimateNewCameraMatrixForUndistortRectify(K, D, DIM, np.eye(3), balance=balance)
    map1, map2 = cv.fisheye.initUndistortRectifyMap(K, D, np.eye(3), new_K, DIM, cv.CV_16SC2)
    undistorted_img = cv.remap(img, map1, map2, interpolation=cv.INTER_LINEAR, borderMode=cv.BORDER_CONSTANT)
    img_gray = cv.cvtColor(undistorted_img, cv.COLOR_BGR2GRAY) # Needs to be grayscale for AprilTag detection!

    # Detect the tag
    at_detector = Detector(families="tag36h11", nthreads=1, quad_decimate=1.0, quad_sigma=0.0, refine_edges=1, decode_sharpening=0.25, debug=0)
    detection = at_detector.detect(img_gray)

    num_tags = len(detection) # Number of tags detected
    pos_sum = np.array([0, 0, 0]) # Sum of positions, to take avg later
    heading_sum = np.array([0, 0]) # Sum of headings, take avg later

    if num_tags == 0:
        print("ERROR: No tag detected!")
    else:
        # Get position for each detected tag
        for detect in detection:
            tag_id = detect.tag_id
            T_tag_to_wrld = tag_data[tag_id]

            # Get corners from detection and call solvePnP
            corners = detect.corners
            ret = cv.solvePnP(objectPoints=objectPoints, imagePoints=corners, cameraMatrix=new_K, distCoeffs=np.zeros((1,5)), flags=cv.SOLVEPNP_IPPE_SQUARE)

            # Returns rotation, translation arrays to convert tag frame to camera frame
            r_tag_to_cam = ret[1]
            t_tag_to_cam = ret[2]
            # The rvec is a vector representing the rotation axis, and its length encodes the angle in radians to rotate.
            # The Rodrigues method turns such a rotation vector into a 3x3 matrix.
            R_tag_to_cam = cv.Rodrigues(r_tag_to_cam)[0]
            # Augment to transformation matrix and get inverse
            T_tag_to_cam = np.concatenate((np.concatenate((R_tag_to_cam, t_tag_to_cam), axis=1), np.array([[0, 0, 0, 1]])), axis=0)
            T_cam_to_tag = np.linalg.inv(T_tag_to_cam)
            
            # Can now get T_c^w, which converts from camera coordinates to world
            # coordinates. This matrix contains position of camera in world, which is
            # what we want.
            T_cam_to_wrld = np.matmul(T_tag_to_wrld, T_cam_to_tag)

            # Heading - see photo. First two entries of third column give unit
            # vector in direction camera is pointing. First two entries of first
            # column give unit vector in direction of x-vector of camera.
            heading = T_cam_to_wrld[0:2, 2] # Get first two entries of third column

            pos = T_cam_to_wrld[0:3, 3] # Get position
            pos_ft = np.divide(pos, ft_to_cm) # Position in feet

            pos_sum = np.add(pos_sum, pos) # Add to sum of positions
            heading_sum = np.add(heading_sum, heading) # Add to sum of headings
            
            print(f'Tag {tag_id} pos:', pos_ft) # Print tag number and computed pos
            print(f'Tag {tag_id} heading:', heading) # Print tag number and heading

        # Print avgs
        pos_avg = pos_sum / num_tags
        pos_avg_ft = np.divide(pos_avg, ft_to_cm)
        heading_avg = heading_sum / num_tags
        print("Avg pos:", pos_avg_ft)
        print("Avg heading:", heading_avg)
        print("\n")
    
    # Show image after detections
    cv.imshow("undistorted", img_gray)
    cv.waitKey(0)
      
    # The 'q' button is set as the quitting button you may use any sdesired button of your choice
    if cv.waitKey(1) & 0xFF == ord('q'):
        break

vid.release() # After the loop release the cap object
cv.destroyAllWindows() # Destroy all the windows