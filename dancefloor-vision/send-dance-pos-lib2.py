#!/usr/bin/env python

import numpy as np
from argparse import ArgumentParser
import os
import cv2 as cv
import apriltag

import subprocess

# Get heading and position live, using fisheye undistort.
# Run save-dancefloor-info.py first to get dancefloor info.

robot_ip = "192.168.22.53"
robot_port = "2391"

sideLen = 16 # Tag side len in cm
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
tag_data[0] = np.load("T_tag0_to_wrld.npy")
tag_data[1] = np.load("T_tag1_to_wrld.npy")
tag_data[6] = np.load("T_tag6_to_wrld.npy")
tag_data[17] = np.load("T_tag17_to_wrld.npy")
tag_data[40] = np.load("T_tag40_to_wrld.npy")
tag_data[45] = np.load("T_tag45_to_wrld.npy")
tag_data[48] = np.load("T_tag48_to_wrld.npy")
tag_data[49] = np.load("T_tag49_to_wrld.npy")

# Get params
K = np.load("K.npy")
D = np.load("D.npy")
DIM = np.load("DIM.npy")

################################################################################

def apriltag_video(input_streams=[0], # For default cam use -> [0]
                   output_stream=False,
                   display_stream=True,
                   detection_window_name='AprilTag',
                  ):

    '''
    Detect AprilTags from video stream.

    Args:   input_streams [list(int/str)]: Camera index or movie name to run detection algorithm on
            output_stream [bool]: Boolean flag to save/not stream annotated with detections
            display_stream [bool]: Boolean flag to display/not stream annotated with detections
            detection_window_name [str]: Title of displayed (output) tag detection window
    '''

    parser = ArgumentParser(description='Detect AprilTags from video stream.')
    apriltag.add_arguments(parser)
    options = parser.parse_args()

    '''
    Set up a reasonable search path for the apriltag DLL.
    Either install the DLL in the appropriate system-wide
    location, or specify your own search paths as needed.
    '''

    detector = apriltag.Detector(options, searchpath=apriltag._get_dll_path())

    for stream in input_streams:

        video = cv.VideoCapture(stream)

        output = None

        if output_stream:
            width = int(video.get(cv.CAP_PROP_FRAME_WIDTH))
            height = int(video.get(cv.CAP_PROP_FRAME_HEIGHT))
            fps = int(video.get(cv.CAP_PROP_FPS))
            codec = cv.VideoWriter_fourcc(*'XVID')
            if type(stream) != int:
                output_path = '../media/output/'+str(os.path.split(stream)[1])
                output_path = output_path.replace(str(os.path.splitext(stream)[1]), '.avi')
            else:
                output_path = '../media/output/'+'camera_'+str(stream)+'.avi'
            output = cv.VideoWriter(output_path, codec, fps, (width, height))

        useFrame = 0

        while(video.isOpened()):

            success, img = video.read()
            if not success:
                break

            # Throw away 99/100 of the frames
            useFrame = (useFrame + 1) % 100
            if useFrame == 0:

                # Undistort it with previous code
                balance = 1
                new_K = cv.fisheye.estimateNewCameraMatrixForUndistortRectify(K, D, DIM, np.eye(3), balance=balance)
                map1, map2 = cv.fisheye.initUndistortRectifyMap(K, D, np.eye(3), new_K, DIM, cv.CV_16SC2)
                undistorted_img = cv.remap(img, map1, map2, interpolation=cv.INTER_LINEAR, borderMode=cv.BORDER_CONSTANT)
                img_gray = cv.cvtColor(undistorted_img, cv.COLOR_BGR2GRAY) # Needs to be grayscale for AprilTag detection!

                # Note - detection has 4 elts for each detected tag.
                # Detection object, array, 2 numbers.
                detection, overlay = apriltag.detect_tags(img_gray,
                                                    detector,
                                                    camera_params=(3156.71852, 3129.52243, 359.097908, 239.736909),
                                                    tag_size=0.0762,
                                                    vizualization=3,
                                                    verbose=3,
                                                    annotation=True
                                                    )

                num_tags = len(detection) / 4 # Number of tags detected
                pos_sum = np.array([0, 0, 0]) # Sum of positions, to take avg later
                heading_sum = np.array([0, 0]) # Sum of headings, take avg later

                if num_tags == 0:
                    print("ERROR: No tag detected!")
                else:
                    # Get position for each detected tag
                    i = 0
                    while i < num_tags:
                        detect = detection[4 * i]

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

                        i = i + 1 # For while loop

                    # Print avgs
                    pos_avg = pos_sum / num_tags
                    pos_avg_ft = np.divide(pos_avg, ft_to_cm)
                    heading_avg = heading_sum / num_tags
                    print("Avg pos:", pos_avg_ft)
                    print("Avg heading:", heading_avg)
                    print("\n")

                    msg = f'{pos_avg[0]} {pos_avg[1]} {heading_avg[0]} {heading_avg[1]}' # Combine into message
                    # Syscall to send it
                    print("Sending:", msg)
                    cmd = "echo -n {} | nc -u {} {}".format(msg, robot_ip, robot_port)
                    ps = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
                
                # Show image after detections
                #cv.imshow("undistorted", img_gray)
                #cv.waitKey(0)
                
                # The 'q' button is set as the quitting button you may use any desired button of your choice
                #if cv.waitKey(1) & 0xFF == ord('q'):
                #    break

            #vid.release() # After the loop release the cap object
            cv.destroyAllWindows() # Destroy all the windows             
                

################################################################################

if __name__ == '__main__':
    apriltag_video()