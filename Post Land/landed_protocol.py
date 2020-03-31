"""
This demo calculates multiple things for different scenarios.

Here are the defined reference frames:

TAG:
                A y
                |
                |
                |tag center
                O---------> x

CAMERA:


                X--------> x
                | frame center
                |
                |
                V y

F1: Flipped (180 deg) tag frame around x axis
F2: Flipped (180 deg) camera frame around x axis

The attitude of a generic frame 2 respect to a frame 1 can obtained by calculating euler(R_21.T)

We are going to obtain the following quantities:
    > from aruco library we obtain tvec and Rct, position of the tag in camera frame and attitude of the tag
    > position of the Camera in Tag axis: -R_ct.T*tvec
    > Transformation of the camera, respect to f1 (the tag flipped frame): R_cf1 = R_ct*R_tf1 = R_cf*R_f
    > Transformation of the tag, respect to f2 (the camera flipped frame): R_tf2 = Rtc*R_cf2 = R_tc*R_f
    > R_tf1 = R_cf2 an symmetric = R_f


"""
import numpy as np
import cv2
import cv2.aruco as aruco
import sys, time, math
from aruco_pos import aruco_pos
from aruco_coord import aruco_coord

cen_offset = 0 # offset in cm
comp_offset = 0


coord_fin = ['n', 'n', 'n']
landedFlag = False

# ADD TO LANDING SCRIPT
def getCoords():
    global coord_fin
    return coord_fin

# ADD TO LANDING SCRIPT
def getLandedFlag():
    global landedFlag
    return landedFlag


# Checks if a matrix is a valid rotation matrix.
def isRotationMatrix(R):
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype=R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6


# Calculates rotation matrix to euler angles
# The result is the same as MATLAB except the order
# of the euler angles ( x and z are swapped ).
def rotationMatrixToEulerAngles(R):
    assert (isRotationMatrix(R))

    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

    singular = sy < 1e-6

    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0

    return np.array([x, y, z])



def landed_protocol():
    global coord_fin
    #--- Define Tag
    #id_to_find  = 1
    marker_size = 5
    #marker_size  = 5  #- [cm]

    #--- Get the camera calibration path
    calib_path  = ""
    camera_matrix   = np.loadtxt(calib_path+'cameraMatrix.txt', delimiter=',')
    camera_distortion   = np.loadtxt(calib_path+'cameraDistortion.txt', delimiter=',')

    #--- 180 deg rotation matrix around the x axis
    R_flip  = np.zeros((3,3), dtype=np.float32)
    R_flip[0,0] = 1.0
    R_flip[1,1] =-1.0
    R_flip[2,2] =-1.0

    #--- Define the aruco dictionary
    aruco_dict  = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
    parameters  = aruco.DetectorParameters_create()

    #--- Capture the videocamera (this may also be a video or a picture)
    cap = cv2.VideoCapture(0)

    #-- Set the camera size as the one it was calibrated with
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    #-- Font for the text in the image
    font = cv2.FONT_HERSHEY_PLAIN

    #-- Correction Outputs
    turn_left = 0
    turn_right = 0
    flip_up = 0
    flip_down = 0
    closer = 0
    farther = 0
    x_center = 0
    y_center = 0
    z_center = 0
    filled_True = 0
    z_position = 0
    marker_found = 0


    while True:

        #-- Read the camera frame
        ret, frame = cap.read()

        #-- Convert in gray scale
        gray    = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) #-- remember, OpenCV stores color images in Blue, Green, Red

        #-- Find all the aruco markers in the image
        corners, ids, rejected = aruco.detectMarkers(image=gray, dictionary=aruco_dict, parameters=parameters,
                                  cameraMatrix=camera_matrix, distCoeff=camera_distortion)

        if np.any(ids):
            #-- ret = [rvec, tvec, ?]
            #-- array of rotation and position of each marker in camera frame
            #-- rvec = [[rvec_1], [rvec_2], ...]    attitude of the marker respect to camera frame
            #-- tvec = [[tvec_1], [tvec_2], ...]    position of the marker in camera frame
            # taking out 1 2 and 3 - just the 5cms
            indices = []
            ids_list = ids.tolist()
            if 1 in ids:
                indices.append(ids_list.index([1]))

            if 2 in ids:
                indices.append(ids_list.index([2]))

            if 3 in ids:
                indices.append(ids_list.index([3]))

            corners_corr = list(corners)
            if len(indices) != 0:
                for x in indices:
                    corners_corr.pop(x)
            if len(corners_corr) == 0:
                continue


            ret = aruco.estimatePoseSingleMarkers(corners, marker_size, camera_matrix, camera_distortion)

            #-- Unpack the output, get only the first
            rvec, tvec = ret[0][0,0,:], ret[1][0,0,:]

            #-- Draw the detected marker and put a reference frame over it
            aruco.drawDetectedMarkers(frame, corners)
            #aruco.drawAxis(frame, camera_matrix, camera_distortion, rvec, tvec, 10)
            tx = tvec[0]
            ty = tvec[1]
            tvec[0] = 0
            tvec[1] = 0
            aruco.drawAxis(frame,camera_matrix, camera_distortion, rvec, tvec, 10)
            tvec[0] = tx
            tvec[1] = ty
    #-- Store for tvec filled
            filled_True = 1

        if filled_True == 1:
            z_position = tvec[2]

        if np.any(ids):
            #-- Print the tag position in camera frame
            str_position = "MARKER Position x=%4.0f  y=%4.0f  z=%4.0f"%(tvec[0], tvec[1]*-1, tvec[2])
            cv2.putText(frame, str_position, (0, 390), font, 1, (0, 120, 255), 2, cv2.LINE_AA)


            #-- Obtain the rotation matrix tag->camera
            R_ct    = np.matrix(cv2.Rodrigues(rvec)[0])
            R_tc    = R_ct.T

            #-- Get the attitude in terms of euler 321 (Needs to be flipped first)
            roll_marker, pitch_marker, yaw_marker = rotationMatrixToEulerAngles(R_flip*R_tc)

            #compass
            if math.degrees(yaw_marker) >= 0:
                    compass = math.degrees(yaw_marker)
            elif math.degrees(yaw_marker) < 0:
                    compass = (-1) * math.degrees(yaw_marker)
                    compass = 360 - compass
                    #if compass < 90:
                    #	compass = 360 - compass
                    #elif compass >= 90:
                    #	compass = 180 - compass + 270
            #compass = math.degrees(yaw_marker)
            compass = compass - comp_offset
            #-- Print the marker's attitude respect to camera frame
            str_attitude = "MARKER Attitude r=%4.0f  p=%4.0f  y=%4.0f"%(math.degrees(roll_marker),math.degrees(pitch_marker),
                                compass)
            cv2.putText(frame, str_attitude, (0, 405), font, 1, (0, 120, 255), 2, cv2.LINE_AA)

            #-- Print out correction
            #-- X correction
            #if math.degrees(pitch_marker) < -2:
            #        turn_left = 1
            #        turn_right = 0
            #        x_center = 0
            #elif math.degrees(pitch_marker) > -2 and math.degrees(pitch_marker) < 2:
            #        turn_left = 0
            #        turn_right = 0
            #        x_center = 1
            #elif math.degrees(pitch_marker) > 2:
            #        turn_left = 0
            #        turn_right = 1
            #        x_center = 0

            #str_correction = "Turn Left=%4.0f Turn Right=%4.0f Centered=%4.0f"%(turn_left, turn_right, x_center)
            #cv2.putText(frame, str_correction, (0, 420), font, 0.85, (225, 255, 255), 2, cv2.LINE_AA)

            #-- Y correction
            #if math.degrees(roll_marker) < -2:
            #        turn_down = 1
            #        turn_up = 0
            #        y_center = 0
            #elif math.degrees(roll_marker) > -2 and math.degrees(roll_marker) < 2:
            #        turn_down = 0
            #        turn_up = 0
            #        y_center = 1
            #elif math.degrees(roll_marker) > 2:
            #        turn_down = 0
            #        turn_up = 1
            #        y_center = 0
            #str_correction = "Turn Down=%4.0f Turn Up=%4.0f Centered=%4.0f"%(turn_down, turn_up, y_center)
            #cv2.putText(frame, str_correction, (0, 435), font,0.85, (255, 255, 255), 2, cv2.LINE_AA)

            #-- Z correction
            #if tvec[2] >  52:
            #        closer = 1
            #        further = 0
            #        z_center = 0
            #elif tvec[2] > 48  and tvec[2] < 52:
            #        closer = 0
            #        further = 0
            #        z_center = 1
            #elif tvec[2] < 48:
            #        closer = 0
            #        further = 1
            #        z_center = 0

            #west fixation
            #if compass >= 226 and compass <= 314:
                    #tvec[1] = tvec[1] * (-1)
                    #tvec[0] = tvec[0] * (-1)

            #south fixation
            #if compass >= 135 and compass <= 225:
                    #tvec[0] = tvec[0] * (-1)

            #north fixation
            #if compass >= 315 or compass <= 45:
                    #tvec[0] = tvec[0] * (-1)

    ##            if id_to_find < 11:
    ##                    y_pos = tvec[1] + 17.5
    ##            elif id_to_find < 21:
    ##                    y_pos = tvec[1] + 12.5
    ##            elif id_to_find < 31:
    ##                    y_pos = tvec[1] + 7.5
    ##            elif id_to_find < 41:
    ##                    y_pos = tvec[1] + 2.5
    ##            elif id_to_find < 51:
    ##                    y_pos = tvec[1] - 2.5
    ##            elif id_to_find < 61:
    ##                    y_pos = tvec[1] - 7.5
    ##            elif id_to_find < 71:
    ##                    y_pos = tvec[1] - 12.5
    ##            elif id_to_find < 81:
    ##                    y_pos = tvec[1] - 17.5
    ##            else:
    ##                y_pos = tvec[1]
    ##
    ##            x_pos = tvec[0]
    ##            if (id_to_find % 10) == 1:
    ##                    x_pos = x_pos + 22.5
    ##            elif (id_to_find % 10) == 2:
    ##                    x_pos = x_pos + 17.5
    ##            elif (id_to_find % 10) == 3:
    ##                    x_pos = x_pos + 12.5
    ##            elif (id_to_find % 10) == 4:
    ##                    x_pos = x_pos + 7.5
    ##            elif (id_to_find % 10) == 5:
    ##                    x_pos = x_pos + 2.5
    ##            elif (id_to_find % 10) == 6:
    ##                    x_pos = x_pos - 2.5
    ##            elif (id_to_find % 10) == 7:
    ##                    x_pos = x_pos - 7.5
    ##            elif (id_to_find % 10) == 8:
    ##                    x_pos = x_pos - 12.5
    ##            elif (id_to_find % 10) == 9:
    ##                    x_pos = x_pos - 17.5
    ##            elif (id_to_find % 10) == 0:
    ##                    x_pos = x_pos - 22.5


            str_correction = "Compass=%4.0f"%(compass)
            cv2.putText(frame, str_correction, (0, 435), font,1, (0, 120, 255), 2, cv2.LINE_AA)
            if(tvec[2] <= 26):
                center_id = 0
                ##-- Landed Handling

                # setup
                marker_dist = 8.5 # SHOULDNT NEED THIS!
                #marker_dist = 5
                if ids[0][0] != 3 and ids[0][0] != 2 and ids[0][0] != 1 and ids[0][0] != 0:# cm distance between marker centers
                    
                    coords = aruco_pos[ ids[0][0] ]   # location on board of marker
                    #coords = aruco_coord[ ids[0][0] ]   # location on board of marker
                    x_marker = tvec[0]                  # x coordinate on camera image
                    y_marker = -tvec[1]                 # y coordinate on camera image

                    # axis corrections
                    if (x_marker >= 0 and y_marker >= 0):  # 1st quadrant
                        ang_corr = math.radians(0)
                        quad = 1
                    elif (x_marker < 0 and y_marker >=0):  # 2nd quadrant
                        ang_corr = math.radians(90)
                        quad = 2
                    elif (x_marker < 0 and y_marker <0):   # 3rd quadrant
                        ang_corr = math.radians(180)
                        quad = 3
                    elif (x_marker >= 0 and y_marker < 0): # 4th quadrant
                        ang_corr = math.radians(270)
                        quad = 4

                    # calculations    
                    dist = math.sqrt(pow(x_marker,2)+pow(y_marker,2))           # cm distance between marker and center of camera image
                    if quad == 2 or quad == 4:                                  # calculation of angle with respect to pos x axis (camera)
                        phi = math.atan(float(abs(x_marker))/abs(y_marker))
                    else:
                        phi = math.atan(float(abs(y_marker))/abs(x_marker))
                    phi = phi + ang_corr                                        # placing angle in correct quadrant
                    tht = phi - math.radians(compass)                           # correcting from camera angle
                    x_marker = math.cos(tht)*dist                               # x coordinate with respect to pad
                    y_marker = math.sin(tht)*dist                               # y coordinate with respect to pad
                    #x_5cm = x_marker/marker_dist                                # x coordinate in unit of markers
                    #y_5cm = y_marker/marker_dist                                # y coordinate in unit of markers
                    coord_f = [0, 0]
                    coord_f[0] = coords[0]*marker_dist - x_marker                              # final coordinates with caluclated corrections
                    coord_f[1] = coords[1]*marker_dist - y_marker

                    #for key, value in aruco_coord.items():
                        #if value == coord_f:
                            #center_id = key
                            
                    
                    
                    if cen_offset != 0:
                        # more calculations
                        phi = math.radians(compass)
                        x_off = math.sin(phi)*cen_offset
                        y_off = math.cos(phi)*cen_offset
                        #x_5off = x_off/marker_dist
                        #y_5off = y_off/marker_dist
                        coord_fin[0] = coord_f[0] - x_off                             # final coordinates with caluclated corrections
                        coord_fin[1] = coord_f[1] - y_off
                        coord_fin[2] = compass
                    else:
                        coord_fin[0] = coord_f[0] # final coordinates with caluclated corrections
                        coord_fin[1] = coord_f[1]
                        coord_fin[2] = compass

                    #for key, value in aruco_pos.items():
                    ##for key, value in aruco_coord.items():
                        #if value == coord_f:
                            #center_id = key
                    
                    str_land = "Ref ID: %4.0f pos: %4.0f cm, %4.0f cm "%(ids[0][0], coord_f[0], coord_f[1])
                    cv2.putText(frame, str_land, (0, 150), font, 1, (0, 0, 255), 2, cv2.LINE_AA)
                    landedFlag = True
                    #str_debug = "DEBUG start pos: %4.0f, %4.0f  end pos: %4.0f,%4.0f 5cm coords:%4.0f, %4.0f"%(coords[0], coords[1], coord_f[0], coord_f[1], x_5cm, y_5cm)
                    #str_debug2 = "DEBUG xmarker: %4.0f ymarker: %4.0f  quadrant: %4.0f phi: %4.4f"%(x_marker, y_marker, quad, math.degrees(phi))
                    #cv2.putText(frame, str_debug, (0, 150), font, 1, (0, 0, 255), 2, cv2.LINE_AA)
                    #cv2.putText(frame, str_debug2, (0, 170), font, 1, (0, 0, 255), 2, cv2.LINE_AA)
                    

            marker_found = 5
            #str_correction = "Closer=%4.0f Further=%4.0f Centered=%4.0f"%(closer, further, z_center)
            #cv2.putText(frame, str_correction, (0, 450), font,0.85, (255, 255, 255), 2, cv2.LINE_AA)
            #str_correction = "Current ID=%4.0f"%(id_to_find)
            #cv2.putText(frame, str_correction, (0, 465), font,0.85, (255, 255, 255), 2, cv2.LINE_AA)

    ##        else:
    ##            if marker_found == 0:
    ##                    id_to_find = id_to_find + 1
    ##            else:
    ##                    marker_found = marker_found - 1
    ##            if id_to_find > 155:
    ##                    id_to_find = 0

        #--- Display the frame
        cv2.imshow('frame', frame)

        #--- use 'q' to quit
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            cap.release()
            cv2.destroyAllWindows()
            break































