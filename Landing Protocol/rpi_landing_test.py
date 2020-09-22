from __future__ import division
import numpy as np
import cv2
import cv2.aruco as aruco
import sys, time, math
#import land_handling_final

#--- Define Tag
id_to_find  = 1
marker_size  = 50.75 #- [cm]
#marker_size = 13.7


finalCoords = []
landedFlag = False

# ADD TO LANDING SCRIPT
def getCoords():
    global finalCoords
    return finalCoords

# ADD TO LANDING SCRIPT
def getLandedFlag():
    global landedFlag
    return landedFlag

#------------------------------------------------------------------------------
#------- ROTATIONS https://www.learnopencv.com/rotation-matrix-to-euler-angles/
#------------------------------------------------------------------------------
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

def marker_position_to_angle(x, y, z):

    angle_x = math.atan2(x,z)
    angle_y = math.atan2(y,z)

    return (angle_x, angle_y)


#--- Init
print("enter s to show frame, q to quit")

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




#while third_flag == 0:
def land_handling():
    global finalCoords
    global landedFlag
    global id_to_find
    global marker_size

    filled_True = 0
    z_position = 0
    first_flag = 0
    second_flag = 0
    third_flag = 0
    fourth_flag = 0
    marker_found = 0

    counter = 0
    x_arr = []
    y_arr = []
    z_arr = []
    ret, frame = cap.read()
    cv2.imshow('frame', frame)
    while True:
        #-- Read the camera frame
        ret, frame = cap.read()

        #-- Convert in gray scale
        gray    = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) #-- remember, OpenCV stores color images in Blue, Green, Red

        #-- Find all the aruco markers in the image
        corners, ids, rejected = aruco.detectMarkers(image=gray, dictionary=aruco_dict, parameters=parameters,
                                  cameraMatrix=camera_matrix, distCoeff=camera_distortion)

        #if id_to_find in ids:
        if np.any(ids) and id_to_find in ids:
            ids_list = ids.tolist()

            #index = ids_list.index([id_to_find])
            index = 0
            corners_corr = np.array(corners[index])
            ret = aruco.estimatePoseSingleMarkers(corners_corr, marker_size, camera_matrix, camera_distortion)
        #-- Unpack the output, get only the first
            rvec, tvec = ret[0][0,0,:], ret[1][0,0,:]

            #-- Draw the detected marker and put a reference frame over it
            aruco.drawDetectedMarkers(frame, corners)
            aruco.drawAxis(frame, camera_matrix, camera_distortion, rvec, tvec, 10)

        #-- Store for tvec filled
            filled_True = 1

            if filled_True == 1:
                z_position = tvec[2]

            #if id_to_find in ids:
            if np.any(ids) and id_to_find in ids:
                #-- Print the tag position in camera frame
                str_position = "MARKER %4.0f Position x=%4.0f  y=%4.0f  z=%4.0f"%(id_to_find, tvec[0], tvec[1], tvec[2])
                cv2.putText(frame, str_position, (0, 390), font, 1, (0, 120, 255), 2, cv2.LINE_AA)

                #-- Obtain the rotation matrix tag->camera
                R_ct    = np.matrix(cv2.Rodrigues(rvec)[0])
                R_tc    = R_ct.T

                #-- Get the attitude in terms of euler 321 (Needs to be flipped first)
                roll_marker, pitch_marker, yaw_marker = rotationMatrixToEulerAngles(R_flip*R_tc)

            # compass
                if math.degrees(yaw_marker) >= 0:
                    compass = math.degrees(yaw_marker) + 90
                elif math.degrees(yaw_marker) < 0:
                    compass = (-1) * math.degrees(yaw_marker)
                    if compass < 90:
                        compass = 90 - compass
                    elif compass >= 90:
                        compass = 180 - compass + 270

                #-- Print the marker's attitude respect to camera frame
                #str_attitude = "MARKER Attitude r=%4.0f  p=%4.0f  y=%4.0f"%(math.degrees(roll_marker),math.degrees(pitch_marker), compass)
                #cv2.putText(frame, str_attitude, (0, 405), font, 1, (0, 120, 255), 2, cv2.LINE_AA)

            #-- CORRECTIONS

            if counter < 5:
                # put in arrays
                x_arr.append(tvec[0])
                y_arr.append(tvec[1])
                z_arr.append(tvec[2])
                counter += 1

            if counter == 5:
                counter = 0
                x_avg = sum(x_arr)/len(x_arr)
                y_avg = sum(y_arr)/len(x_arr)
                z_avg = sum(z_arr)/len(z_arr)
                finalCoords = [x_avg, y_avg, z_avg]
                # avg n output n empty arrays
                x_arr = []
                y_arr = []
                z_arr = []

            #angle_x, angle_y    = marker_position_to_angle(-tvec[1], tvec[0], tvec[2])
            #corr_string = str(-tvec[1]) + ", " + str(tvec[0]) + ", " + str(tvec[2]) + "\r\n" #+ ", "+ str(angle_x) + ", " + str(angle_y) + "\r\n"
            #fid.write( corr_string )


        #west fixation
            if compass >= 226 and compass <= 314:
                tvec[1] = tvec[1] * (-1)
                tvec[0] = tvec[0] * (-1)

        #south fixation
            if compass >= 135 and compass <= 225:
                tvec[0] = tvec[0] * (-1)

        #north fixation
            if compass >= 315 or compass <= 45:
                tvec[0] = tvec[0] * (-1)

            if tvec[2] < 100 and first_flag == 0:
                counter += 1
                if counter == 5:
                    id_to_find = 2
                    marker_size = 38
                    #marker_size = 10
                    first_flag = 1
                    counter = 0
                    print("Looking for second marker")


            if tvec[2] < 70 and second_flag == 0 and first_flag == 1:
                counter += 1
                if counter == 5:
                    id_to_find = 3
                    marker_size = 24
                    #marker_size = 6.5
                    second_flag = 1
                    counter = 0
                    print("Looking for third marker")

            if tvec[2] < 40 and third_flag == 0 and second_flag == 1:
                counter += 1  # do i rlly need this??
                if counter == 5:
                    #id_to_find = 4 # ????
                    #marker_size = 1.3
                    third_flag = 1
                    counter = 0
                    print("Beginning position script")
                    finalCoords.append("end")
                    landedFlag = True



            marker_found = 5
            #str_correction = "Current ID=%4.0f"%(id_to_find)
            #v2.putText(frame, str_correction, (0, 465), font,0.85, (0, 0, 255), 2, cv2.LINE_AA)

        key = cv2.waitKey(1) & 0xFF
        #--- Display the frame
        if key == ord('s'):
            cv2.imshow('frame', frame)

        #--- use 'q' to quit

        if key == ord('q'):
            cap.release()
            cv2.destroyAllWindows()

            break

# landing
#cap.release()
#cv2.destroyAllWindows()

#land_handling_final.land_handling()



























