from __future__ import print_function
from __future__ import division
import sys
import time
import argparse
import math
import cv2
import cv2.aruco as aruco
import numpy as np
from dronekit import connect, VehicleMode, LocationGlobalRelative


# dronekit variables
landedFlag = False
max_alt = 3 # meters
freq_send           = 1 #- Hz
land_alt_cm         = 40
angle_descend       = 5* math.pi / 180
land_speed_cms      = 30.0

# aruco variables
id_to_find = 1
marker_size = 50.75
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
ret, frame = cap.read()
cv2.imshow('frame',frame)

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

def uav_to_ne(x_uav, y_uav, yaw_rad):
    c       = math.cos(yaw_rad)
    s       = math.sin(yaw_rad)
    
    north   = x_uav*c - y_uav*s
    east    = x_uav*s + y_uav*c 
    return(north, east)

def get_location_metres(dNorth, dEast):

    earth_radius=6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*33/180))

    #New position in decimal degrees
    newlat = 33 + (dLat * 180/math.pi)
    newlon = -88 + (dLon * 180/math.pi)
    return(newlat, newlon)

def check_angle_descend(x, y, z, angle_desc):
    #print("err angle: ", math.atan2(math.sqrt(x**2 + y**2), z)*(180/math.pi))
    return(math.atan2(math.sqrt(x**2 + y**2), z) <= angle_desc)


def marker_position_to_angle(x, y, z):
    
    angle_x = math.atan2(x,z)
    angle_y = math.atan2(y,z)
    
    return (angle_x, angle_y)


# Vehicle setup

connection_string = "/dev/ttyACM0"     # usb conn
#connection_string = "/dev/ttyAMA0"      # serial port
baud_rate = 115200

#--- Now that we have started the SITL and we have the connection string (basically the ip and udp port)...

print(">>>> Connecting with the UAV <<<")
#vehicle = connect(connection_string, baud=baud_rate, wait_ready=True)     #- wait_ready flag hold the program untill all the parameters are been read (=, not .)

# takeoff
#takeoff = int(input("Okay to takeoff?\n"))
#arm_and_takeoff(max_alt)

time.sleep(3)
time_0 = time.time()
#home_location = vehicle.location.global_relative_frame
while(landedFlag == False):
    #id_to_find = 5
    #marker_size = 5
    ret, frame = cap.read()

    #-- Convert in gray scale
    gray    = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) #-- remember, OpenCV stores color images in Blue, Green, Red

    #-- Find all the aruco markers in the image
    corners, ids, rejected = aruco.detectMarkers(image=gray, dictionary=aruco_dict, parameters=parameters,
                              cameraMatrix=camera_matrix, distCoeff=camera_distortion)

    cv2.putText(frame, "o", (315, 245), font,1, (0, 120, 255), 2, cv2.LINE_AA)
    if np.any(ids) and id_to_find in ids:
        ids_list = ids.tolist()

        index = ids_list.index([id_to_find])
        #index = 0
        corners_corr = np.array(corners[index])
        ret = aruco.estimatePoseSingleMarkers(corners_corr, marker_size, camera_matrix, camera_distortion)
    #-- Unpack the output, get only the first
        rvec, tvec = ret[0][0,0,:], ret[1][0,0,:]

        #-- Draw the detected marker and put a reference frame over it
        aruco.drawDetectedMarkers(frame, corners)
        aruco.drawAxis(frame, camera_matrix, camera_distortion, rvec, tvec, 10)


        #-- Print the tag position in camera frame
        #str_position = "MARKER %4.0f Position x=%4.4f  y=%4.4f  z=%4.3f"%(id_to_find, tvec[0], -tvec[1], tvec[2])
        #cv2.putText(frame, str_position, (0, 390), font, 1, (0, 120, 255), 2, cv2.LINE_AA)

        #-- Obtain the rotation matrix tag->camera
        R_ct    = np.matrix(cv2.Rodrigues(rvec)[0])
        R_tc    = R_ct.T

        #-- Get the attitude in terms of euler 321 (Needs to be flipped first)
        roll_marker, pitch_marker, yaw_marker = rotationMatrixToEulerAngles(R_flip*R_tc)

        if math.degrees(yaw_marker) >= 0:
                        compass = math.degrees(yaw_marker)
        elif math.degrees(yaw_marker) < 0:
                compass = (-1) * math.degrees(yaw_marker)
                compass = 360 - compass


        x_cm = tvec[0]
        y_cm = -tvec[1]
        z_cm = tvec[2]
        x_ang, y_ang = marker_position_to_angle(x_cm, y_cm, z_cm)

        # axis corrections
        if (x_cm >= 0 and y_cm >= 0):  # 1st quadrant
            ang_corr = math.radians(0)
            quad = 1
        elif (x_cm < 0 and y_cm >=0):  # 2nd quadrant
            ang_corr = math.radians(90)
            quad = 2
        elif (x_cm < 0 and y_cm <0):   # 3rd quadrant
            ang_corr = math.radians(180)
            quad = 3
        elif (x_cm >= 0 and y_cm < 0): # 4th quadrant
            ang_corr = math.radians(270)
            quad = 4

        # calculations    
        dist = math.sqrt(pow(x_cm,2)+pow(y_cm,2))           # cm distance between marker and center of camera image
        if quad == 2 or quad == 4:                                  # calculation of angle with respect to pos x axis (camera)
            phi = math.atan(float(abs(x_cm))/abs(y_cm))
        else:
            phi = math.atan(float(abs(y_cm))/abs(x_cm))
        phi = phi + ang_corr                                        # placing angle in correct quadrant
        tht = phi - math.radians(compass)                           # correcting from camera angle
        x_1 = math.cos(tht)*dist                               # x coordinate with respect to pad
        y_1 = math.sin(tht)*dist

        north, east = uav_to_ne(x_cm, y_cm, math.radians(360-compass))

        str_position = "MARKER %4.0f Position x=%4.4f  y=%4.4f  z=%4.4f"%(id_to_find, tvec[0], -tvec[1], tvec[2])
        cv2.putText(frame, str_position, (0, 380), font, 1, (0, 120, 255), 2, cv2.LINE_AA)

        str_position = "Calculated x new=%4.4f  y new=%4.4f"%( x_1, y_1)
        cv2.putText(frame, str_position, (0, 400), font, 1, (0, 120, 255), 2, cv2.LINE_AA)


        
        marker_lat, marker_lon  = get_location_metres(north*0.01, east*0.01)
        #print("marker:", id_to_find, "size:", marker_size)
        #print("x:", x_1, "y:", y_1)
        #print("north:", marker_lat-33, "east:", marker_lon+88)
        #print("compass:", compass)

        if check_angle_descend(x_cm, y_cm, z_cm, angle_descend):
            str_position = "LOW ERROR"
            cv2.putText(frame, str_position, (0, 420), font, 1, (0, 120, 255), 2, cv2.LINE_AA)

        else:
            str_position = "HIGH ERROR"
            cv2.putText(frame, str_position, (0, 420), font, 1, (0, 120, 255), 2, cv2.LINE_AA)

        if tvec[2] >= 200:
                id_to_find = 1
                marker_size = 50.75
        elif tvec[2] <= 200 and tvec[2] > 100:
                id_to_find = 2
                marker_size = 38
        elif tvec[2] <= 100  and tvec[2] > 60:
                id_to_find = 3
                marker_size = 24.5

# less than 40, pick a value

    key = cv2.waitKey(1) & 0xFF
    #--- Display the frame
    #--- use 'q' to quit
    cv2.imshow('frame',frame)
    if key == ord('q'):
        cap.release()
        cv2.destroyAllWindows()