from __future__ import print_function
import threading
import time
import argparse
from pymavlink import mavutil
import socket, json, math, sys, os
from dronekit import connect, VehicleMode, LocationGlobalRelative

data = ['n','n','n', False]
landedFlag = False
vehicle = 0

def send_global_velocity(velocity_x, velocity_y, velocity_z, duration):
    """
    Move vehicle in direction based on specified velocity vectors.
    """
    msg = vehicle.message_factory.set_position_target_global_int_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, # lat_int - X Position in WGS84 frame in 1e7 * meters
        0, # lon_int - Y Position in WGS84 frame in 1e7 * meters
        0, # alt - Altitude in meters in AMSL altitude(not WGS84 if absolute or relative)
        # altitude above terrain if GLOBAL_TERRAIN_ALT_INT
        velocity_x, # X velocity in NED frame in m/s
        velocity_y, # Y velocity in NED frame in m/s
        velocity_z, # Z velocity in NED frame in m/s
        0, 0, 0, # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

    # send command to vehicle on 1 Hz cycle
    for x in range(0,duration):
        vehicle.send_mavlink(msg)
        time.sleep(1)


def setData(newData):
    global data
    data = newData
    return


def setLandedFlag(flag):
    global landedFlag
    landedFlag = flag
    return


def uav_to_ne(x_uav, y_uav, yaw_rad):
    c = math.cos(yaw_rad)
    s = math.sin(yaw_rad)

    north = x_uav * c - y_uav * s
    east = x_uav * s + y_uav * c
    return (north, east)


def get_location_metres(original_location, dNorth, dEast):
    earth_radius = 6378137.0  # Radius of "spherical" earth
    # Coordinate offsets in radians
    dLat = dNorth / earth_radius
    dLon = dEast / (earth_radius * math.cos(math.pi * original_location.lat / 180))

    # New position in decimal degrees
    newlat = original_location.lat + (dLat * 180 / math.pi)
    newlon = original_location.lon + (dLon * 180 / math.pi)
    return (newlat, newlon)


def check_angle_descend(angle_x, angle_y, angle_desc):
    return (math.sqrt(angle_x ** 2 + angle_y ** 2) <= angle_desc)


# def arm_and_takeoff(aTargetAltitude):
#     global vehicle
#     print("Basic pre-arm checks")
#     # Don't let the user try to arm until autopilot is ready
#     while not vehicle.is_armable:
#         print("Waiting for vehicle to initialise...")
#         time.sleep(1)
#
#     print("Arming motors")
#     # Copter should arm in GUIDED mode
#     vehicle.mode = VehicleMode("GUIDED")
#     vehicle.armed = True
#
#     while not vehicle.armed:
#         print("Waiting for arming...")
#         time.sleep(1)
#
#     print("Taking off!")
#     vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude
#
#     # Check that vehicle has reached takeoff altitude
#     while True:
#         print("Altitude: ", vehicle.location.global_relative_frame.alt)
#         # Break and return from function just below target altitude.
#         if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
#             print("Reached target altitude")
#             break
#         time.sleep(1)

def arm_and_takeoff(aTargetAltitude):
    global vehicle
    print("Basic pre-arm checks")
    # Don't let the user try to arm until autopilot is ready
    while not vehicle.is_armable:
        print("Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print("Waiting for arming...")
        time.sleep(1)

    print("Taking off!")

    send_global_velocity(0,0,(aTargetAltitude*0.95)/3,3)


def check_angle_descend(x, y, desc):
    return (math.sqrt(x ** 2 + y ** 2) <= desc)


def marker_position_to_angle(x, y, z):
    angle_x = math.atan2(x, z)
    angle_y = math.atan2(y, z)

    return (angle_x, angle_y)


##- Setup for vehicle

def runVehicle():
    global vehicle

    parser = argparse.ArgumentParser(description='Commands vehicle using vehicle.simple_goto.')
    parser.add_argument('--connect',
                        help="Vehicle connection target string. If not specified, SITL automatically started and used.")
    args = parser.parse_args()

    connection_string = args.connect
    sitl = None

    # Start SITL if no connection string specified
    if not connection_string:
        import dronekit_sitl
        sitl = dronekit_sitl.start_default()
        connection_string = sitl.connection_string()

    # Connect to the Vehicle
    print('Connecting to vehicle on: %s' % connection_string)
    #vehicle = connect(connection_string, wait_ready=True)
    vehicle = connect('com21', baud=57600)

    ##- vars and threads
    max_alt = 1.5  # meters


    freq_send = 1  # - Hz
    land_alt_cm = 40
    angle_descend = 20 * math.pi / 180
    land_speed_cms = 30.0

    ##conn_thread.daemon = True

    ##- actual script

    home_position = (vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon)

    ##arm_and_takeoff(max_alt)
    ##print("max altitude reached")
    print("taking control in 3 seconds")
    time.sleep(3)
    print("beginning landing sequence")
    # LANDING!!

    lat = vehicle.location.global_relative_frame.lat
    lon = vehicle.location.global_relative_frame.lon

    '''
    while len(threading.enumerate()) != 1:
        alt = vehicle.location.global_relative_frame.alt
        if alt < 1:
            vehicle.mode = VehicleMode("LAND") # if it gets too low, land it
            vehicle.close()
            sys.exit(0)
        point = LocationGlobalRelative(lat, lon, alt-0.10)
        vehicle.simple_goto(point)
        time.sleep(3) # wait 3 seconds
        '''

    ##print("found pad, beginning sequence")
    time_0 = time.time()
    uav_location = vehicle.location.global_relative_frame
    prev_data = 0
    no_marker_threshold = 10
    velocity = 1  # m/s
    z_vel = 30  # cm descent distance
    rad_error = 1  # cm radius of error
    while (landedFlag == False):

        while (data.count('n') > 0):
            continue

        if(prev_data != data) :
            # means marker found if data changes
            prev_data = angles
            angles = data # x_cm, y_cm, z_cm
            x_cm = float(angles[0])  # cm x distance from marker center
            y_cm = float(angles[1])  # cm y distance from marker center
            z_cm = float(angles[2])  # cm z distance from marker center
            no_marker_cnt = 0

        else:
            print("cant see marker, going back")
            x_cm = -1 * prev_data[0]
            y_cm = -1 * prev_data[1]
            no_marker_cnt += 1

        #if (no_marker_cnt >= no_marker_threshold):
         #   vehicle.goto(home_position) # CHANGE TO NOT GPS FOR TESTING??
         #   no_marker_threshold = 0


        xy_error = math.sqrt(x_cm ** 2 + y_cm ** 2)  # find distance from center of marker

        if (xy_error <= rad_error):
            print("error small enough, lowering drone")
            send_global_velocity(0, 0, float(z_vel/100), 1)  # decrease at constant rate
            send_global_velocity(0, 0, 0, 1)
        else:
            x_v = 0
            y_v = 0
            if(abs(x_cm) > 1):
                x_v = x_cm
            if(abs(y_cm) > 1):
                y_v = y_cm
            send_global_velocity(float(x_v/100), float(y_v/100), 0, 1)
            send_global_velocity(0, 0, 0, 1)


        print("UAV Location    x = %.7f  y = %.7f" % (x_cm, y_cm))

        # if low enough, land
        if z_cm <= land_alt_cm:
            print("Vehicle low enough, proceeding to land")
            if vehicle.mode == "GUIDED":
                print(" -->>COMMANDING TO LAND<<")
                vehicle.mode = "LAND"
