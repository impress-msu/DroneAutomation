from __future__ import print_function
import threading
import time
import argparse
import socket, json, math, sys, os
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil

data = ['n','n','n']
landedFlag = False
vehicle = 0

def xy_goto(velocity_x, velocity_y, velocity_z=0, duration=1):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_NED, # frame Needs to be MAV_FRAME_BODY_NED for forward/back left/right control.
        0b0000111111000111, # type_mask
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # m/s
        0, 0, 0, # x, y, z acceleration
        0, 0)
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
    c       = math.cos(yaw_rad)
    s       = math.sin(yaw_rad)
    
    north   = x_uav*c - y_uav*s
    east    = x_uav*s + y_uav*c 
    return(north, east)

def get_location_metres(original_location, dNorth, dEast):

    earth_radius=6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    return(newlat, newlon)

def check_angle_descend(angle_x, angle_y, angle_desc):
    return(math.sqrt(angle_x**2 + angle_y**2) <= angle_desc)

def arm_and_takeoff(aTargetAltitude):
    global vehicle
    print ("Basic pre-arm checks")
    # Don't let the user try to arm until autopilot is ready
    while not vehicle.is_armable:
        print ("Waiting for vehicle to initialise...")
        time.sleep(1)

    print ("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print ("Waiting for arming...")
        time.sleep(1)

    print ("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

    # Check that vehicle has reached takeoff altitude
    while True:
        print ("Altitude: ", vehicle.location.global_relative_frame.alt)
        # Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print ("Reached target altitude")
            break
        time.sleep(1)
        
def check_angle_descend(x, y, desc):
    return(math.sqrt(x**2 + y**2) <= desc)

def marker_position_to_angle(x, y, z):
    
    angle_x = math.atan2(x,z)
    angle_y = math.atan2(y,z)
    
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
    vehicle = connect(connection_string, wait_ready=True)
    #vehicle = connect('com19', baud=57600)

    ##- vars and threads
    max_alt = 3 # meters
    #ip = "raspberrypi.local"
    #remote_path = "how_do_drones_work/opencv/camera_01/corrections.txt"
    #local_path = "corrections.txt"

    freq_send           = 1 #- Hz
    land_alt_cm         = 40
    angle_descend       = 20* math.pi / 180
    land_speed_cms      = 30.0

    ##conn_thread.daemon = True

    # for x in range(4):
    #     lat = vehicle.location.global_relative_frame.lat
    #     lon = vehicle.location.global_relative_frame.lon
    #     alt = vehicle.location.global_relative_frame.alt
    #     point = LocationGlobalRelative(lat, lon, alt - 1)
    #     vehicle.simple_goto(point)
    #     while vehicle.location.global_relative_frame.alt > alt - 1:
    #         time.sleep(1)
    #         print("proof we can get stuff from the server: ", data)
    #         pass
    #     print("Current altitude: ", vehicle.location.global_relative_frame.alt)
    #
    # print(" -->>COMMANDING TO LAND<<")
    # vehicle.mode = VehicleMode("LAND")
    # while vehicle.armed:
    #     print("landing")
    #     time.sleep(1)
    # vehicle.close()
    # print("Vehicle closed")

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

    print( "found pad, beginning sequence" )
    time_0 = time.time()
    uav_location = vehicle.location.global_relative_frame
    while(landedFlag == False):

        while(data.count('n') > 0):
            continue

        angles = data # x_cm, y_cm, z_cm
        x_cm = float(angles[0])
        y_cm = float(angles[1])
        z_cm = float(angles[2])
        #print(angles)

        x_ang, y_ang = marker_position_to_angle(x_cm, y_cm, z_cm)
        # do calculations
        if time.time() >= time_0 + 1.0/freq_send:
            time_0 = time.time()

            north, east = uav_to_ne(x_cm, y_cm, vehicle.attitude.yaw * math.pi / 180)
            marker_lat, marker_lon  = get_location_metres(uav_location, north*0.01, east*0.01)

            if check_angle_descend(x_ang, y_ang, angle_descend):
            #    print("Low error: descending")
            #   location_marker = LocationGlobalRelative(marker_lat, marker_lon, uav_location.alt-(land_speed_cms*0.01/freq_send))
                xy_goto(0, 0, 0.01) # in cm maybe??
                xy_goto(0, 0, 0)  # in cm maybe??
            else:
            #    location_marker = LocationGlobalRelative(marker_lat, marker_lon, uav_location.alt)
                xy_goto(-x_cm / 100, -y_cm / 100)  # in cm maybe??
                xy_goto(0, 0, 0) #stop

            #vehicle.simple_goto(location_marker)
            print("UAV Location    Lat = %.7f  Lon = %.7f"%(uav_location.lat, uav_location.lon))
            print("Commanding to   Lat = %.7f  Lon = %.7f"%(location_marker.lat, location_marker.lon))

    
        #if low enough, land
        if z_cm <= land_alt_cm:
            if vehicle.mode == VehicleMode("GUIDED"):
                print(" -->>COMMANDING TO LAND<<")
                vehicle.mode = VehicleMode("LAND")
