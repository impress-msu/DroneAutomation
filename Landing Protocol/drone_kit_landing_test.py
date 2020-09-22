from __future__ import print_function
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time
import argparse

com_port = 'com23'
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

def send_ned_velocity(velocity_x, velocity_y, velocity_z, duration):
    """
    Move vehicle in direction based on specified velocity vectors.
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)


    # send command to vehicle on 1 Hz cycle
    for x in range(0,duration):
        vehicle.send_mavlink(msg)
        time.sleep(1)


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
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

    # Check that vehicle has reached takeoff altitude
    while True:
        print("Altitude: ", vehicle.location.global_relative_frame.alt)
        # Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

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
print('Connecting to vehicle on: %s' % connection_string)
# vehicle = connect(connection_string, wait_ready=True)

vehicle = connect(com_port, baud=57600)

while not vehicle.is_armable:
    print("Waiting for vehicle to initialise...")
    time.sleep(1)

print("switching mode")

# Copter should arm in GUIDED mode
vehicle.mode = VehicleMode("GUIDED")
while(vehicle.mode != VehicleMode("GUIDED")):
    print("waiting for mode change")


aTargetAltitude = vehicle.location.global_relative_frame.alt + 1 #meter

# while True:
#     print("Altitude: ", vehicle.location.global_relative_frame.alt)
#     # Break and return from function just below target altitude.
#     if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
#         print("Reached target altitude")
#         send_ned_velocity(0,0,0,1)
#         break
#     send_ned_velocity(0,0,-0.5,1)
#     #send_global_velocity(0,0,0,10)
#     time.sleep(1)

arm_and_takeoff(1)
print("Altitude: ", vehicle.location.global_relative_frame.alt)

vehicle.mode = VehicleMode("ALT_HOLD")
while(vehicle.mode != VehicleMode("ALT_HOLD")):
    print("waiting for mode change")
print("Testing directional input")
inp = 0
while inp != "":
    inp = raw_input("")
    if inp == "w":
        print("forward")
        send_ned_velocity(0.05, 0, 0, 1)
        send_ned_velocity(0, 0, 0, 1)
    elif inp == "a":
        print("left")
        send_ned_velocity(0, -0.05, 0, 1)
        send_ned_velocity(0, 0, 0, 1)
    elif inp == "s":
        print("back")
        send_ned_velocity(-0.05, 0, 0, 1)
        send_ned_velocity(0, 0, 0, 1)
    elif inp == "d":
        print("right")
        send_ned_velocity(0, 0.05, 0, 1)
        send_ned_velocity(0, 0, 0, 1)

vehicle.mode = VehicleMode("GUIDED")
while(vehicle.mode != VehicleMode("GUIDED")):
    print("waiting for mode change")


print(" -->>COMMANDING TO LAND<<")
vehicle.mode = VehicleMode("LAND")
vehicle.close()


#max_alt = 2  # meters
# arm_and_takeoff(max_alt)
# get = raw_input("Altitude reached, good to go?")
# print("lowering 0.5 meters")
# send_global_velocity(0, 0, float(-(0.5) / 2), 2)
# send_global_velocity(0, 0, 0, 1)
# get = raw_input("good to go?")
# print("lowering 0.5 meters")
# send_global_velocity(0, 0, float(-(0.5) / 2), 2)
# send_global_velocity(0, 0, 0, 1)
2
# get = raw_input("good to go?")
# print("landing")
# vehicle.mode = VehicleMode("LAND")
