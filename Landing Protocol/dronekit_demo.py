
from __future__ import print_function

import time
import argparse
import math
from dronekit import connect, VehicleMode, LocationGlobalRelative

def arm_and_takeoff(aTargetAltitude):
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

#### Setup for Vehicle ###
connection_string = "/dev/ttyACM0"     # usb conn
#connection_string = "/dev/ttyAMA0"      # serial port
baud_rate = 115200

#--- Now that we have started the SITL and we have the connection string (basically the ip and udp port)...

print(">>>> Connecting with the UAV <<<")
vehicle = connect(connection_string, baud=baud_rate, wait_ready=True)     #- wait_ready flag hold the program untill all the parameters are been read (=, not .)

latlist = []
longlist = []
altlist = []
print("Discrete Altitude Test\n")

max_alt = int(input("Enter your maximum altitude\n"))
stops = int(input("Enter the number of times you would like to stop\n"))
increments = (max_alt) / (stops + 1)
print("Increments of :", increments)
# lat = input("Enter the latitude of your test point:\n")
# latlist.append(lat)
# lon = input("Enter the longitude of your test point:\n")
# longlist.append(lon)

point_alt = max_alt
while (point_alt > 0):
    altlist.append(point_alt)
    print("Adding point at", point_alt)
    point_alt -= increments

speed = 10
t = increments / speed

arm_and_takeoff(max_alt)
print("initial altitude reached")

latlist.append(vehicle.location.global_relative_frame.lat)
longlist.append(vehicle.location.global_relative_frame.lon)

print("Going to test point at", str(latlist[0]) + ",", str(longlist[0]) + ",", str(max_alt))
# point = LocationGlobalRelative(latlist[0], longlist[0], max_alt)
point = LocationGlobalRelative(vehicle.location.global_relative_frame.lat,
                               vehicle.location.global_relative_frame.lon, max_alt)
vehicle.simple_goto(point, groundspeed=speed)

for i in range(len(altlist) - 1):
    print("Going to altitude", altlist[i])
    point = LocationGlobalRelative(latlist[0], longlist[0], altlist[i])
    vehicle.simple_goto(point, groundspeed=speed)
    # sleep so we can see the change in map
    time.sleep(6)

point = LocationGlobalRelative(latlist[0], longlist[0], altlist[0])
vehicle.simple_goto(point, groundspeed=speed)
# sleep so we can see the change in map
time.sleep(t)

print("Returning to Launch")
vehicle.mode = VehicleMode("LAND")

# Close vehicle object before exiting script
print("Close vehicle object")
vehicle.close()

