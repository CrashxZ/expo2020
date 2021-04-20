#!/usr/bin/env python
from __future__ import print_function
import timeimport sys
from dronekit import connect, VehicleMode, LocationGlobalRelative
#change to /tty for physical connection
connection_string = "udpin:0.0.0.0:14550"
lattitude = -35.3639614
longitude = 149.1650408
altitude = 20
# Connect to the Vehicle
print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True)


def arm_and_takeoff(aTargetAltitude):
    print("Basic pre-arm checks") 
    # Don't try to arm until autopilot is ready 
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)
 print("Arming motors")
 # Copter should arm in GUIDED mode 
 vehicle.mode = VehicleMode("GUIDED")
 vehicle.armed = True
 # Confirm vehicle armed before attempting to take off 
 while not vehicle.armed:
     print(" Waiting for arming...")
     time.sleep(1)
 print("Taking off!")
 vehicle.simple_takeoff(aTargetAltitude)
 # Take off to target altitude 
 while True:
     print(" Altitude: ", vehicle.location.global_relative_frame.alt)
     # Break and return from function just below target altitude.
     if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
         print("Reached target altitude") break time.sleep(1)

#wait for sattelite fix
print("Waiting for GNSS fix. Number of Sattelites = ")
print(vehicle.gps_0.satellites_visible)
while(vehicle.gps_0.satellites_visible<=3):
    print("Waiting for GNSS fix. Number of Sattelites = ")
    print(vehicle.gps_0.satellites_visible)
    time.sleep(1)
# get home coordinates
homeLat = vehicle.location.global_relative_frame.lat
homeLong = vehicle.location.global_relative_frame.lon

arm_and_takeoff(10)
vehicle.airspeed = 3
print("Going towards : "+ str(lattitude)+","+str(longitude))
point1 = LocationGlobalRelative(lattitude, longitude, altitude)
vehicle.simple_goto(point1)
time.sleep(5)
while(vehicle.airspeed >= 0.5):
    print("Moving Towards Waypoint")
    print(vehicle.airspeed)
    time.sleep(10)
print("Landing")
vehicle.mode = VehicleMode("LAND")
while True:
    print(" Altitude: ", vehicle.location.global_relative_frame.alt) 
    #Break and return from function just below target altitude.
    if vehicle.location.global_relative_frame.alt <= 0.3:
        print("Landed")
        break
    time.sleep(1)

time.sleep(10)
arm_and_takeoff(10)
print("Going Home")
home = LocationGlobalRelative(homeLat, homeLong, 20)
vehicle.simple_goto(home, groundspeed=10)

print("Exiting Program")
vehicle.close()
