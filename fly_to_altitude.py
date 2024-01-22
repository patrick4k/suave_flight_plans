from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException
import time
import socket
import exceptions
import math
import argparse

# notes
"""
- Copter 3.2.1 needs manual mode switch to GUIDED
- Copter 3.3 can take off in AUTO, provided the mission has a MAV_CMD_NAV_TAKEOFF. the mission will not start until you explicity send MAV_CMD_MISSION_START message.
"""

## HOW TO USE
# python <this file>.py --connect /dev/<connection name> --alt <target altitude>

def connectMyCopter():
    connection_string = args.connect
    baud_rate = 57600

    vehicle = connect(connection_string, baud=baud_rate, wait_ready=True)
    return vehicle

def arm():
    
    while not vehicle.is_armable:
        print("Waiting for vehicle to initialize...")
        time.sleep(1)
    
    print("Vehicle is initialize/armable.\n")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed=True
    
    while not vehicle.armed:
        print("Waiting for drone to become armed...")
        time.sleep(1)

    print("Vehicle is now armed.")

def takeoff():
    targetAlt = float(args.alt)
    vehicle.simple_takeoff(targetAlt)

    while True:
        currAlt = vehicle.location.global_relative_frame.alt
        print("Altitude = ", currAlt)
        if currAlt >= 0.95*targetAlt:
            print("Reached target altitude")
            break
        time.sleep(0.5)


parser = argparse.ArgumentParser(description='commands')
parser.add_argument('--connect')
parser.add_argument('--alt')
args = parser.parse_args()

vehicle = connectMyCopter()
arm()
takeoff()
print("End of flight plan")
