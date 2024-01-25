from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException
import time
import socket
import exceptions
import math
import argparse

### notes
## Copter 3.2.1 needs manual mode switch to GUIDED
## Copter 3.3 can take off in AUTO, provided the mission has a MAV_CMD_NAV_TAKEOFF. 
# the mission will not start until you explicity send MAV_CMD_MISSION_START message.

## HOW TO USE
# python <this file>.py --connect /dev/<connection name> --alt <target altitude>

def debugDump():
    print("Autopilot Firmware version: %s" % vehicle.version)
    print("Autopilot capabilities (supports ftp): %s" % vehicle.capabilities.ftp)
    print("Global Location: %s" % vehicle.location.global_frame)
    print("Global Location (relative altitude): %s" % vehicle.location.global_relative_frame)
    print("Local Location: %s" % vehicle.location.local_frame)    #NED
    print("Attitude: %s" % vehicle.attitude)
    print("Velocity: %s" % vehicle.velocity)
    print("GPS: %s" % vehicle.gps_0)
    print("Groundspeed: %s" % vehicle.groundspeed)
    print("Airspeed: %s" % vehicle.airspeed)
    print("Gimbal status: %s" % vehicle.gimbal)
    print("Battery: %s" % vehicle.battery)
    print("EKF OK?: %s" % vehicle.ekf_ok)
    print("Last Heartbeat: %s" % vehicle.last_heartbeat)
    print("Rangefinder: %s" % vehicle.rangefinder)
    print("Rangefinder distance: %s" % vehicle.rangefinder.distance)
    print("Rangefinder voltage: %s" % vehicle.rangefinder.voltage)
    print("Heading: %s" % vehicle.heading)
    print("Is Armable?: %s" % vehicle.is_armable)
    print("System status: %s" % vehicle.system_status.state)
    print("Mode: %s" % vehicle.mode.name)    # settable
    print("Armed: %s" % vehicle.armed)    # settable
    
def connectMyCopter():
    connection_string = args.connect
    baud_rate = 57600
    vehicle = connect(connection_string, baud=baud_rate, wait_ready=True)
    print("Vehicle connected, dumping vehicle state")
    debugDump()
    return vehicle

def arm():
    while not vehicle.is_armable:
        print("Waiting for vehicle to initialize...")
        time.sleep(1)
    print("Vehicle is initialize/armable.\n")

    targetMode = "GUIDED"
    vehicle.mode = VehicleMode(targetMode)
    vehicle.armed = True
    
    while not vehicle.armed and vehicle.mode.name != targetMode:
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

def disarm():
    print("Disarming vehicle, dumping vehicle state")
    vehicle.armed = False
    vehicle.close()
    debugDump()    

####################################################################################

# Parse arguments
parser = argparse.ArgumentParser(description='commands')
parser.add_argument('--connect')
parser.add_argument('--alt')
args = parser.parse_args()

# Connect to copter
vehicle = connectMyCopter()

# Run flight plan
arm()
time.sleep(5)
takeoff()
time.sleep(5)

# Return to launch
print("Returning to Launch")
vehicle.mode = VehicleMode("RTL")

# Disarm
disarm()
