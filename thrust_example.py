from __future__ import print_function

from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil # Needed for command message definitions
import math
import time
import argparse

"""
HOW TO USE:
python <this file>.py --connect /dev/<connection name>
ex: python thrust_example.py --connect /dev/ttyACM0

DEVELOPER NOTES:
- Reference link: https://hackmd.io/@willy541222/S15ALdL5d
- vehicle.is_armable checks if the vehicle is initializing and has a stable GPS connection
- use vehicle.mode != 'INITIALISING' for initialization check
"""

MAX_THRUST = 1.5

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
    print("Attempting to connect to vehicle: %s" % connection_string)
    vehicle = connect(connection_string, wait_ready=True, baud=57600)
    print("Vehicle connected, dumping vehicle state")
    return vehicle

def arm():
    
    while vehicle.mode == 'INITIALISING':
        print("Waiting for drone to initialize")
        time.sleep(1)
    
    print("Drone has successfully initialized")
    print("GPS Connection: %s" % vehicle.is_armable)
    
    try:
        vehicle.mode = VehicleMode("GUIDED_NOGPS")
    except:
        print("Could not set vehicle mode to GUIDED_NOGPS")
    
    print("Arming Vehicle now")
    vehicle.armed = True
    
    while not vehicle.armed:
        print("Waiting for drone to become armed...")
        time.sleep(1)

    print("Vehicle is now armed.")
    
def thrust(thrust):
    if thrust > MAX_THRUST or thrust < 0:
        print("Invalid thrust request: %s" % thrust)
        return
    # TODO: Apply thrust via message factory

def disarm():
    print("Disarming vehicle, dumping vehicle state")
    vehicle.armed = False
    vehicle.close()
    debugDump()    

####################################################################################

# Parse arguments
parser = argparse.ArgumentParser(description='commands')
parser.add_argument('--connect')
args = parser.parse_args()

# Connect to copter
vehicle = connectMyCopter()

# dump inital state
debugDump()

# Run flight plan
try:
    arm()
    time.sleep(10)
finally:
    disarm()
