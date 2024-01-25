from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException
import time
import argparse

"""
HOW TO USE:
python <this file>.py --connect /dev/<connection name> --alt <target altitude>
ex: python arm_test.py --connect /dev/ttyACM0 --alt 10

DEVELOPER NOTES:
- This file should work as a living template to create a python flight plan
- vehicle.is_armable checks if the vehicle is initializing and has a stable GPS connection
- use vehicle.mode != 'INITIALISING' for initialization check
"""

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
    
    print("Arming Vehicle now")
    vehicle.armed = True
    
    while not vehicle.armed:
        print("Waiting for drone to become armed...")
        time.sleep(1)

    print("Vehicle is now armed.")

def disarm():
    print("Disarming vehicle, dumping vehicle state")
    vehicle.armed = False
    vehicle.close()
    debugDump()  
     
####################################################################################
# Write your flight plan here

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
        
def do_flight():
    arm()
    takeoff()
    time.sleep(10)

####################################################################################

# Create argument parser
parser = argparse.ArgumentParser(description='commands')

# Add Arguments here
parser.add_argument('--connect')
parser.add_argument('--alt')

# Parse argument
args = parser.parse_args()

# Connect to copter
vehicle = connectMyCopter()

# dump inital state
debugDump()

# Run flight plan
try:
    do_flight()
except Exception as e:
    print("ERROR!")
    print(e)
finally:
    disarm()
