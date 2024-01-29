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

def disarm():
    print("Disarming vehicle, dumping vehicle state")
    vehicle.armed = False
    vehicle.close()
    debugDump()    

####################################################################################

def print_ned(cycles = 10, period = 1):
    for i in range(cycles):
        print("NED: %s" % vehicle.location.local_frame)
        time.sleep(period)

def goto_position_target_local_ned(north, east, down):
    """	
    Send SET_POSITION_TARGET_LOCAL_NED command to request the vehicle fly to a specified 
    location in the North, East, Down frame.

    It is important to remember that in this frame, positive altitudes are entered as negative 
    "Down" values. So if down is "10", this will be 10 metres below the home altitude.

    Starting from AC3.3 the method respects the frame setting. Prior to that the frame was
    ignored. For more information see: 
    http://dev.ardupilot.com/wiki/copter-commands-in-guided-mode/#set_position_target_local_ned

    See the above link for information on the type_mask (0=enable, 1=ignore). 
    At time of writing, acceleration and yaw bits are ignored.

    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111111000, # type_mask (only positions enabled)
        north, east, down, # x, y, z positions (or North, East, Down in the MAV_FRAME_BODY_NED frame
        0, 0, 0, # x, y, z velocity in m/s  (not used)
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 
    # send command to vehicle
    vehicle.send_mavlink(msg)
    
def send_ned_velocity(velocity_x, velocity_y, velocity_z, cycles, period=1):
    """
    Move vehicle in direction based on specified velocity vectors and
    for the specified duration.
    
    cycles - number of cycles to execute
    period - time length of a single cycle in seconds

    This uses the SET_POSITION_TARGET_LOCAL_NED command with a type mask enabling only 
    velocity components 
    (http://dev.ardupilot.com/wiki/copter-commands-in-guided-mode/#set_position_target_local_ned).
    
    Note that from AC3.3 the message should be re-sent every second (after about 3 seconds
    with no message the velocity will drop back to zero). In AC3.2.1 and earlier the specified
    velocity persists until it is canceled. The code below should work on either version 
    (sending the message multiple times does not cause problems).
    
    See the above link for information on the type_mask (0=enable, 1=ignore). 
    At time of writing, acceleration and yaw bits are ignored.
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

    # send command to vehicle on 1/period Hz cycle
    for x in range(0,cycles):
        vehicle.send_mavlink(msg)
        time.sleep(period)
    
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
    while True:
        mode = int(input("Select mode, [0] print, [1] location, [2] velocity, [3] exit: "))
        if mode == 0:
            cycles = int(input("Input number of cycles: "))
            period = float(input("Input period duration: "))
            print("Printing NED position")
            print_ned(cycles, period)
            print("Finsihed")
        elif mode == 1:
            n_location = float(input("Input n position: "))
            e_location = float(input("Input e position: "))
            d_location = float(input("Input d position: "))
            print("Targeting NED position of %s, %s, %s" % (n_location, e_location, d_location))
            goto_position_target_local_ned(n_location, e_location, d_location)
            print("Finished")
        elif mode == 2:
            x_vel = float(input("Input x velocity: "))
            y_vel = float(input("Input y velocity: "))
            z_vel = float(input("Input z velocity: "))
            cycles = int(input("Input number of cycles: "))
            period = float(input("Input period duration: "))
            print("Targeting XYZ velocity of %s, %s, %s" % (x_vel, y_vel, z_vel))
            send_ned_velocity(x_vel, y_vel, z_vel, cycles, period)
            print("Finished")
        elif mode == 3:
            print("Exiting program")
            break
            
finally:
    disarm()
