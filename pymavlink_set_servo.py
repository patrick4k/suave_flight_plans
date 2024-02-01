from __future__ import print_function

from pymavlink import mavutil # Needed for command message definitions
import argparse

"""
HOW TO USE:
python <this file>.py --connect /dev/<connection name>
ex: python thrust_example.py --connect /dev/ttyACM0
"""
    
def connect_copter():
    connection_string = ""
    if args.connect is not None:
        connection_string = args.connect
    else: 
        connection_string = input("Enter device: ")
    print("Attempting to connect to vehicle: %s" % connection_string)
    vehicle = mavutil.mavlink_connection(connection_string, baud=57600, )
    return vehicle

def arm():
    vehicle.motors_armed_wait()

def disarm():
    vehicle.mototrs_disarmed_wait()

####################################################################################

def set_servo(n, val):
    vehicle.set_servo(n, val)
    
####################################################################################

# Parse arguments
parser = argparse.ArgumentParser(description='commands')
parser.add_argument('--connect')
args = parser.parse_args()

# Connect to copter
vehicle = connect_copter()

# Run flight plan
try:
    while True:
        mode = int(input("Select mode, [0] arm, [1] channel, [2] mode, [3] exit: "))
        if mode == 0:
            arm()
        elif mode == 1:
            n = input("Input channel number: ")
            val = int(input("Input new value: "))
            set_servo(n, val)
        elif mode == 2:
            done = False
            while not done:
                try:
                    print(vehicle.mode_mapping())
                    desired_mode = input("Mode: ")
                    if desired_mode == "break":
                        break
                    try:
                        vehicle.set_mode(desired_mode)
                        done = True
                    except:
                        done = False
                    if not done:
                        print("Timeout, could not switch to mode")
                except:
                    print("Could not switch to mode")
        elif mode == 3:
            print("Exiting program")
            break
    disarm()
    
except KeyboardInterrupt:
    disarm()    
