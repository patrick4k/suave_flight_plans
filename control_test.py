import os
import argparse
import asyncio
import traceback
from datetime import datetime
from mavsdk import System
from mavsdk.offboard import (OffboardError, PositionNedYaw, VelocityNedYaw, Attitude, AttitudeRate, VelocityBodyYawspeed, AccelerationNed)
from typing import TypeVar, Generic

##################################################################################
# Util ###########################################################################

class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

bcolorslist: [chr] = [bcolors.HEADER, bcolors.OKBLUE, bcolors.OKCYAN, bcolors.OKGREEN, bcolors.WARNING, bcolors.FAIL, bcolors.ENDC, bcolors.BOLD, bcolors.UNDERLINE]
    
start_time = datetime.now()
start_time_str = start_time.strftime('%Y-%m-%d-%H-%M-%S')
console_history = []

try:
    os.mkdir("./log")
except:
    ""

def out(msg):
    print(msg)
    filtered = msg
    for bcolor in bcolorslist:
        filtered = filtered.replace(bcolor, "")
    console_history.append(filtered)

def write_out(filename, header:[str]=[]):
    f = open(filename, "w")
    for line in header:
        f.write(line)
        f.write("\n")
    for line in console_history:
        f.write(line)
        f.write("\n")
    f.close()
        
async def write_now():
    write_out(f"./log/flight_{start_time_str}.txt")
    
async def write_exit():
    write_out(f"./log/flight_latest.txt", ["Recorded flight on " + start_time_str, "This file will be overridden if executed again", ""])

def log(msg):
    msg = str(msg)
    out(bcolors.ENDC + bcolors.BOLD + "LOG: " + bcolors.ENDC + msg + bcolors.ENDC)

def succ(msg):
    msg = str(msg)
    out(bcolors.ENDC + bcolors.BOLD + bcolors.OKGREEN + "SUCCESS: " + bcolors.ENDC + bcolors.OKGREEN + msg + bcolors.ENDC)

def err(msg):
    msg = str(msg)
    out(bcolors.ENDC + bcolors.FAIL + bcolors.BOLD + "ERROR: " + bcolors.ENDC + bcolors.FAIL + msg + bcolors.ENDC)
    
def throw(msg):
    err(msg)
    raise RuntimeError(msg)

def warn(msg):
    msg = str(msg)
    out(bcolors.ENDC + bcolors.WARNING + bcolors.BOLD + "WARNING: " + bcolors.ENDC + bcolors.WARNING + msg + bcolors.ENDC)

def get_input_fn(parser):
    def get_user_intput(msg):
        msg = str(msg)
        val = input(bcolors.ENDC + bcolors.OKBLUE + bcolors.BOLD + msg + bcolors.ENDC)
        console_history.append(msg + val)
        return parser(val)
    return get_user_intput    

getstr = get_input_fn(str)
getfloat = get_input_fn(float)
getint = get_input_fn(int)

T = TypeVar('T')
class Result(Generic[T]):
    _isok = False
    _obj: T = None
    _errmsg = None
    def __init__(self, obj:T=None, err: str="No information provided"):
        if obj is None:
            self._isok = False
            self._errmsg = err
        else:
            self._obj = obj
            self._isok = True
    def __bool__(self):
        return self._isok
    def ok(self, obj:T=None):
        self._isok = True
        self._obj = obj
    def err(self, msg:str):
        self._isok = False
        self._errmsg = msg
    def disp(self):
        if not self:
            err(self._errmsg)
    def verify(self):
        if not self:
            err(self._errmsg)
            raise RuntimeError(f"result.unwrap() on not ok Result")
    def unwrap(self) -> T:
        self.verify()
        return self._obj
    
T = TypeVar('T')
class Parameter:
    def __init__(self, default:T, parser):
        self.value = default
        self.parser = parser
    def set(self, s:str):
        self.value = self.parser(s)
    value: T
    parser = None

class Parameters:
    parameter_map = dict()
    def getkeys(self):
        return self.parameter_map.keys()
    def add_parameter(self, name:str, default:T, parser):
        self.parameter_map[name] = Parameter(default, parser)
    def get(self, name:str):
        return self.parameter_map[name].value
    def set(self, name:str, value:str):
        parser = self.parameter_map[name].parser
        self.parameter_map[name] = Parameter(parser(value), parser)
    
##################################################################################    
# Flight plan Functions ##########################################################

async def print_state():
    connected = False
    offboard =  await drone.offboard.is_active()
    async for state in drone.core.connection_state():
        if state.is_connected:
            connected = True
            break
    log(f"Connected = {connected}")
    log(f"Offboard = {offboard}")
    async def check_arm_requirements():
        async for info in drone.telemetry.health():
            return info.is_gyrometer_calibration_ok and info.is_accelerometer_calibration_ok
    if not await check_arm_requirements():
        warn("Drone does not meet requirements for arming without GPS.")
    else:
        log("Drone meets requirements for arming without GPS.")
    
async def connect_drone():
    connection_string = ""
    if args.connect is not None:
        connection_string = args.connect
    else: 
        connection_string = input("Enter device: ")
    address = f"serial://{connection_string}:57600"
    log(f"Connecting to address = {address}")
    try:
        await drone.connect(system_address=address)
        print("Waiting for drone to connect...")
        connected = False
        async for state in drone.core.connection_state():
            if state.is_connected:
                connected = True
                break
        if connected:
            succ(f"Successfully connected to {address}!")
        else:
            err(f"Not successfully connected to {address}!")
    except:
        err(f"Failed to connect to {address}")
    
async def connect():
    result = await connect_drone()
    if result:
        drone_result.ok(result.unwrap())
    else:
        drone_result.err(result._errmsg)
        raise RuntimeError("Could not connect to drone")
    
async def arm():
    log("Armming drone...")
    await drone.action.arm()
    
    async def wait_until_armed():
        async for is_armed in drone.telemetry.armed():
            if is_armed:
                return

    await wait_until_armed()
    succ("Drone arming complete!")
    
async def disarm():
    log("Disarmming drone")
    await drone.action.disarm()
    succ("Drone disarming complete!")
    
async def start_offboard():
    log("Starting offboard mode...")
    try:
        await drone.offboard.start()
        is_active = await drone.offboard.is_active()
        log(f"Offboard.is_active = {is_active}")
        if is_active:
            succ("Start offboard complete!")
        else:
            err("Failed to start offboard mode")
    except OffboardError as e:
        err(f"Starting offboard mode failed with error code: {e._result.result}")
        raise e

async def stop_offboard():
    log("Stopping offboard...")
    try:
        await drone.offboard.stop()
        is_active = await drone.offboard.is_active()
        log(f"Offboard.is_active = {is_active}")
        if not is_active:
            succ("Stop offboard complete!")
        else:
            err("Failed to stop offboard mode")
    except OffboardError as e:
        err(f"Starting offboard mode failed with error code: {e._result.result}")
        raise e

# Attitude
async def attitude_setpoint():
    log("Setting Attitude setpoint...")
    await drone.offboard.set_attitude(Attitude(0.0, 0.0, 0.0, 0.0))
    succ("Setpoint is set!")
async def thrust():
    thrust = getfloat("Enter thrust [0-1]: ")
    if thrust < 0 or thrust > 1:
        throw("Thrust not in range [0-1]")
    log(f"Setting thrust to {thrust}")
    await drone.offboard.set_attitude(Attitude(0.0, 0.0, 0.0, thrust))
    succ("Finished setting thrust!")
async def set_attitude():
    axis_list = {"roll": 0, "pitch": 0, "yaw": 0, "thrust": 0}
    for key in axis_list.keys():
        axis_list[key] = getfloat(f"Enter {key} = ")
    attitude = Attitude(axis_list["roll"], axis_list["pitch"], axis_list["yaw"], axis_list["thrust"])
    log(f"Setting attitude to: {attitude}")
    await drone.offboard.set_attitude(attitude)
    succ("Finished setting attitude and thrust!")
    
# Attitude Rate
async def attitude_rate_setpoint():
    log("Setting Attitude Rate setpoint...")
    await drone.offboard.set_attitude_rate(AttitudeRate(0.0, 0.0, 0.0, 0.0))
    succ("Setpoint is set!")
async def set_attitude_rate():
    axis_list = {"roll_rate": 0, "pitch_rate": 0, "yaw_rate": 0, "thrust": 0}
    for key in axis_list.keys():
        axis_list[key] = getfloat(f"Enter {key} = ")
    attitude_rate = AttitudeRate(axis_list["roll_rate"], axis_list["pitch_rate"], axis_list["yaw_rate"], axis_list["thrust"])
    log(f"Setting attitude_rate to: {attitude_rate}")
    await drone.offboard.set_attitude_rate(attitude_rate)
    succ("Finished setting attitude and thrust!")
    
# Position NED
async def position_ned_setpoint():
    log("Setting Position NED setpoint...")
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))
    succ("Setpoint is set!")
async def set_position_ned():
    axis_list = {"north": 0, "east": 0, "down": 0, "yaw": 0}
    for key in axis_list.keys():
        axis_list[key] = getfloat(f"Enter {key} = ")
    position = PositionNedYaw(axis_list["north"], axis_list["east"], axis_list["down"], axis_list["yaw"])
    log(f"Setting position to {position}")
    await drone.offboard.set_position_ned(position)
    succ("Finished setting position!")
    
# Velocity NED
async def velocity_ned_setpoint():
    log("Setting Velocity NED setpoint...")
    await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))
    succ("Setpoint is set!")
async def set_velocity_ned():
    axis_list = {"north": 0, "east": 0, "down": 0, "yaw": 0}
    for key in axis_list.keys():
        axis_list[key] = getfloat(f"Enter {key} = ")
    velocity = VelocityNedYaw(axis_list["north"], axis_list["east"], axis_list["down"], axis_list["yaw"])
    log(f"Setting velocity to {velocity}")
    await drone.offboard.set_velocity_ned(velocity)
    succ("Finished setting velocity!")
async def set_velocity_up():
    velocity = VelocityNedYaw(0.0, 0.0, -0.5, 0.0)
    log(f"Setting velocity to {velocity}")
    await drone.offboard.set_velocity_ned(velocity)
    succ("Finished setting velocity!")
async def set_velocity_north():
    velocity = VelocityNedYaw(0.75, 0.0, 0.0, 0.0)
    log(f"Setting velocity to {velocity}")
    await drone.offboard.set_velocity_ned(velocity)
    succ("Finished setting velocity!")
    
# Velocity Body
async def velocity_body_setpoint():
    log("Setting Velocity Body setpoint...")
    await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
    succ("Setpoint is set!")
async def set_velocity_body():
    axis_list = {"x": 0, "y": 0, "z": 0, "yaw_rate": 0}
    for key in axis_list.keys():
        axis_list[key] = getfloat(f"Enter {key} = ")
    velocity = VelocityBodyYawspeed(axis_list["x"], axis_list["y"], axis_list["z"], axis_list["yaw_rate"])
    log(f"Setting velocity to {velocity}")
    await drone.offboard.set_velocity_body(velocity)
    succ("Finished setting velocity!")
    
# Acceleration NED
async def acceleration_ned_setpoint():
    log("Setting Acceleration NED setpoint...")
    await drone.offboard.set_acceleration_ned(AccelerationNed(0.0, 0.0, 0.0))
    succ("Setpoint is set!")
async def set_acceleration_ned():
    axis_list = {"north": 0, "east": 0, "down": 0}
    for key in axis_list.keys():
        axis_list[key] = getfloat(f"Enter {key} = ")
    acceleration = AccelerationNed(axis_list["north"], axis_list["east"], axis_list["down"])
    log(f"Setting acceleration to {acceleration}")
    await drone.offboard.set_acceleration_ned(acceleration)
    succ("Finished setting acceleration!")
    
# Position Velocity Acceleration NED
async def position_velocity_ned_setpoint():
    log("Setting Position Velocity NED setpoint...")
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))
    await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))
    succ("Setpoint is set!")
async def set_position_velocity_ned():
    axis_list = {"north": 0, "east": 0, "down": 0, "yaw": 0}
    for key in axis_list.keys():
        axis_list[key] = getfloat(f"Enter position {key} = ")
    position = PositionNedYaw(axis_list["north"], axis_list["east"], axis_list["down"], axis_list["yaw"])
    for key in axis_list.keys():
        axis_list[key] = getfloat(f"Enter velocity {key} = ")
    velocity = VelocityNedYaw(axis_list["north"], axis_list["east"], axis_list["down"], axis_list["yaw"])
    log(f"Setting position = {position}")
    log(f"Setting velocity = {velocity}")
    await drone.offboard.set_position_velocity_ned(position, velocity)

def sleep(n):
    async def sleep_n():
        log(f"Sleeping for {n} seconds...")
        await asyncio.sleep(n)
        log("Awake!")
    return sleep_n
    
##################################################################################
# Main ###########################################################################

# Args
argparser = argparse.ArgumentParser(description='commands')
argparser.add_argument('--connect')
args = argparser.parse_args()

# Parameters
parameters = Parameters()
parameters.add_parameter("endonfail", 1, int)
parameters.add_parameter("should_exit", 0, int)
async def print_params():
    ks = parameters.getkeys()
    for k in ks:
        log(f"{k} = {parameters.get(k)}")
async def edit_params():
    param = getstr("%s\nSelect parameter: " % parameters.getkeys())
    if param == "none":
        return
    log(f"{param} = {parameters.get(param)}")
    parameters.set(param, getstr("New value: "))
    log(f"{param} = {parameters.get(param)}")
async def exit_program():
    parameters.set("should_exit", "1")

# Flight plan Definition 
SLEEP_TIME = 5
flight_plan = {
    "connect": [connect_drone, print_state],
    "arm": [arm],
    "disarm": [disarm],
    "offboard": [start_offboard, sleep(SLEEP_TIME), stop_offboard],
    "thrust": [arm, attitude_setpoint, start_offboard, thrust, sleep(SLEEP_TIME), attitude_setpoint, sleep(SLEEP_TIME), disarm],
    "upnorth": [arm, velocity_ned_setpoint, start_offboard, set_velocity_up, sleep(2), set_velocity_north, sleep(10), velocity_ned_setpoint, sleep(SLEEP_TIME), disarm],
    "attitude": [arm, attitude_setpoint, start_offboard, sleep(2), set_attitude, sleep(SLEEP_TIME), disarm],
    "velocity_ned": [arm, velocity_ned_setpoint, start_offboard, set_velocity_ned, sleep(SLEEP_TIME), disarm],
    "velocity_body": [arm, velocity_body_setpoint, start_offboard, set_velocity_body, sleep(SLEEP_TIME), disarm],
    "position": [arm, position_ned_setpoint, start_offboard, set_position_ned, sleep(SLEEP_TIME), disarm],
    "acceleration": [arm, acceleration_ned_setpoint, start_offboard, set_acceleration_ned, sleep(SLEEP_TIME), disarm],
    "posvel": [arm, position_velocity_ned_setpoint, start_offboard, set_position_velocity_ned, sleep(SLEEP_TIME), disarm],
    "write": [write_now],
    "debug": [print_state],
    "edit": [print_params, edit_params],
    "exit": [write_exit, exit_program]
}

# Execute program
drone_result: Result[System] = Result(err="Not yet connected")
drone = System()

async def main():
    while True:
        plan_name = getstr("%s\nSelect: " % flight_plan.keys())
        plan_list = flight_plan.get(plan_name)
        if plan_list is not None:
            log(f"Starting '{plan_name}'")
            success = True
            for plan in plan_list:
                try:
                    await plan()
                except Exception as e:
                    success = False
                    err(traceback.format_exception(e))
                except KeyboardInterrupt:
                    success = False
                    err(f"Keyboard interruption, ending '{plan_name}'")
                    break
                if not success and parameters.get("endonfail"):
                    break
                if parameters.get("should_exit"):
                    log("should_exit = True")
                    return
            if success:
                succ(f"Finished '{plan_name}'")
            else:
                err(f"Failed flight '{plan_name}'")
        else:
            warn(f"Could not find  named '{plan_name}'")

asyncio.run(main())
log("Exiting program...")

##################################################################################
