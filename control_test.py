import time
import argparse
import asyncio
from mavsdk import System
from mavsdk.offboard import (OffboardError, PositionNedYaw, VelocityNedYaw)

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

def log(msg):
    msg = str(msg)
    print(bcolors.BOLD + "LOG: " + bcolors.ENDC + msg + bcolors.ENDC)

def succ(msg):
    msg = str(msg)
    print(bcolors.BOLD + bcolors.OKGREEN + "SUCCESS: " + bcolors.ENDC + bcolors.OKGREEN + msg + bcolors.ENDC)

def err(msg):
    msg = str(msg)
    print(bcolors.FAIL + bcolors.BOLD + "ERROR: " + bcolors.ENDC + bcolors.FAIL + msg + bcolors.ENDC)

def warn(msg):
    msg = str(msg)
    print(bcolors.WARNING + bcolors.BOLD + "WARNING: " + bcolors.ENDC + bcolors.WARNING + msg + bcolors.ENDC)

class Result:
    _isok = False
    _obj = None
    _errmsg = None
    def __init__(self, obj=None, err="No information provided"):
        if obj is None:
            self._isok = False
            self._errmsg = err
        else:
            self._obj = obj
            self._isok = True
    def __bool__(self):
        return self._isok
    def ok(self, obj=None):
        self._isok = True
        self._obj = obj
    def err(self, msg):
        self._isok = False
        self._errmsg = msg
    def verify(self):
        if not self:
            err(self._errmsg)
            raise RuntimeError(f"result.unwrap() on not ok Result")
    def unwrap(self):
        self.verify()
        return self._obj

async def connect_drone() -> Result:
    connection_string = ""
    if args.connect is not None:
        connection_string = args.connect
    else: 
        connection_string = input("Enter device: ")
    address = f"serial://{connection_string}:57600"
    log(f"Connecting to address = {address}")
    drone = System()
    try:
        await drone.connect(system_address=address)
        succ(f"Successfully connected to {address}")
        return Result(drone)
    except:
        return Result(err=f"Failed to connect to {address}")

parser = argparse.ArgumentParser(description='commands')
parser.add_argument('--connect')
args = parser.parse_args()

drone = asyncio.run(connect_drone()).unwrap()
