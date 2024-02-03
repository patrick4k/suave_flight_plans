import time
import argparse
import asyncio
from mavsdk import System
from mavsdk.offboard import (OffboardError, PositionNedYaw, VelocityNedYaw)

async def arm_drone():

    # Get connection string
    connection_string = ""
    if args.connect is not None:
        connection_string = args.connect
    else: 
        connection_string = input("Enter device: ")

    # Connect to the drone
    drone = System()
    await drone.connect(system_address=f"serial://{connection_string}:57600")

    time.sleep(1)

    # Check if the autopilot allows arming without GPS
    # async def check_arm_requirements():
    #     async for info in drone.telemetry.health():
    #         return info.is_gyrometer_calibration_ok and info.is_accelerometer_calibration_ok

    # if not await check_arm_requirements():
    #     print("Drone does not meet requirements for arming without GPS.")
    #     return

    # Arm the drone
    print("Arming the drone...")
    await drone.action.arm()

    # Wait for the drone to be armed
    async def wait_until_armed():
        async for is_armed in drone.telemetry.armed():
            if is_armed:
                return

    await wait_until_armed()
    print("Drone is armed.")

    time.sleep(5)

    print("Setting initial setpoint")
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))

    print("Starting offboard")
    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(f"Starting offboard mode failed with error code: {error._result.result}")
        print("Disarming")
        await drone.action.disarm()
        return

    await drone.offboard.set_velocity_ned(VelocityNedYaw(0, 0, -10, 0))

    is_active = await drone.offboard.is_active()
    print(f"Offboard.is_active = {is_active}")

    await drone.action.disarm()

parser = argparse.ArgumentParser(description='commands')
parser.add_argument('--connect')
args = parser.parse_args()

# Run the script
asyncio.run(arm_drone())