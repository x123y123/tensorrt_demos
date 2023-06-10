import time
import os
import platform
import sys

from dronekit import connect, VehicleMode,LocationGlobal,LocationGlobalRelative
from pymavlink import mavutil


######### performance monitor  ##########
from performance_model import performance_monitor
######### DRONEKIT ############
#x for(+)_back(-)
#y let(-)_right(+)
#z up(-)_down(+)
######### FUNCTIONS ###########

class drone_controller:
#This class can be used easily for drone controlling.
    @performance_monitor
    def __init__(self, connection_string: str) -> None:
        """
        Function: __init__()
        --------------------
        Initialize and connect to pixhawk
        
        connection_string: port id on host device

        return: None
        """
        self.vehicle = connect(connection_string, wait_ready=True)

    @performance_monitor
    def takeoff(self, altitude: float) -> None:
        """
        Function: takeoff()
        --------------------
        Let drone takeoff and change to mode to GUIDED mode
        
        altitude: the target height(meters) you want to takeoff

        return: None
        """
        self.vehicle.mode = VehicleMode("GUIDED")
        #self.vehicle.mode = VehicleMode("LOITER")
        
        self.vehicle.armed = True

        # Confirm vehicle armed before attempting to take off
        while not self.vehicle.armed:
            print(" Waiting for arming...")
            time.sleep(1)

        self.vehicle.simple_takeoff(altitude)

        while True:
            if self.vehicle.location.global_relative_frame.alt >= altitude * 0.95:
                print("Reached target altitude")
                break
            time.sleep(1)

    @performance_monitor
    def land(self) -> None:
        """
        Function: land()
        --------------------
        Let drone land by changing the mode to LAND mode,
        and check the altitude at the same time.

        return: None
        """
        self.vehicle.mode = VehicleMode("LAND")
        
        while True:
            if self.vehicle.location.global_relative_frame.alt <= 0.1:
                break
            time.sleep(1)
        
        self.vehicle.close()
    
    @performance_monitor
    def move_right(self) -> None:
        """
        Function: move_right()
        --------------------
        Let drone move right by sending mavlink message.
        
        return: None
        """
        velocity_x = 0
        velocity_y = 0.1
        velocity_z = 0
        
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
                0,       # time_boot_ms (not used)
                0, 0,    # target system, target component
                mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, # frame Needs to be MAV_FRAME_BODY_NED for forward/back left/right control.
                0b0000111111000111, # type_mask
                0, 0, 0, # x, y, z positions (not used)
                velocity_x, velocity_y, velocity_z, # m/s
                0, 0, 0, # x, y, z acceleration
                0, 0)
        start_time = time.time()
        self.vehicle.send_mavlink(msg)
        self.vehicle.flush()
        end_time = time.time()
        latency = end_time - start_time
        print(f"move_right latency: {latency}")
        
    @performance_monitor
    def move_left(self) -> None:
        """
        Function: move_left()
        --------------------
        Let drone move left by sending mavlink message.
        
        return: None
        """
        
        velocity_x = 0
        velocity_y = -0.1
        velocity_z = 0
        
        
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
                0,       # time_boot_ms (not used)
                0, 0,    # target system, target component
                mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, # frame Needs to be MAV_FRAME_BODY_NED for forward/back left/right control.
                0b0000111111000111, # type_mask
                0, 0, 0, # x, y, z positions (not used)
                velocity_x, velocity_y, velocity_z, # m/s
                0, 0, 0, # x, y, z acceleration
                0, 0)
        start_time = time.time()
        self.vehicle.send_mavlink(msg)
        self.vehicle.flush()
        end_time = time.time()
        latency = end_time - start_time
        print(f"move_left latency: {latency}")


    @performance_monitor
    def move_backward(self) -> None:
        """
        Function: move_backward()
        --------------------
        Let drone move backward by sending mavlink message.
        
        return: None
        """
        velocity_x = -0.1
        velocity_y = 0
        velocity_z = 0
        
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
                0,       # time_boot_ms (not used)
                0, 0,    # target system, target component
                mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, # frame Needs to be MAV_FRAME_BODY_NED for forward/back left/right control.
                0b0000111111000111, # type_mask
                0, 0, 0, # x, y, z positions (not used)
                velocity_x, velocity_y, velocity_z, # m/s
                0, 0, 0, # x, y, z acceleration
                0, 0)
        start_time = time.time()
        self.vehicle.send_mavlink(msg)
        self.vehicle.flush()
        end_time = time.time()
        latency = end_time - start_time
        print(f"move_backward latency: {latency}")

    @performance_monitor
    def move_forward(self) -> None:
        """
        Function: move_forward()
        --------------------
        Let drone move forward by sending mavlink message.
        
        return: None
        """
        velocity_x = 0.1
        velocity_y = 0
        velocity_z = 0
        
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
                0,       # time_boot_ms (not used)
                0, 0,    # target system, target component
                mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, # frame Needs to be MAV_FRAME_BODY_NED for forward/back left/right control.
                0b0000111111000111, # type_mask
                0, 0, 0, # x, y, z positions (not used)
                velocity_x, velocity_y, velocity_z, # m/s
                0, 0, 0, # x, y, z acceleration
                0, 0)
        start_time = time.time()
        self.vehicle.send_mavlink(msg)
        self.vehicle.flush()
        end_time = time.time()
        latency = end_time - start_time
        print(f"move_forward latency: {latency}")
    
    @performance_monitor
    def move_up(self) -> None:
        """
        Function: move_up()
        --------------------
        Let drone move up by sending mavlink message.
        
        return: None
        """
        velocity_x = 0
        velocity_y = 0
        velocity_z = -0.1
        
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
                0,       # time_boot_ms (not used)
                0, 0,    # target system, target component
                mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, # frame Needs to be MAV_FRAME_BODY_NED for forward/back left/right control.
                0b0000111111000111, # type_mask
                0, 0, 0, # x, y, z positions (not used)
                velocity_x, velocity_y, velocity_z, # m/s
                0, 0, 0, # x, y, z acceleration
                0, 0)
        start_time = time.time()
        self.vehicle.send_mavlink(msg)
        self.vehicle.flush()
        end_time = time.time()
        latency = end_time - start_time
        print(f"move_up latency: {latency}")


    @performance_monitor
    def move_down(self) -> None:
        """
        Function: move_down()
        --------------------
        Let drone move down by sending mavlink message.
        
        return: None
        """
        velocity_x = 0
        velocity_y = 0
        velocity_z = 0.1
        
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
                0,       # time_boot_ms (not used)
                0, 0,    # target system, target component
                mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, # frame Needs to be MAV_FRAME_BODY_NED for forward/back left/right control.
                0b0000111111000111, # type_mask
                0, 0, 0, # x, y, z positions (not used)
                velocity_x, velocity_y, velocity_z, # m/s
                0, 0, 0, # x, y, z acceleration
                0, 0)
        start_time = time.time()
        self.vehicle.send_mavlink(msg)
        self.vehicle.flush()
        end_time = time.time()
        latency = end_time - start_time
        print(f"move_down latency: {latency}")
    
#    @performance_monitor
    def move(self, velocity_x, velocity_y, velocity_z) -> None:
        """
        Function: move()
        --------------------
        Let drone moving by sending mavlink message.
        
        return: None
        """
        
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
                0,       # time_boot_ms (not used)
                0, 0,    # target system, target component
                mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, # frame Needs to be MAV_FRAME_BODY_NED for forward/back left/right control.
                0b0000111111000111, # type_mask
                0, 0, 0, # x, y, z positions (not used)
                velocity_x, velocity_y, velocity_z, # m/s
                0, 0, 0, # x, y, z acceleration
                0, 0)
        start_time = time.time()
        self.vehicle.send_mavlink(msg)
        self.vehicle.flush()
        end_time = time.time()
        latency = end_time - start_time
        print(f"move_right latency: {latency}")
"""
def setPositionTarget(vehicle: Vehicle, position: Tuple[float, float], yawRate=0) -> None:
    global IS_LOITER

    localNorth, localEast = position

    if localNorth == 0 and localEast == 0:
        # Loiter in place with guided mode
        setLoiterGuided(vehicle, yawRate)
    else:
        IS_LOITER = False

        # Find altitude target for NED frame
        currentAltitude = vehicle.location.global_relative_frame.alt
        targetAltOffset = 0.0 - (ALTITUDE - currentAltitude) # up is negative

        ignoreVelocityMask =  0b111000
        ignoreAccelMask =  0b111000000
        ignoreYaw = 0b10000000000
        emptyMask = 0b0000000000000000

        msg = vehicle.message_factory.set_position_target_local_ned_encode(
            0,       # time_boot_ms (not used)
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, # Use offset from current position
            emptyMask + ignoreAccelMask + ignoreVelocityMask + ignoreYaw, # type_mask
            localNorth, localEast, targetAltOffset,
            0, 0, 0, # x, y, z velocity in m/s (not used)
            0, 0, 0, # x, y, z acceleration (not used)
            0, math.radians(yawRate))    # yaw, yaw_rate

        vehicle.send_mavlink(msg)

def setLoiterGuided(vehicle: Vehicle, yawRate: float) -> None:
    global IS_LOITER
    global LOITER_POSITION

    if IS_LOITER != True:
        IS_LOITER = True

        # update position
        frame = vehicle.location.global_relative_frame
        LOITER_POSITION = {
            "latitude": frame.lat,
            "longitude": frame.lon
        }

    ignoreVelocityMask =  0b111000
    ignoreAccelMask =  0b111000000
    ignoreYaw = 0b10000000000
    emptyMask = 0b0000000000000000

    msg = vehicle.message_factory.set_position_target_global_int_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, # frame
        emptyMask + ignoreAccelMask + ignoreVelocityMask + ignoreYaw, # type_mask (only speeds enabled)
        int(LOITER_POSITION["latitude"] * 1e7), # lat_int - X Position in WGS84 frame in 1e7 * meters
        int(LOITER_POSITION["longitude"] * 1e7), # lon_int - Y Position in WGS84 frame in 1e7 * meters
        ALTITUDE,
        0, # X velocity in NED frame in m/s
        0, # Y velocity in NED frame in m/s
        0, # Z velocity in NED frame in m/s
        0, 0, 0, # afx, afy, afz acceleration
        0, math.radians(yawRate))    # yaw, yaw_rate (rad/s)

    vehicle.send_mavlink(msg)
"""
############MAIN###############
if __name__ == '__main__':
    connection_string = '/dev/ttyACM0'
    start_time = time.time()
    control = drone_controller(connection_string)
    end_time = time.time()
    latency = end_time - start_time
    print(f"connect latency: {latency}")
    
    control.takeoff(2)
    print("takeoff act!!")
    time.sleep(1)
    

    control.move(0.1, 0.1, -0.1)
    print("move test")
    print("moving~")
    time.sleep(1)
    """
    control.move_forward()
    print("forward act!!")
    time.sleep(1)
    
    control.move_left()
    print("left act!!")
    time.sleep(1)
    
    control.move_right()
    print("right act!!")
    time.sleep(1)
    
    control.move_backward()
    print("back act!!")
    time.sleep(1)
    
    control.move_up()
    print("up act!!")
    time.sleep(1)
    control.move_down()
    print("down act!!")
    time.sleep(1)
    """
    
    control.land()
    print("land act!!")
