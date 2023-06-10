import numpy as np
import time
import math

import filterpy.kalman as kf
from dronekit import connect, VehicleMode,LocationGlobal,LocationGlobalRelative
from pymavlink import mavutil

class MyKalman:
    def __init__(self, mode = 0) -> None:
        if (mode == 0):
            self.Is3dModel = False
            self.x = np.array(1.)
            self.F = np.array(1.)
            self.H = np.array(1.)
            self.P = 3.73
            self.R = 1.258578818
            self.Q = 0.
        elif (mode == 1):
            self.set_3d_kalman()
            self.pre_yaw = 0

    def set_3d_kalman(self) -> None:
        self.Is3dModel = True
        self.x = np.array([36652279.61564, 1, 1])
        self.F = np.eye(3)
        self.H = np.array([[1, 1, 1]])
        self.B = np.zeros((3, 2))
        self.P = np.array([[10000, 0, 0], [0, 10, 0], [0, 0, 10]])
        self.Q = np.array([[10, 0, 0], [0, 10, 0], [0, 0, 10]])
        self.R = 2.25878818

    def bbox2dist(self, bbox) -> float:
        n = 3.180226752
        l0 = 1903.809012
        a = 36652279.61564

        return (n * (l0 / (bbox ** 0.5) - 1))

    def dist2bbox(self, dist) -> float:
        n = 3.180226752
        l0 = 1903.809012
        a = 36652279.61564

        return (l0 * (dist / n + 1) ** -1) ** 2

    def update(self, state, vel, dt: float, yaw = 0) -> float:

        if (self.Is3dModel == False):
            B = np.array([dt])
            u = np.array([vel])
            z = np.array([state])

            self.x, self.P = kf.predict(self.x, self.P, u = u, B = dt)
            self.x, self.P = kf.update(self.x, self.P, z, self.R, self.H)

        else :
            dyaw = yaw - self.pre_yaw
            pre_yaw = yaw

            self.F[1][2] = -1 * dyaw
            self.F[2][1] = dyaw
            self.B[1][0] = dt
            self.B[2][1] = dt
            z = np.array([state])

            self.x, self.P = kf.predict(self.x, self.P, F = self.F, u = vel, B = self.B)

            self.H[0][0] = pow(self.x[1] ** 2 + self.x[2] ** 2, -1)
            self.H[0][1] = -2 * self.x[0] * pow(self.x[1] ** 2 + self.x[2] ** 2, -1) * self.x[1]
            self.H[0][2] = -2 * self.x[0] * pow(self.x[1] ** 2 + self.x[2] ** 2, -1) * self.x[2]

            self.x, self.P = kf.update(self.x, self.P, z, self.R, H = self.H)

        return self.x

    def connect_px4(self) -> None:
        self.vehicle = connect('/dev/ttyACM0', wait_ready=True)

    def disconnect_px4(self) -> None:
        self.vehicle.close()

    def haversine_distance(self, lat1, lon1, lat2, lon2):
        R = 6371  # Radius of the Earth in kilometers

        # Convert latitude and longitude to radians
        lat1_rad = math.radians(lat1)
        lon1_rad = math.radians(lon1)
        lat2_rad = math.radians(lat2)
        lon2_rad = math.radians(lon2)

        # Calculate the differences between the coordinates
        dlat = lat2_rad - lat1_rad
        dlon = lon2_rad - lon1_rad

        # Apply the Haversine formula
        a = math.sin(dlat/2)**2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon/2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        distance = R * c

        return distance

    def reset_home(self) -> None:
        #download commands to reset home location
        cmds = kf.vehicle.commands
        cmds.download()
        cmds.wait_ready()

        #recode current GPS data
        self.vehicle.home_location = self.vehicle.location.global_frame


if __name__ == '__main__':
    kf = MyKalman()
    #connect to pixhawk
    # kf.connect_px4()

    ''' save data from pix4
    f = open("/home/uav/code/tensorrt_demos/gps_data_0608.txt", "w")
    for i in range(1000):
        f.write(str(kf.vehicle.location.global_frame.lat) + ' ' + str(kf.vehicle.location.global_frame.lon) + '\n')

    f.close()
    '''

    # kf.disconnect_px4()

    '''  test gps data
    pre_data = [0.0, 0.0]
    
    for line in open("/home/uav/code/tensorrt_demos/gps_data_0602_1.txt"):
        curr_data = line.split(' ')
        curr_data[0] = float(curr_data[0])
        curr_data[1] = float(curr_data[1])
        print("dist: " + str(kf.haversine_distance(pre_data[0], pre_data[1], curr_data[0], curr_data[1])))
        pre_data = curr_data
    '''
