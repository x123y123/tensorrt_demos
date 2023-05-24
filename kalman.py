import numpy as np
import time

import filterpy.kalman as kf
from dronekit import connect, VehicleMode,LocationGlobal,LocationGlobalRelative
from pymavlink import mavutil


class MyKalman:
    def __init__(self) -> None:
        #self.f = kf(dim_x = 1, dim_z = 1)
        self.x = np.zeros(1)
        self.F = np.array([1.])
        self.H = np.array([1.])
        self.P = 4.73
        self.R = 2.258578818
        self.Q = 0.;

    def bbox2dist(self, bbox) -> float:
        n = 3.180226752
        l0 = 1903.809012

        return (n * (l0 / (bbox ** 0.5) - 1))

    def dist2bbox(self, dist) -> float:
        n = 3.180226752
        l0 = 1903.809012

        return (l0 * (dist / n + 1) ** -1) ** 2

    def update(self, dist, vel, dt: float) -> float:
        B = np.array([dt])
        u = np.array([vel])
        z = np.array([dist])

        self.x, self.P = kf.predict(self.x, self.P, u = u, B = dt)
        self.x, self.P = kf.update(self.x, self.P, z, self.R, self.H)

        return self.x[0]

    def connect_px4(self) -> None:
        self.vehicle = connect('/dev/ttyACM0', wait_ready=True)

    def disconnect_px4(self) -> None:
        self.vehicle.close()

if __name__ == '__main__':
    kf = MyKalman()
    kf.connect_px4()
    time.sleep(2)
    #for i in range(20):
    while(True):
        print("gps: %s\n" % kf.vehicle.gps_0)
        print("G Location: %s\n" % kf.vehicle.location.global_relative_frame)
        print("L Location: %s\n" % kf.vehicle.location.local_frame)
        print("==========================================\n")
    kf.disconnect_px4()
