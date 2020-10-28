#!/usr/bin/env python3

from nxp_imu import IMU
import time

# verbose will print some start-up info on the IMU sensors
imu = IMU(gs=4, dps=2000, verbose=True)

# setting a bias correction for the accel only, the mags/gyros
# are not getting a bias correction
imu.setBias((0.1,-0.02,.25), None, None)

# grab data at 10Hz
rate = Rate(10)

while True:
    header = 67
    print('-'*header)
    print("| {:17} | {:20} | {:20} |".format("Accels [g's]", " Magnet [uT]", "Gyros [dps]"))
    print('-'*header)
    for _ in range(10):
        a, m, g = imu.get()
        print('| {:>5.2f} {:>5.2f} {:>5.2f} | {:>6.1f} {:>6.1f} {:>6.1f} | {:>6.1f} {:>6.1f} {:>6.1f} |'.format(
            a[0], a[1], a[2],
            m[0], m[1], m[2],
            g[0], g[1], g[2])
        )
        time.sleep(0.50)
    print('-'*header)
    print(' uT: micro Tesla')
    print('  g: gravity')
    print('dps: degrees per second')
    print('')

    rate.sleep()