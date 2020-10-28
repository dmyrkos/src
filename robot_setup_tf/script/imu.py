#!/usr/bin/env python3

import rospy
import math
#from std_msgs.msg import Float32MultiArray
import time
import tf2_ros
# import board
# import busio
# import adafruit_fxas21002c
# import adafruit_fxos8700
from nxp_imu import IMU
from  sensor_msgs.msg import Imu
from  sensor_msgs.msg import MagneticField




magnetic_offset = [7.1,-27.15,-49.0]
gyro_offset = [-0.46,1.97,1.67]

def send_imu_data():
	#pub = rospy.Publisher('imu_data',Float32MultiArray,queue_size=10)
	pub2 = rospy.Publisher('imu/data_raw',Imu,queue_size=10)
	pub3 = rospy.Publisher('imu/mag',MagneticField,queue_size=10)
	rospy.init_node('imu_data')
	mag_msg = MagneticField()
	imu_msg = Imu()
	#imu_data = Float32MultiArray()

	rate=rospy.Rate(10)
	while not rospy.is_shutdown():
		imu_msg.header.stamp = rospy.Time.now()
		imu_msg.header.frame_id='imu_tf' 
		mag_msg.header.stamp = rospy.Time.now()
		a,m,g = imu.get()
		imu_msg.angular_velocity.x,imu_msg.angular_velocity.y,imu_msg.angular_velocity.z = g #sensorG.gyroscope
		imu_msg.linear_acceleration.x,imu_msg.linear_acceleration.y,imu_msg.linear_acceleration.z = a #sensorAM.accelerometer
		#mag_x, mag_y, mag_z= sensorAM.magnetometer
		#imu_data.data = [gyro_x , gyro_y,gyro_z,accel_x, accel_y, accel_z,mag_x, mag_y, mag_z ]
		mag_msg.magnetic_field.x,mag_msg.magnetic_field.y,mag_msg.magnetic_field.z = m 
		pub2.publish(imu_msg)
		pub3.publish(mag_msg)

		rate.sleep()

	rospy.spin()



if __name__ == '__main__':
	
	# Initialize I2C bus and device.
	# i2c = busio.I2C(board.SCL, board.SDA)
	# sensorG = adafruit_fxas21002c.FXAS21002C(i2c)
	# i2c = busio.I2C(board.SCL, board.SDA)
	# sensorAM = adafruit_fxos8700.FXOS8700(i2c)
	imu = IMU(gs=4, dps=2000, verbose=True)
	# setting a bias correction for the accel only, the mags/gyros
	# are not getting a bias correction
	imu.setBias((0.1,-0.02,.25), (7.1,-27.15,-49.0), (-0.46,1.97,1.67))



	send_imu_data()





	# Simple demo of the FXAS21002C gyroscope.
# Will print the gyroscope values every second.


# i2c = busio.I2C(board.SCL, board.SDA)
# sensor = adafruit_fxos8700.FXOS8700(i2c)
# Optionally create the sensor with a different accelerometer range (the
# default is 2G, but you can use 4G or 8G values):
# sensor = adafruit_fxos8700.FXOS8700(i2c, accel_range=adafruit_fxos8700.ACCEL_RANGE_4G)
# sensor = adafruit_fxos8700.FXOS8700(i2c, accel_range=adafruit_fxos8700.ACCEL_RANGE_8G)

# Main loop will read the acceleration and magnetometer values every second
# and print them out.


# Initialize I2C bus and device.

# Optionally create the sensor with a different gyroscope range (the
# default is 250 DPS, but you can use 500, 1000, or 2000 DPS values):
# sensor = adafruit_fxas21002c.FXAS21002C(i2c, gyro_range=adafruit_fxas21002c.GYRO_RANGE_500DPS)
# sensor = adafruit_fxas21002c.FXAS21002C(i2c, gyro_range=adafruit_fxas21002c.GYRO_RANGE_1000DPS)
# sensor = adafruit_fxas21002c.FXAS21002C(i2c, gyro_range=adafruit_fxas21002c.GYRO_RANGE_2000DPS)

# Main loop will read the gyroscope values every second and print them out.
# while True:
#     # Read gyroscope.
#     gyro_x, gyro_y, gyro_z = sensor.gyroscope
#     # Print values.
#     print(
#         "Gyroscope (radians/s): ({0:0.3f},  {1:0.3f},  {2:0.3f})".format(
#             gyro_x, gyro_y, gyro_z
#         )
#     )
#     # Delay for a second.
#     time.sleep(1.0)

# while True:
#     # Read acceleration & magnetometer.
#     accel_x, accel_y, accel_z = sensor.accelerometer
#     mag_x, mag_y, mag_z = sensor.magnetometer
#     # Print values.
#     print(
#         "Acceleration (m/s^2): ({0:0.3f}, {1:0.3f}, {2:0.3f})".format(
#             accel_x, accel_y, accel_z
#         )
#     )
#     print(
#         "Magnetometer (uTesla): ({0:0.3f}, {1:0.3f}, {2:0.3f})".format(
#             mag_x, mag_y, mag_z
#         )
#     )
#     # Delay for a second.
#     time.sleep(1.0)
