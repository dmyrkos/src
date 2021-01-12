#!/usr/bin/env python3  
import rospy
import logging
import sys
import time
import json

import tf2_ros
import tf_conversions

import numpy as np

import std_msgs 

from std_msgs.msg import Float64

import math

import geometry_msgs.msg

from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, TransformStamped, AccelStamped

from sensor_msgs.msg import Imu

from Adafruit_BNO055 import BNO055

# Constatnts
DEFAULT_CAL_FILE = 'calibration.json'

DEFAULT_RATE = 30
MIN_RATE = 1
MAX_RATE = 30

# Global variables
global bno, loop_rate, rate_changed

def handleLoadCalibrationJson(req):
	
	filename = req.file

	print('Loading calibration file: {0}'.format(filename))

	try:
		with open(filename, 'r') as cal_file:
			data = json.load(cal_file)
			bno.set_calibration(data)

			return 1

	except IOError:

		print ('Could not load calibration file...')
		print ('Initializing uncalibrated')

	return 0

def handleSetStreamRate(req):

	global rate_changed, loop_rate

	new_rate = req.hz

	print('Received new rate:   {0}'.format(new_rate))

	# Check if new rate is acceptable
	if (new_rate >= MIN_RATE) and (new_rate <= MAX_RATE):

		rate_changed = True
		loop_rate = req.hz

		return 1

	else:

		return 0

def euler_to_quaternion(roll, pitch, yaw):

        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

        return Quaternion(qx, qy, qz, qw)

def quaternion_multiply(quat1, quat0):
    
    quat_ret = Quaternion()

    x0 = quat0.x
    y0 = quat0.y
    z0 = quat0.z
    w0 = quat0.w

    x1 = quat1.x
    y1 = quat1.y
    z1 = quat1.z
    w1 = quat1.w

    quat_ret.x =  x1 * w0 + y1 * z0 - z1 * y0 + w1 * x0
    quat_ret.y = -x1 * z0 + y1 * w0 + z1 * x0 + w1 * y0
    quat_ret.z =  x1 * y0 - y1 * x0 + z1 * w0 + w1 * z0
    quat_ret.w = -x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0

    return quat_ret

if __name__ == '__main__':

	rospy.init_node('bno055_node')

	imu_pub   = rospy.Publisher("bno055/imu", Imu, queue_size=10)
	#heading_pub = rospy.Publisher("sense_gimbal/bno055/compass_hdg", Float64, queue_size=10)
	#accel_pub 	= rospy.Publisher("sense_gimbal/bno055/accel_stamped", AccelStamped, queue_size=10)

	#load_calib = rospy.Service('sense_gimbal/bno055/load_calibration_json', LoadCalibrationJson, handleLoadCalibrationJson)
	#set_rate = rospy.Service('sense_gimbal/bno055/set_stream_rate', SetStreamRate, handleSetStreamRate)
	
	# Raspberry Pi configuration with serial UART and RST connected to GPIO 18:
	bno = BNO055.BNO055(serial_port='/dev/ttyS0', rst=18)

	# Try to read calibration file initialy
	calib_file_name = rospy.get_param('bno055_calibration_file', DEFAULT_CAL_FILE)

	try:
		with open(calib_file_name, 'r') as cal_file:
			data = json.load(cal_file)
			bno.set_calibration(data)

	except IOError:

		print('Could not load calibration file...')
		print('Initializing uncalibrated')
	
	# Initialize the BNO055 and stop if something went wrong.
	if not bno.begin():
	    raise RuntimeError('Failed to initialize BNO055! Is the sensor connected?')

	# Print system status and self test result.
	status, self_test, error = bno.get_system_status()

	print('System status: {0}'.format(status))
	print('Self test result (0x0F is normal): 0x{0:02X}'.format(self_test))
	# Print out an error if system status is in error mode.
	if status == 0x01:
	    print('System error: {0}'.format(error))
	    print('See datasheet section 4.3.59 for the meaning.')

	# Print BNO055 software revision and other diagnostic data.
	sw, bl, accel, mag, gyro = bno.get_revision()
	print('Software version:   {0}'.format(sw))
	print('Bootloader version: {0}'.format(bl))
	print('Accelerometer ID:   0x{0:02X}'.format(accel))
	print('Magnetometer ID:    0x{0:02X}'.format(mag))
	print('Gyroscope ID:       0x{0:02X}\n'.format(gyro))
	print('Reading BNO055 data, press Ctrl-C to quit...')

	# Try to read Default Loop Rate
	rate_changed = False

	loop_rate = rospy.get_param('bno055_stream_rate', DEFAULT_RATE) # Default rate in Hz

	rate = rospy.Rate(loop_rate) 

	# Initialize Message 
	imu_msg = Imu()

	imu_msg.header = std_msgs.msg.Header()
	imu_msg.header.stamp = rospy.Time.now()
	imu_msg.header.frame_id = 'bno055'
	
	imu_msg.orientation = Quaternion()
	imu_msg.angular_velocity = Vector3()
	imu_msg.linear_acceleration = Vector3()

	accel_msg = AccelStamped()
	accel_msg.header = std_msgs.msg.Header()
	accel_msg.header.stamp = rospy.Time.now()
	accel_msg.header.frame_id = 'bno055'

	while not rospy.is_shutdown():

		# Check if the rate is changed
		if rate_changed:
			
			rate = rospy.Rate(loop_rate) 
			rate_changed = False

			print('New Rate:   {0}'.format(loop_rate))


		# Update bno055 message
		time_now = rospy.Time.now()

		#bno_msg.header.stamp   = time_now
		imu_msg.header.stamp   = time_now
		accel_msg.header.stamp = time_now
		
		# Get quaternion Data
		x,y,z,w = bno.read_quaternion()

		quat = Quaternion()
		quat.x = x
		quat.y = y
		quat.z = z
		quat.w = w

		# Pass Orientation Data
		imu_msg.orientation = quat

		x,y,z = bno.read_gyroscope()

		ang_vel = Vector3()
		ang_vel.x = x
		ang_vel.y = y
		ang_vel.z = z

		# Pass Angular Velocity Data
		imu_msg.angular_velocity = ang_vel

		x,y,z = bno.read_accelerometer()

		lin_acc = Vector3()
		lin_acc.x = x
		lin_acc.y = y
		lin_acc.z = z

		# Pass Angular Velocity Data
		imu_msg.linear_acceleration = lin_acc

		x,y,z = bno.read_linear_acceleration()

		accel_msg.accel.linear.x = x
		accel_msg.accel.linear.y = y
		accel_msg.accel.linear.z = z

		# Read the calibration status, 0 = uncalibrated and 3 = fully calibrated.
		sys, gyro, accel, mag = bno.get_calibration_status()

		# Pass calibration Data
		#bno_msg.calibration_status.sys 	 = sys
		#bno_msg.calibration_status.gyro  = gyro
		#bno_msg.calibration_status.accel = accel
		#bno_msg.calibration_status.mag 	 = mag

		heading, roll, pitch = bno.read_euler()
		
		hdg_msg = Float64(heading)
		
		# Print everything out.
		#print('Heading={0:0.2F} '.format(heading))

		# Publish
		#heading_pub.publish(hdg_msg)
		#quat_pub.publish(bno_msg)
		imu_pub.publish(imu_msg)
		#accel_pub.publish(accel_msg)

		rate.sleep()

# Read the Euler angles for heading, roll, pitch (all in degrees).

#heading, roll, pitch = bno.read_euler()

# Print everything out.
#print('Heading={0:0.2F} Roll={1:0.2F} Pitch={2:0.2F}\tSys_cal={3} Gyro_cal={4} Accel_cal={5} Mag_cal={6}'.format(
#      heading, roll, pitch, sys, gyro, accel, mag))

# Other values you can optionally read:
# Orientation as a quaternion:
#x,y,z,w = bno.read_quaterion()
# Sensor temperature in degrees Celsius:
#temp_c = bno.read_temp()
# Magnetometer data (in micro-Teslas):
#x,y,z = bno.read_magnetometer()
# Gyroscope data (in degrees per second):
#x,y,z = bno.read_gyroscope()
# Accelerometer data (in meters per second squared):
#x,y,z = bno.read_accelerometer()
# Linear acceleration data (i.e. acceleration from movement, not gravity--
# returned in meters per second squared):
#x,y,z = bno.read_linear_acceleration()
# Gravity acceleration data (i.e. acceleration just from gravity--returned
# in meters per second squared):
#x,y,z = bno.read_gravity()
# Sleep for a second until the next reading.

#Header header

#geometry_msgs/Quaternion orientation
#float64[9] orientation_covariance # Row major about x, y, z axes

#geometry_msgs/Vector3 angular_velocity
#float64[9] angular_velocity_covariance # Row major about x, y, z axes

#geometry_msgs/Vector3 linear_acceleration
#float64[9] linear_acceleration_covariance # Row major x, y z 
