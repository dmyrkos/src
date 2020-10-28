#!/usr/bin/env python3

# Python ROS
import rospy

from robot_setup_tf.msg import Sonar

import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)

# PINOUT
# Sonar 0 
TRIG = 23
ECHO = 24

# foramt sonar  id  : TRiger , Echo
sonars_dic = {
		's0' : (17,27),'s1' : (5,6),
		's2' : (23,24),'s3' : (20,21) 
		}
				


def get_distance(TRIG,ECHO):

	GPIO.setup(TRIG,GPIO.OUT)
	GPIO.setup(ECHO,GPIO.IN)
	GPIO.output(TRIG, False)

	#print ("Waiting For Sensor To Settle")
	time.sleep(0.5)

	GPIO.output(TRIG, True)
	time.sleep(0.00001)
	GPIO.output(TRIG, False)

	while GPIO.input(ECHO)==0:
		pulse_start = time.time()

	while GPIO.input(ECHO)==1:
		pulse_end = time.time()

	pulse_duration = pulse_end - pulse_start

	distance = pulse_duration * 17150

	distance = round(distance, 2)

	#print ("Distance:",distance,"cm")
	return distance

def run_sonars():

	rospy.init_node("sonars", anonymous=True)
	sonar_pub = rospy.Publisher("sonar_data", Sonar, queue_size=10)
	
	sonar_msg = Sonar()
	sonar_msg.header.frame_id = "sonar_mount"

	rate = rospy.Rate(5) # 1hz
	while not rospy.is_shutdown():

		rospy.loginfo("New Data")

		sonar_msg.header.stamp = rospy.Time.now()
		sonar_msg.distance = get_distance()

		sonar_pub.publish(sonar_msg)
		rate.sleep()

if __name__ == '__main__':
	try:
		for x in sonars_dic :

		run_sonars()
	except rospy.ROSInterruptException:
		pass



