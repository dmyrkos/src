#!/usr/bin/env python3

# Python ROS
import rospy

from robot_setup_tf.msg import Sonar

import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)


# foramt sonar  id  : TRiger , Echo
sonars_dic = {
		's0' : (17,27),'s1' : (5,6),
		 's2' : (23,24),'s3' : (21,20) 
		}
				


def get_distance(ECHO,TRIG):

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

	rate = rospy.Rate(10) # 5hz
	while not rospy.is_shutdown():

		rospy.loginfo("New Data")

		sonar_msg.header.stamp = rospy.Time.now()
		for x,i in enumerate(sonars_dic,0):
			sonar_msg.distance[x]=get_distance(sonars_dic[i][0],sonars_dic[i][1])
		sonar_pub.publish(sonar_msg)
		rate.sleep()

if __name__ == '__main__':
	try:
		for x,i in enumerate(sonars_dic,0):
			print(sonars_dic[i][0])
		run_sonars()
	except rospy.ROSInterruptException:
		pass



