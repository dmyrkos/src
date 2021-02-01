#!/usr/bin/env python3

# Python ROS
import rospy

from robot_setup_tf.msg import Sonar
import time

global s = []


def callback(data):
	print(data)
	s = data.distance
	print("sss = ",s)
	rospy.loginfo(rospy.get_caller_id() + " %s", data.distance)
	return s

     
def receive_message():
 
    #rospy.init_node('sensor_distance', anonymous=True)
 
    rospy.Subscriber("sonar_data", Sonar, callback)
 
    # spin() simply keeps python from exiting until this node is stopped
 
 




rospy.init_node('sensor_distance', anonymous=True)

rate =rospy.Rate(1)
while not rospy.is_shutdown():
	receive_message()
	print("hey",s)
	rate.sleep()




# if __name__ == '__main__':
# 	print("heey")
# 	receive_message()