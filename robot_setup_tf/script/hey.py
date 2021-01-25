#!/usr/bin/env python3

# Python ROS
import rospy

from robot_setup_tf.msg import Sonar
import time

s = []


def callback(data):
	print(data)
	s = data.distance
	print("sss = ",s)
	rospy.loginfo(rospy.get_caller_id() + " %s", data.distance)

     
def receive_message():
 
    #rospy.init_node('sensor_distance', anonymous=True)
 
    rospy.Subscriber("sonar_data", Sonar, callback)
 
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
 




rospy.init_node('sensor_distance', anonymous=True)
print("heey")
receive_message()


# if __name__ == '__main__':
# 	print("heey")
# 	receive_message()