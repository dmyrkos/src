#!/usr/bin/env python3

# Python ROS
import rospy

from robot_setup_tf.msg import Sonar
import time

def callback(data):

	rospy.loginfo(rospy.get_caller_id() + " %f", data.data)
     
def receive_message():
 
    rospy.init_node('sensor_distance', anonymous=True)
 
    rospy.Subscriber("sonar_data", Sonar, callback)
 
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
 
if __name__ == '__main__':
    receive_message()