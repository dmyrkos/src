#!/usr/bin/env python3
import rospy
import math
import tf2_ros 
import geometry_msgs.msg
import tf_conversions
from std_msgs.msg import Float32MultiArray
# from sensor_msgs import Imu



def callback(data):
	rospy.loginfo(rospy.get_caller_id() + " set %s", data.data)
     
def receive_message():
 
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'simple_python_subscriber' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('imu_listener', anonymous=True)
 
    rospy.Subscriber("imu_data", Float32MultiArray, callback)
 
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
 
if __name__ == '__main__':
    receive_message()
