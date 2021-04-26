#!/usr/bin/env python3
import rospy
import math
import tf2_ros 
import geometry_msgs.msg 
from nav_msgs.msg import Odometry
import tf_conversions 
# from sensor_msgs import Imu



def callback(data):
    t.header.stamp = rospy.Time.now()
    t.header.frame_id ="map"
    t.child_frame_id ="odom"
    t.transform.translation.x= data.pose.pose.position.x 
    t.transform.translation.y= data.pose.pose.position.y 
    t.transform.translation.z= data.pose.pose.position.z 
    # q = tf_conversions.transformations.quaternion_from_euler(
    # math.radians(), math.radians() , math.radians() )#roll pitch yaw
    t.transform.rotation.x = data.pose.pose.orientation.x
    t.transform.rotation.y = data.pose.pose.orientation.y
    t.transform.rotation.z = data.pose.pose.orientation.z
    t.transform.rotation.w = data.pose.pose.orientation.w
    br.sendTransform(t)

     
def receive_message():
 
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'simple_python_subscriber' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('pose_listener', anonymous=True)
 
    rospy.Subscriber("pose/filtered", Odometry, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()






if __name__ == '__main__':

    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()
    receive_message()
