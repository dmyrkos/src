#!/usr/bin/env python3


import rospy
import math
import tf2_ros 
import geometry_msgs.msg
import tf_conversions
from std_msgs.msg import Int32MultiArray



def callback(data):
	hey = list(data.data)
	hey.append(9)
	for i in hey : print(i) 
	rospy.loginfo(rospy.get_caller_id() + " set %s", str(hey))
     
def receive_message():
 
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'simple_python_subscriber' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('tf2_joint', anonymous=True)
 
    rospy.Subscriber("my_bro", Int32MultiArray, callback)
 
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
 
if __name__ == '__main__':
    receive_message()

# if __name__ =='__main__':
# 	rospy.init_node('tf2_joints')
# 	br=tf2_ros.TransformBroadcaster()
# 	rate=rospy.Rate(5)


# 	t=geometry_msgs.msg.TransformStamped()

# 	while not rospy.is_shutdown():
# 		t.header.stamp = rospy.Time.now()
# 		t.header.frame_id = "A"
# 		t.child_frame_id = "B"
# 		t.transform.translation.x= 0.1
# 		t.transform.translation.y= 0.1
# 		t.transform.translation.z= 0.5
# 		q = tf_conversions.transformations.quaternion_from_euler(
# 		math.radians(0), math.radians(0) , math.radians(90) ) #roll pitch yaw
# 		t.transform.rotation.x = q[0]
# 		t.transform.rotation.y = q[1]
# 		t.transform.rotation.z = q[2]
# 		t.transform.rotation.w = q[3]
# 		br.sendTransform(t)
# # 		rate.sleep()

# 	rospy.spin()