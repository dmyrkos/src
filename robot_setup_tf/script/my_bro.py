#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32MultiArray 

ss=[2]

servos = [ 12,3,3,4,5,6,7,8,9]
 
def publish_message():
    pub = rospy.Publisher('my_bro', Int32MultiArray, queue_size=10)
    rospy.init_node('my_bro', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
    	data = Int32MultiArray()
    	data.data= servos
    	rospy.loginfo("i sent:")
    	pub.publish(data)
    	rate.sleep()
 
if __name__ == '__main__':
    try:
        publish_message()
    except rospy.ROSInterruptException:
        pass
