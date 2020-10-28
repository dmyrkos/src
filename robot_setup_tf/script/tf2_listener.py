#!/usr/bin/env python3  
import rospy

import math
import tf2_ros
import geometry_msgs.msg
import tf_conversions


if __name__ == '__main__':
    rospy.init_node('tf2_listener')

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    # = geometry_msgs.msg.TransformStamped()
 

    br = tf2_ros.TransformBroadcaster()

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform('RFK','RFA', rospy.Time())

            trans.header.stamp = rospy.Time.now()
            #trans.header.frame_id = "base_link"
            #trans.child_frame_id = "RBH"   
            
            trans.transform.translation.x= 4
            
            q = tf_conversions.transformations.quaternion_from_euler(            
            math.radians(0), math.radians(0), math.radians(45) ) #roll pitch yaw
            
            trans.transform.rotation.x = q[0]
            trans.transform.rotation.y = q[1]
            trans.transform.rotation.z = q[2]
            trans.transform.rotation.w = q[3]

            br.sendTransform(trans)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue

        # msg = geometry_msgs.msg.Twist()

        # msg.angular.z = 4 * math.atan2(trans.transform.translation.y, trans.transform.translation.x)
        # msg.linear.x = 0.5 * math.sqrt(trans.transform.translation.x ** 2 + trans.transform.translation.y ** 2)

        rate.sleep()