#!/usr/bin/env python3 
import rospy
# Because of transformations
import tf_conversions
import tf2_ros
import geometry_msgs.msg
import time

import math
#dict default positions base on initialize position (stand pos)

con2  = { # format > parent -> child :x,y,z,r,p,y
        "R1" : {"base_link" :("RBH",0.14,-0.085,0,0,0,0), "RBH": ("RBK",0.08,-0.08,-0.02,0,0,0),"RBK": ("RBA",0.085,-0.075,0,0,0,0) },
        "R2" : {"base_link" :("RMH",0,-0.11,0,0,0,0), "RMH": ("RMK",0,-0.11,-0.02,0,0,0),"RMK": ("RMA",0,-0.13,0,0,0,0) },
        "R3" : {"base_link" :("RFH",-0.16,-0.09,0,0,0,0), "RFH": ("RFK",-0.08,-0.07,-0.02,0,0,0),"RFK": ("RFA",-0.1,-0.07,0,0,0,0) },                                  
        "L1" : {"base_link" :("LBH",0.14,0.085,0,0,0,0), "LBH": ("LBK",0.08,0.08,-0.02,0,0,0),"LBK": ("LBA",0.085,0.075,0,0,0,0) },
        "L2" : {"base_link" :("LMH",0,0.11,0,0,0,0), "LMH": ("LMK",0,0.11,-0.02,0,0,0),"LMK": ("LMA",0,0.13,0,0,0,0) },
        "L3" : {"base_link" :("LFH",0.16,0.09,0,0,0,45), "LFH": ("LFK",0.08,0.07,-0.02,0,0,0),"LFK": ("LFA",0.1,0.07,0,0,0,0) }
        }

con  = { # format > parent -> child :x,y,z,r,p,y
        "R1" : {"base_link" :("RBS1",-0.1,0.04,0,0,0,0), "RBS1": ("RBH",0.03,0.03,0,0,0,0), "RBH": ("RBK",0.035,0.035,0,0,0,0),"RBK": ("RBA",0.035,0.035,0,0,0,0),"RBA": ("RBS2",0.14,0.14,0,0,0,0) }
        #"R2" : {"base_link" :("RMS1",0,0.04,0,0,0,0),"RMS1": ("RMH",0,0.02,0,0,0,0), "RMH": ("RMK",0,-0.11,0,0,0,0),"RMK": ("RMA",0,-0.13,0,0,0,0) },
        # "R3" : {"base_link" :("RFS1",0.1,0.04,0,0,0,0),"RFS1": ("RFH",0,0.02,0,0,0,0), "RFH": ("RFK",-0.08,-0.05,0,0,-45,0),"RFK": ("RFA",-0.1,-0.06,0,0,0,0) },                                  
        # "L1" : {"base_link" :("LBS1",-0.1,-0.04,0,0,0,0),"LBS1": ("LBH",0,-0.02,0,0,0,0), "LBH": ("LBK",0.08,0.05,0,0,45,0),"LBK": ("LBA",0.085,0.055,0,0,0,0) },
        # "L2" : {"base_link" :("LMS1",0,-0.04,0,0,0,0),"LMS1": ("LMH",0.,-0.02,0,0,0,0), "LMH": ("LMK",0,0.11,0,-45,0,0),"LMK": ("LMA",0,0.13,0,0,0,0) },
        # "L3" : {"base_link" :("LFS1",0.1,-0.04,0,0,0,0),"LFS1": ("LFH",0,-0.02,0,0,0,0), "LFH": ("LFK",-0.08,0.05,0,0,-45,0),"LFK": ("LFA",-0.1,0.06,0,0,0,0) }
         }

def callback():
    rospy.init_node('tf_broadcaster')
    br = tf2_ros.TransformBroadcaster()
    rate = rospy.Rate(5)
    pa = 0
    t = geometry_msgs.msg.TransformStamped()
  
    while not rospy.is_shutdown():
    	for x in con :
    		for y in con[x]:
    			print(math.radians(con[x][y][5]),"Y=",y,pa+1,x)
    			t.header.stamp = rospy.Time.now()
    			t.header.frame_id = y
    			t.child_frame_id = con[x][y][0]
    			t.transform.translation.x= con[x][y][1]
    			t.transform.translation.y= con[x][y][2]
    			t.transform.translation.z= con[x][y][3]
    			q = tf_conversions.transformations.quaternion_from_euler(
				    math.radians(con[x][y][4]), math.radians(con[x][y][5]) , math.radians(con[x][y][6]) ) #roll pitch yaw
    			t.transform.rotation.x = q[0]
    			t.transform.rotation.y = q[1]
    			t.transform.rotation.z = q[2]
    			t.transform.rotation.w = q[3]
    			br.sendTransform(t)
    	rate.sleep()


def update(parent,child,x,y,z,r,p,yaw):
    #br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()   
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = parent
    t.child_frame_id = child 
    t.transform.translation.x += x
    t.transform.translation.y += y
    t.transform.translation.z += z
    q = tf_conversions.transformations.quaternion_from_euler(
            r,p, y) #roll pitch yaw
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]
    print(type(t))
    return t



if __name__ == '__main__':

    # p = con["R1"]["base_link"][0]
    # print(list(con["R1"]))
    # print("this si " ,p)
    # print(type(p))
    # for x in con:
    #     for y in con[x]:
    #         print(y,con[x][y][0])
    #     print(x)
    #for x in legs: print(legs[x][1])
    callback()    
    # rospy.init_node('tf_broadcaster')
    # br = tf2_ros.TransformBroadcaster()
    # t = geometry_msgs.msg.TransformStamped()

    # t.header.stamp = rospy.Time.now()
    # t.header.frame_id = "/world"
    # t.child_frame_id = "RBH"
    # t.transform.translation = (0.8,-0.6,-0.2)
    # q = tf_conversions.transformations.quaternion_from_euler(0, 0, 45)
    # t.transform.rotation.x = q[0]
    # t.transform.rotation.y = q[1]
    # t.transform.rotation.z = q[2]
    # t.transform.rotation.w = q[3]

    # br.sendTransform(t)
    
    rospy.spin()

