#!/usr/bin/env python3 
import rospy
# Because of transformations
import tf_conversions
import tf2_ros
import geometry_msgs.msg
import time

import math
#dict default positions base on initialize position (stand pos)

# con2  = { # format > parent -> child :x,y,z,r,p,y
#         "R1" : {"base_link" :("RBH",0.14,-0.085,0,0,0,0), "RBH": ("RBK",0.08,-0.08,-0.02,0,0,0),"RBK": ("RBA",0.085,-0.075,0,0,0,0) },
#         "R2" : {"base_link" :("RMH",0,-0.11,0,0,0,0), "RMH": ("RMK",0,-0.11,-0.02,0,0,0),"RMK": ("RMA",0,-0.13,0,0,0,0) },
#         "R3" : {"base_link" :("RFH",-0.16,-0.09,0,0,0,0), "RFH": ("RFK",-0.08,-0.07,-0.02,0,0,0),"RFK": ("RFA",-0.1,-0.07,0,0,0,0) },                                  
#         "L1" : {"base_link" :("LBH",0.14,0.085,0,0,0,0), "LBH": ("LBK",0.08,0.08,-0.02,0,0,0),"LBK": ("LBA",0.085,0.075,0,0,0,0) },
#         "L2" : {"base_link" :("LMH",0,0.11,0,0,0,0), "LMH": ("LMK",0,0.11,-0.02,0,0,0),"LMK": ("LMA",0,0.13,0,0,0,0) },
#         "L3" : {"base_link" :("LFH",0.16,0.09,0,0,0,45), "LFH": ("LFK",0.08,0.07,-0.02,0,0,0),"LFK": ("LFA",0.1,0.07,0,0,0,0) }
#         }

# con  = { # format > parent -> child :x,y,z,r,p,y
#         "R1" : {"base_link" :("RBS1",-0.1,0.04,0,0,0,0), "RBS1": ("RBH",0.03,0.03,0,0,0,0), "RBH": ("RBK",0.035,0.035,0,0,0,0),"RBK": ("RBA",0.035,0.035,0,0,0,0),"RBA": ("RBS2",0.14,0.14,0,0,0,0) }
#         #"R2" : {"base_link" :("RMS1",0,0.04,0,0,0,0),"RMS1": ("RMH",0,0.02,0,0,0,0), "RMH": ("RMK",0,-0.11,0,0,0,0),"RMK": ("RMA",0,-0.13,0,0,0,0) },
#         # "R3" : {"base_link" :("RFS1",0.1,0.04,0,0,0,0),"RFS1": ("RFH",0,0.02,0,0,0,0), "RFH": ("RFK",-0.08,-0.05,0,0,-45,0),"RFK": ("RFA",-0.1,-0.06,0,0,0,0) },                                  
#         # "L1" : {"base_link" :("LBS1",-0.1,-0.04,0,0,0,0),"LBS1": ("LBH",0,-0.02,0,0,0,0), "LBH": ("LBK",0.08,0.05,0,0,45,0),"LBK": ("LBA",0.085,0.055,0,0,0,0) },
#         # "L2" : {"base_link" :("LMS1",0,-0.04,0,0,0,0),"LMS1": ("LMH",0.,-0.02,0,0,0,0), "LMH": ("LMK",0,0.11,0,-45,0,0),"LMK": ("LMA",0,0.13,0,0,0,0) },
#         # "L3" : {"base_link" :("LFS1",0.1,-0.04,0,0,0,0),"LFS1": ("LFH",0,-0.02,0,0,0,0), "LFH": ("LFK",-0.08,0.05,0,0,-45,0),"LFK": ("LFA",-0.1,0.06,0,0,0,0) }
#          }

dynamic_links =  {
                "RF" :{ "A_RF":("RFH",0.01,-0.01,0,0,0,0), "RFH":("B_RF",0.05,-0.05,0,0,0,0),  "C_RF":("RFK",0,0,0,0,0,0), 
                        "RFK":("D_RF",0.027,-0.027,0,0,0,0) , "E_RF":("RFA",0,0,0,0,0,0) , "RFA":("F_RF",0.02,-0.02,0,0,0,0)},
                "LF" : { "A_LF":("LFH",0.01,0.01,0,0,0,0), "LFH":("B_LF",0.05,0.05,0,0,0,0),  "C_LF":("LFK",0,0,0,0,0,0), 
                        "LFK":("D_LF",0.027,0.027,0,0,0,0) , "E_LF":("LFA",0,0,0,0,0,0) , "LFA":("F_LF",0.02,0.02,0,0,0,0)},
                "RM" : { "A_RM":("RMH",0,-0.01,0,0,0,0), "RMH":("B_RM",0,-0.05,0,0,0,0),  "C_RM":("RMK",0,0,0,0,0,0), 
                        "RMK":("D_RM",0,-0.027,0,0,0,0) , "E_RM":("RMA",0,0,0,0,0,0) , "RMA":("F_RM",0,-0.02,0,0,0,0)},
                "LM": { "A_LM":("LMH",0,0.01,0,0,0,0), "LMH":("B_LM",0,0.05,0,0,0,0),  "C_LM":("LMK",0,0,0,0,0,0), 
                        "LMK":("D_LM",0,0.027,0,0,0,0) , "E_LM":("LMA",0,0,0,0,0,0) , "LMA":("F_LM",0,0.02,0,0,0,0)},
                "RB":  { "A_RB":("RBH",-0.01,-0.01,0,0,0,0), "RBH":("B_RB",-0.05,-0.05,0,0,0,0),  "C_RB":("RBK",0,0,0,0,0,0), 
                        "RBK":("D_RB",-0.027,-0.027,0,0,0,0) , "E_RB":("RBA",0,0,0,0,0,0) , "RBA":("F_RB",-0.02,-0.02,0,0,0,0)},
                "LB":  { "A_LB":("LBH",-0.01,0.01,0,0,0,0), "LBH":("B_LB",-0.05,0.05,0,0,0,0),  "C_LB":("LBK",0,0,0,0,0,0), 
                        "LBK":("D_LB",-0.027,0.027,0,0,0,0) , "E_LB":("LBA",0,0,0,0,0,0) , "LBA":("F_LB",-0.02,0.02,0,0,0,0)}
                 }

def callback():
    rospy.init_node('tf_broadcaster')
    br = tf2_ros.TransformBroadcaster()
    rate = rospy.Rate(5)
    t = geometry_msgs.msg.TransformStamped()
  
    while not rospy.is_shutdown():
        for i in dynamic_links:
            for j in dynamic_links[i]:
                t.header.stamp = rospy.Time.now()
                t.header.frame_id = j
                t.child_frame_id = dynamic_links[i][j][0]
                t.transform.translation.x= dynamic_links[i][j][1]
                t.transform.translation.y= dynamic_links[i][j][2]
                t.transform.translation.z= dynamic_links[i][j][3]
                q = tf_conversions.transformations.quaternion_from_euler(math.radians(dynamic_links[i][j][4]), math.radians(dynamic_links[i][j][5]) , math.radians(dynamic_links[i][j][6] ) )#roll pitch yaw
                t.transform.rotation.x = q[0]
                t.transform.rotation.y = q[1]
                t.transform.rotation.z = q[2]
                t.transform.rotation.w = q[3]
                br.sendTransform(t)
    
        rate.sleep()




if __name__ == '__main__':

    
    callback()    
    
    rospy.spin()

