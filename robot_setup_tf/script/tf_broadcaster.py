#!/usr/bin/env python3 
import rospy
# Because of transformations
import tf_conversions
import tf2_ros
import geometry_msgs.msg
import time
from robot_setup_tf.msg import Servo_tf

import math
#dict default positions base on initialize position (stand pos)
dynamic_links =  {
                "RF" :{ "A_RF":["RFH",0.01,0,0,-90,0,0], "RFH":["B_RF",0,0,0,90,0,0],  "C_RF":["RFK",0,0,0,0,45,0], 
                        "RFK":["D_RF",0.027,0,0,0,0,0] , "E_RF":["RFA",0,0,0,0,45,0] , "RFA":["F_RF",0.02,0,0,0,0,0]},
                "LF" : { "A_LF":["LFH",0.01,0,0,90,0,0], "LFH":["B_LF",0,0,0,-90,0,0],  "C_LF":["LFK",0,0,0,0,45,0], 
                        "LFK":["D_LF",0.027,0,0,0,0,0] , "E_LF":["LFA",0,0,0,0,45,0] , "LFA":["F_LF",0.02,0,0,0,0,0]},
                "RM" : { "A_RM":["RMH",0.01,0,0,-90,0,0], "RMH":["B_RM",0,0,0,90,0,0],  "C_RM":["RMK",0,0,0,0,45,0],   
                        "RMK":["D_RM",0.027,0,0,0,0,0] , "E_RM":["RMA",0,0,0,0,45,0] , "RMA":["F_RM",0.02,0,0,0,0,0]},
                "LM": { "A_LM":["LMH",0.01,0,0,90,0,0], "LMH":["B_LM",0,0,0,-90,0,0],  "C_LM":["LMK",0,0,0,0,45,0], 
                        "LMK":["D_LM",0.027,0,0,0,0,0] , "E_LM":["LMA",0,0,0,0,45,0] , "LMA":["F_LM",0.02,0,0,0,0,0]},
                "RB":  { "A_RB":["RBH",0.01,0,0,90,0,0], "RBH":["B_RB",0,0,0,-90,0,0],  "C_RB":["RBK",0,0,0,0,45,0], 
                        "RBK":["D_RB",0.027,0,0,0,0,0] , "E_RB":["RBA",0,0,0,0,45,0] , "RBA":["F_RB",0.02,0,0,0,0,0]},
                "LB":  { "A_LB":["LBH",0.01,0,0,90,0,0], "LBH":["B_LB",0,0,0,-90,0,0],  "C_LB":["LBK",0,0,0,0,45,0], 
                        "LBK":["D_LB",0.027,0,0,0,0,0] , "E_LB":["LBA",0,0,0,0,45,0] , "LBA":["F_LB",0.02,0,0,0,0,0]}
                 }


dynamic_links2 =  {
                "RF" :{ "A_RF":["RFH",0.01,0,0,-90,0,-45], "RFH":["B_RF",0,0,0,90,0,0],  "C_RF":["RFK",0,0,0,0,0,0], 
                        "RFK":["D_RF",0.027,0,0,0,0,0] , "E_RF":["RFA",0,0,0,0,0,0] , "RFA":["F_RF",0.02,0,0,0,0,0]},
                "LF" : { "A_LF":["LFH",0.01,0,0,-90,0,45], "LFH":["B_LF",0,0,0,90,0,0],  "C_LF":["LFK",0,0,0,0,0,0], 
                        "LFK":["D_LF",0.027,0,0,0,0,0] , "E_LF":["LFA",0,0,0,0,0,0] , "LFA":["F_LF",0.02,0,0,0,0,0]},
                "RM" : { "A_RM":["RMH",0.01,0,0,-90,0,0], "RMH":["B_RM",0,0,0,90,0,0],  "C_RM":["RMK",0,0,0,0,0,0],   
                        "RMK":["D_RM",0.027,0,0,0,0,0] , "E_RM":["RMA",0,0,0,0,0,0] , "RMA":["F_RM",0.02,0,0,0,0,0]},
                "LM": { "A_LM":["LMH",0.01,0,0,-90,0,0], "LMH":["B_LM",0,0,0,90,0,0],  "C_LM":["LMK",0,0,0,0,0,0], 
                        "LMK":["D_LM",0.027,0,0,0,0,0] , "E_LM":["LMA",0,0,0,0,0,0] , "LMA":["F_LM",0.02,0,0,0,0,0]},
                "RB":  { "A_RB":["RBH",0.01,0,0,-90,0,45], "RBH":["B_RB",0,0,0,90,0,0],  "C_RB":["RBK",0,0,0,0,0,0], 
                        "RBK":["D_RB",0.027,0,0,0,0,0] , "E_RB":["RBA",0,0,0,0,0,0] , "RBA":["F_RB",0.02,0,0,0,0,0]},
                "LB":  { "A_LB":["LBH",0.01,0,0,-90,0,-45], "LBH":["B_LB",0,0,0,90,0,0],  "C_LB":["LBK",0,0,0,0,0,0], 
                        "LBK":["D_LB",0.027,0,0,0,0,0] , "E_LB":["LBA",0,0,0,0,0,0] , "LBA":["F_LB",0.02,0,0,0,0,0]}
                 }

#init_position = ankle=0 knee = 45 ,ankle =45


def servo_rot_callback():
    for i in dynamic_links:
        for j in dynamic_links2[i]:
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = j
            t.child_frame_id = dynamic_links2[i][j][0]
            t.transform.translation.x= dynamic_links2[i][j][1]
            t.transform.translation.y= dynamic_links2[i][j][2]
            t.transform.translation.z= dynamic_links2[i][j][3]
            q = tf_conversions.transformations.quaternion_from_euler(
                math.radians(dynamic_links2[i][j][4]), math.radians(dynamic_links2[i][j][5]) , math.radians(dynamic_links2[i][j][6] ) )#roll pitch yaw
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]
            br.sendTransform(t)



def update_dic(servo_data):

    s1 = servo_data.header.frame_id[:-1] 
    if servo_data.header.frame_id[-1] =='H' :
        s2= 'A_' + servo_data.header.frame_id[:-1]
    elif servo_data.header.frame_id[-1] =='K':
        s2= 'C_' + servo_data.header.frame_id[:-1]  
    else :
        s2= 'E_' + servo_data.header.frame_id[:-1]

    x_axis = dynamic_links[s1][s2][4]
    y_axis = dynamic_links[s1][s2][5]
    z_axis = dynamic_links[s1][s2][6]

    rospy.loginfo( " x : %d ,y : %d , z :%d",x_axis,y_axis,z_axis)

    if s1 == 'H' :
        dynamic_links[s1][s2].insert(6,z_axis + servo_data.servo_deg)
    elif s1 == 'K' :
        dynamic_links[s1][s2].insert(5, y_axis + servo_data.servo_deg)
    else :
        dynamic_links[s1][s2].insert(5,y_axis + servo_data.servo_deg)





if __name__ == '__main__':

    rospy.init_node('tf_broadcaster')
    br = tf2_ros.TransformBroadcaster()
    try:   

        rate = rospy.Rate(1)
        t = geometry_msgs.msg.TransformStamped()

        while not rospy.is_shutdown():
            rospy.Subscriber("servo_data",Servo_tf , update_dic)
            servo_rot_callback()
            rate.sleep()
         
         
    except rospy.ROSInterruptException:
        pass
