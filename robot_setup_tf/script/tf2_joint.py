#!/usr/bin/env python3 
import rospy
# Because of transformations
import tf_conversions
import tf2_ros
import geometry_msgs.msg
import time
from robot_setup_tf.msg import servo
from std_msgs.msg import String
import math
#dict default positions base on initialize position (stand pos)


dynamic_links =  {
                "RF" :{ "A_RF":["RFH",0.01,0,0,0,0,0], "RFH":["B_RF",0,0,0,0,0,0],  "C_RF":["RFK",0,0,0,0,-45,0], 
                        "RFK":["D_RF",0.027,0,0,0,0,0] , "E_RF":["RFA",0,0,0,0,120,0] , "RFA":["F_RF",0.02,0,0,0,0,0]},
                "LF" : { "A_LF":["LFH",0.01,0,0,0,0,0], "LFH":["B_LF",0,0,0,0,0,0],  "C_LF":["LFK",0,0,0,0,-45,0], 
                        "LFK":["D_LF",0.027,0,0,0,0,0] , "E_LF":["LFA",0,0,0,0,120,0] , "LFA":["F_LF",0.02,0,0,0,0,0]},
                "RM" : { "A_RM":["RMH",0.01,0,0,0,0,0], "RMH":["B_RM",0,0,0,0,0,0],  "C_RM":["RMK",0,0,0,0,-45,0],   
                        "RMK":["D_RM",0.027,0,0,0,0,0] , "E_RM":["RMA",0,0,0,0,120,0] , "RMA":["F_RM",0.02,0,0,0,0,0]},
                "LM": { "A_LM":["LMH",0.01,0,0,0,0,0], "LMH":["B_LM",0,0,0,0,0,0],  "C_LM":["LMK",0,0,0,0,-45,0], 
                        "LMK":["D_LM",0.027,0,0,0,0,0] , "E_LM":["LMA",0,0,0,0,120,0] , "LMA":["F_LM",0.02,0,0,0,0,0]},
                "RB":  { "A_RB":["RBH",0.01,0,0,0,0,0], "RBH":["B_RB",0,0,0,0,0,0],  "C_RB":["RBK",0,0,0,0,-45,0], 
                        "RBK":["D_RB",0.027,0,0,0,0,0] , "E_RB":["RBA",0,0,0,0,120,0] , "RBA":["F_RB",0.02,0,0,0,0,0]},
                "LB":  { "A_LB":["LBH",0.01,0,0,0,0,0], "LBH":["B_LB",0,0,0,0,0,0],  "C_LB":["LBK",0,0,0,0,-45,0], 
                        "LBK":["D_LB",0.027,0,0,0,0,0] , "E_LB":["LBA",0,0,0,0,120,0] , "LBA":["F_LB",0.02,0,0,0,0,0]}
                }                

init_pos =  {
                "RF" :{ "A_RF":["RFH",0.01,0,0,0,0,0], "RFH":["B_RF",0,0,0,0,0,0],  "C_RF":["RFK",0,0,0,0,-45,0], 
                        "RFK":["D_RF",0.027,0,0,0,0,0] , "E_RF":["RFA",0,0,0,0,120,0] , "RFA":["F_RF",0.02,0,0,0,0,0]},
                "LF" : { "A_LF":["LFH",0.01,0,0,0,0,0], "LFH":["B_LF",0,0,0,0,0,0],  "C_LF":["LFK",0,0,0,0,-45,0], 
                        "LFK":["D_LF",0.027,0,0,0,0,0] , "E_LF":["LFA",0,0,0,0,120,0] , "LFA":["F_LF",0.02,0,0,0,0,0]},
             "RM" : { "A_RM":["RMH",0.01,0,0,0,0,0], "RMH":["B_RM",0,0,0,0,0,0],  "C_RM":["RMK",0,0,0,0,-45,0],   
                        "RMK":["D_RM",0.027,0,0,0,0,0] , "E_RM":["RMA",0,0,0,0,120,0] , "RMA":["F_RM",0.02,0,0,0,0,0]},
                "LM": { "A_LM":["LMH",0.01,0,0,0,0,0], "LMH":["B_LM",0,0,0,0,0,0],  "C_LM":["LMK",0,0,0,0,-45,0], 
                        "LMK":["D_LM",0.027,0,0,0,0,0] , "E_LM":["LMA",0,0,0,0,120,0] , "LMA":["F_LM",0.02,0,0,0,0,0]},
                "RB":  { "A_RB":["RBH",0.01,0,0,0,0,0], "RBH":["B_RB",0,0,0,0,0,0],  "C_RB":["RBK",0,0,0,0,-45,0], 
                        "RBK":["D_RB",0.027,0,0,0,0,0] , "E_RB":["RBA",0,0,0,0,120,0] , "RBA":["F_RB",0.02,0,0,0,0,0]},
                "LB":  { "A_LB":["LBH",0.01,0,0,0,0,0], "LBH":["B_LB",0,0,0,0,0,0],  "C_LB":["LBK",0,0,0,0,-45,0], 
                        "LBK":["D_LB",0.027,0,0,0,0,0] , "E_LB":["LBA",0,0,0,0,120,0] , "LBA":["F_LB",0.02,0,0,0,0,0]}
                        }

# def servo_rot_callback(p_inc,y_inc,triad):
#     for i in dynamic_links:
#         for j in dynamic_links[i]:
#             t.header.stamp = rospy.Time.now()
#             t.header.frame_id = j
#             t.child_frame_id = dynamic_links[i][j][0]
#             t.transform.translation.x= dynamic_links[i][j][1]
#             t.transform.translation.y= dynamic_links[i][j][2]
#             t.transform.translation.z= dynamic_links[i][j][3]
#             if dynamic_links[i][j][0] in triad and hip_key:
#                 s= dynamic_links[i][j][0]
#                 if s[0] == 'R' :
                    
#                     q = tf_conversions.transformations.quaternion_from_euler(
#                         math.radians(dynamic_links[i][j][4]), math.radians(dynamic_links[i][j][5]) , math.radians(dynamic_links[i][j][6]+y_inc ) )#roll pitch yaw
#                 else :
#                     q = tf_conversions.transformations.quaternion_from_euler(
#                         math.radians(dynamic_links[i][j][4]), math.radians(dynamic_links[i][j][5]) , math.radians(dynamic_links[i][j][6]-y_inc ) )#roll pitch yaw

#                 t.transform.rotation.x = q[0]
#                 t.transform.rotation.y = q[1]
#                 t.transform.rotation.z = q[2]
#                 t.transform.rotation.w = q[3]
#             elif dynamic_links[i][j][0] in knee_key :
#                 q = tf_conversions.transformations.quaternion_from_euler(
#                     math.radians(dynamic_links[i][j][4]), math.radians(dynamic_links[i][j][5] + p_inc) , math.radians(dynamic_links[i][j][6] ) )#roll pitch yaw
#                 t.transform.rotation.x = q[0]
#                 t.transform.rotation.y = q[1]
#                 t.transform.rotation.z = q[2]
#                 t.transform.rotation.w = q[3]    
#             else :
#                 q = tf_conversions.transformations.quaternion_from_euler(
#                     math.radians(dynamic_links[i][j][4]), math.radians(dynamic_links[i][j][5] ) , math.radians(dynamic_links[i][j][6] ) )#roll pitch yaw
#                 t.transform.rotation.x = q[0]
#                 t.transform.rotation.y = q[1]
#                 t.transform.rotation.z = q[2]
#                 t.transform.rotation.w = q[3]

#             br.sendTransform(t)



# def update_dic(move):
#     move_set = move.data
#     if move_set == 'fwd':
#         for i in range(8):
#             if i == 0:
#                 servo_rot_callback(-30,0,tr1)
#             elif i == 1:
#                 servo_rot_callback(-30,30,tr1)
#             elif i == 2:
#                 servo_rot_callback(0,30,tr1)
#             elif i == 3:
#                 servo_rot_callback(0,0,tr1)
#             elif i == 4:
#                 servo_rot_callback(-30,0,tr2)
#             elif i == 5:
#                 servo_rot_callback(-30,30,tr2)
#             elif i == 6:
#                 servo_rot_callback(0,30,tr2)
#             elif i == 7:
#                 servo_rot_callback(0,0,tr2)
#             time.sleep(0.5)
#     elif move_set=='Left': 
#         for i in range(8):
#             if i == 0:
#                 servo_rot_callback(-25,0,tr1)
#             elif i == 1:
#                 servo_rot_callback(-25,25,tr1)
#             elif i == 2:
#                 servo_rot_callback(0,25,tr1)
#             elif i == 3:
#                 servo_rot_callback(0,0,tr1)
#             elif i == 4:
#                 servo_rot_callback(-25,0,tr2)
#             elif i == 5:
#                 servo_rot_callback(-25,25,tr2)
#             elif i == 6:
#                 servo_rot_callback(0,25,tr2)
#             elif i == 7:
#                 servo_rot_callback(0,0,tr2)
#             time.sleep(0.5)

#     else :
#         for i in range(8):
#             if i == 0:
#                 servo_rot_callback(-25,0,tr2)
#             elif i == 1:
#                 servo_rot_callback(-25,25,tr2)
#             elif i == 2:
#                 servo_rot_callback(0,25,tr2)
#             elif i == 3:
#                 servo_rot_callback(0,0,tr2)
#             elif i == 4:
#                 servo_rot_callback(-25,0,tr1)
#             elif i == 5:
#                 servo_rot_callback(-25,25,tr1)
#             elif i == 6:
#                 servo_rot_callback(0,25,tr1)
#             elif i == 7:
#                 servo_rot_callback(0,0,tr1)
#             time.sleep(0.5)

def tdf():
	
	br = tf2_ros.TransformBroadcaster()
	t = geometry_msgs.msg.TransformStamped()
	for i in dynamic_links:
		for j in dynamic_links[i]:
			t.header.stamp = rospy.Time.now()
			t.header.frame_id = j
			t.child_frame_id = dynamic_links[i][j][0]
			t.transform.translation.x= dynamic_links[i][j][1]
			t.transform.translation.y= dynamic_links[i][j][2]
			t.transform.translation.z= dynamic_links[i][j][3]
			q = tf_conversions.transformations.quaternion_from_euler(
    			math.radians(dynamic_links[i][j][4]), math.radians(dynamic_links[i][j][5]) , math.radians(dynamic_links[i][j][6]) )
			t.transform.rotation.x = q[0]
			t.transform.rotation.y = q[1]
			t.transform.rotation.z = q[2]
			t.transform.rotation.w = q[3]
			br.sendTransform(t)


if __name__ == '__main__':

    rospy.init_node('tf_broadcaster')
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
    	tdf()
    	rate.sleep()



