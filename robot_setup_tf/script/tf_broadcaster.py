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
hip_key = ['RFH','RBH','RMH','LFH','LMH','LBH']
knee_key = ['RFK','RBK','RMK','LFK','LMK','LBK']
tr1= ['RFH','RBH','LMH']
tr2=['RMH','LFH','LBH']
#init_position = ankle=0 knee = 45 ,ankle =45
tmp = []
counter = 0
pre =-1
tm=-1 
pp=1

def servo_rot_callback(p_inc,y_inc,triad):
    for i in dynamic_links:
        for j in dynamic_links[i]:
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = j
            t.child_frame_id = dynamic_links[i][j][0]
            t.transform.translation.x= dynamic_links[i][j][1]
            t.transform.translation.y= dynamic_links[i][j][2]
            t.transform.translation.z= dynamic_links[i][j][3]
            if dynamic_links[i][j][0] in triad and hip_key:
                s= dynamic_links[i][j][0]
                print(s)
                if s[0] == 'R' :
                    
                    q = tf_conversions.transformations.quaternion_from_euler(
                        math.radians(dynamic_links[i][j][4]), math.radians(dynamic_links[i][j][5]) , math.radians(dynamic_links[i][j][6]+y_inc ) )#roll pitch yaw
                else :
                    q = tf_conversions.transformations.quaternion_from_euler(
                        math.radians(dynamic_links[i][j][4]), math.radians(dynamic_links[i][j][5]) , math.radians(dynamic_links[i][j][6]-y_inc ) )#roll pitch yaw

                t.transform.rotation.x = q[0]
                t.transform.rotation.y = q[1]
                t.transform.rotation.z = q[2]
                t.transform.rotation.w = q[3]
            elif dynamic_links[i][j][0] in knee_key :
                q = tf_conversions.transformations.quaternion_from_euler(
                    math.radians(dynamic_links[i][j][4]), math.radians(dynamic_links[i][j][5] + p_inc) , math.radians(dynamic_links[i][j][6] ) )#roll pitch yaw
                t.transform.rotation.x = q[0]
                t.transform.rotation.y = q[1]
                t.transform.rotation.z = q[2]
                t.transform.rotation.w = q[3]    
            else :
                q = tf_conversions.transformations.quaternion_from_euler(
                    math.radians(dynamic_links[i][j][4]), math.radians(dynamic_links[i][j][5] ) , math.radians(dynamic_links[i][j][6] ) )#roll pitch yaw
                t.transform.rotation.x = q[0]
                t.transform.rotation.y = q[1]
                t.transform.rotation.z = q[2]
                t.transform.rotation.w = q[3]

            br.sendTransform(t)



def update_dic(move):
    move_set = move.data
    print(move_set)
    if move_set == 'fwd':
        for i in range(8):
            if i == 0:
                print('sup')
                servo_rot_callback(-30,0,tr1)
            elif i == 1:
                print('sup2')
                servo_rot_callback(-30,30,tr1)
            elif i == 2:
                print('sup3')
                servo_rot_callback(0,30,tr1)
            elif i == 3:
                print('sup4')
                servo_rot_callback(0,0,tr1)
            elif i == 4:
                print('sup5')
                servo_rot_callback(-30,0,tr2)
            elif i == 5:
                print('sup6')
                servo_rot_callback(-30,30,tr2)
            elif i == 6:
                print('sup7')
                servo_rot_callback(0,30,tr2)
            elif i == 7:
                print('sup8')
                servo_rot_callback(0,0,tr2)
            print(i)
            time.sleep(0.5)
    elif move_set=='Left': 
        for i in range(8):
            if i == 0:
                servo_rot_callback(-25,0,tr1)
            elif i == 1:
                servo_rot_callback(-25,25,tr1)
            elif i == 2:
                servo_rot_callback(0,25,tr1)
            elif i == 3:
                servo_rot_callback(0,0,tr1)
            elif i == 4:
                servo_rot_callback(-25,0,tr2)
            elif i == 5:
                servo_rot_callback(-25,25,tr2)
            elif i == 6:
                servo_rot_callback(0,25,tr2)
            elif i == 7:
                servo_rot_callback(0,0,tr2)
            time.sleep(0.5)

    else :
        for i in range(8):
            if i == 0:
                servo_rot_callback(-25,0,tr2)
            elif i == 1:
                servo_rot_callback(-25,25,tr2)
            elif i == 2:
                servo_rot_callback(0,25,tr2)
            elif i == 3:
                servo_rot_callback(0,0,tr2)
            elif i == 4:
                servo_rot_callback(-25,0,tr1)
            elif i == 5:
                servo_rot_callback(-25,25,tr1)
            elif i == 6:
                servo_rot_callback(0,25,tr1)
            elif i == 7:
                servo_rot_callback(0,0,tr1)
            time.sleep(0.5)



if __name__ == '__main__':

    rospy.init_node('tf_broadcaster')
    br = tf2_ros.TransformBroadcaster()
    try:   

        rate = rospy.Rate(1)
        t = geometry_msgs.msg.TransformStamped()

        while not rospy.is_shutdown():
            #rospy.Subscriber("servo_data",servo,update_dic)
            rospy.Subscriber("hex_movement",String,update_dic)
            #servo_rot_callback()
            rate.sleep()
        
    except rospy.ROSInterruptException:
        pass






#########################################################################
    # s = servo_data.name
    # global pre,tm,pp
    # global dynamic_links
    # angle = servo_data.angle


    # s1 = s[:-1]
    # if s[-1] =='H' :
    #     s2= 'A_' + s[:-1]
    # elif s[-1] =='K':
    #     s2= 'C_' + s[:-1]  
    # else :
    #     s2= 'E_' + s[:-1]




    # x_axis = dynamic_links[s1][s2][1]
    # y_axis = dynamic_links[s1][s2][2]
    # z_axis = dynamic_links[s1][s2][3]
    # r_axis = dynamic_links[s1][s2][4]
    # p_axis = dynamic_links[s1][s2][5]
    # yaw_axis = dynamic_links[s1][s2][6]

    # if s[0] == 'R' :
    #     tm = tm *(-1)
    # else :
    #     pp = pp  * (1)
    
   
    # if s[-1] == 'H' and s[0] == 'L':
    #     yaw_axis = yaw_axis - (tm)*angle
    # elif s[-1] == 'H' and s[0] == 'R':
    #     yaw_axis = yaw_axis + (pp)*angle
    # elif s[-1] == 'K' :
    #     p_axis = p_axis + (tm)*angle
    # print('-----------------------------',pre, tm,pp)
    # global counter
    # key = {s2 : [s,x_axis,y_axis,z_axis,r_axis,p_axis,yaw_axis]}
    # dynamic_links[s1].update(key)
    # print(' key dict == ',key)


    # if counter > 6 :
    #     pre = pre *(-1)
    #     # pre = -1
    #     # dynamic_links = init_pos
    #     counter = 0
    #     # print("----------------------------------------------------------")
    # print(counter)
    # servo_rot_callback()
    # counter = counter + 1