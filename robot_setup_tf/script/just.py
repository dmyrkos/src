#!/usr/bin/env python3 

import glom
import rospy
import time
from robot_setup_tf.msg import Servo_tf 



# dynamic_links =  {
#                 "RF" :{ "A_RF":["RFH",0.01,-0.01,0,0,0,0], "RFH":["B_RF",0.05,-0.05,0,0,0,0],  "C_RF":["RFK",0,0,0,0,0,0], 
#                         "RFK":["D_RF",0.027,-0.027,0,0,0,0] , "E_RF":["RFA",0,0,0,0,0,0] , "RFA":["F_RF",0.02,-0.02,0,0,0,0]},
#                 "LF" : { "A_LF":["LFH",0.01,0.01,0,0,0,0], "LFH":["B_LF",0.05,0.05,0,0,0,0],  "C_LF":["LFK",0,0,0,0,0,0], 
#                         "LFK":["D_LF",0.027,0.027,0,0,0,0] , "E_LF":["LFA",0,0,0,0,0,0] , "LFA":["F_LF",0.02,0.02,0,0,0,0]},
#                 "RM" : { "A_RM":["RMH",0,-0.01,0,0,0,0], "RMH":["B_RM",0,-0.05,0,0,0,0],  "C_RM":["RMK",0,0,0,0,0,0], 
#                         "RMK":["D_RM",0,-0.027,0,0,0,0] , "E_RM":["RMA",0,0,0,0,0,0] , "RMA":["F_RM",0,-0.02,0,0,0,0]},
#                 "LM": { "A_LM":["LMH",0,0.01,0,0,0,0], "LMH":["B_LM",0,0.05,0,0,0,0],  "C_LM":["LMK",0,0,0,0,0,0], 
#                         "LMK":["D_LM",0,0.027,0,0,0,0] , "E_LM":["LMA",0,0,0,0,0,0] , "LMA":["F_LM",0,0.02,0,0,0,0]},
#                 "RB":  { "A_RB":["RBH",-0.01,-0.01,0,0,0,0], "RBH":["B_RB",-0.05,-0.05,0,0,0,0],  "C_RB":["RBK",0,0,0,0,0,0], 
#                         "RBK":["D_RB",-0.027,-0.027,0,0,0,0] , "E_RB":["RBA",0,0,0,0,0,0] , "RBA":["F_RB",-0.02,-0.02,0,0,0,0]},
#                 "LB":  { "A_LB":["LBH",-0.01,0.01,0,0,0,0], "LBH":["B_LB",-0.05,0.05,0,0,0,0],  "C_LB":["LBK",0,0,0,0,0,0], 
#                         "LBK":["D_LB",-0.027,0.027,0,0,0,0] , "E_LB":["LBA",0,0,0,0,0,0] , "LBA":["F_LB",-0.02,0.02,0,0,0,0]}
#                  }

jp = {
    'LFH': (0,100), 'LFK': (1,110), 'LFA': (2,0),
    'LMH': (3,100), 'LMK': (4,110), 'LMA': (5,0),
    'LBH': (6,100), 'LBK': (7,110), 'LBA': (8,0),
    'RFH': (0,100), 'RFK': (1,70), 'RFA': (2,180),    
    'RMH': (3,100), 'RMK': (4,70), 'RMA': (5,180),
    'RBH': (6,100), 'RBK': (7,70), 'RBA': (8,180)
    }




# tmp= [ 'dfd',4,6,7,4]

# #glom.assign[dynamic_links,"RF.A_RF",tmp]
# print(type(dynamic_links['RF']['A_RF'])
# dynamic_links['RF']['A_RF'].insert(3,99)
# print(dynamic_links['RF']['A_RF'])










if __name__ == '__main__':
    rospy.init_node("servo_data",anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        servo_pub = rospy.Publisher("servo_data",Servo_tf,queue_size=10)
        s = Servo_tf()
        connections = servo_pub.get_num_connections()
        if connections >0 :
            for i in jp:
                s.header.frame_id = str(i)
                s.header.stamp = rospy.Time.now()
                #s.servo_id = i 
                s.servo_deg =int(jp[i][1])
                servo_pub.publish(s)
                rospy.loginfo("sent")
            break
        rate.sleep()
        

