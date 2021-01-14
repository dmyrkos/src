#!/usr/bin/env python3

from hexapod import *
import rospy
import ros_sonar
from robot_setup_tf.msg import Sonar





def main():
	#hex1.rotate_clockwise(3,0.2,True)
	rospy.Subscriber("sonar_data", Sonar, check_sonar)
	rospy.spin()
	#hex1.rotate_clockwise(3,0.2,False)

# check enviroment 






def check_sonar(data) :

	rospy.loginfo(rospy.get_caller_id() + " %f", data.data)


if __name__ == '__main__':

	rospy.init_node('main_code')
	hex1 = Hexapod()
	main()

