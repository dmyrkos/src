#!/usr/bin/env python3

from hexapod import *
import rospy
import ros_sonar
from robot_setup_tf.msg import Sonar
from sensor_msgs.msg import Imu
import geometry_msgs.msg
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, TransformStamped, AccelStamped
import std_msgs 
from std_msgs.msg import Float64
from time import sleep


class Enviroment:
	def __init__(self):
		self.sonar_RF = Sonar_d('right_front',None,2,0)
		self.sonar_LF = Sonar_d('left_front',None,3,0)
		self.sonar_RB = Sonar_d('right_back',None,1,0)
		self.sonar_LB = Sonar_d('left_back',None,0,0)
		self.imu= Imu_d()
		self.sonars = [self.sonar_RF,self.sonar_LF,self.sonar_RB,self.sonar_LB]


class Sonar_d:
	def __init__(self,name,header,num,distance):
		self.name = name
		self.header = header
		self.num = num
		self.distance = distance
		rospy.Subscriber("sonar_data", Sonar, self.sonar_callback)


	def sonar_callback(self,data):
		self.header =data.header
		self.distance = data.distance[self.num]

	def avg_distance(self):
		distance =0 
		for i in range(10):
			distance = distance + self.distance
		return distance/10
		

	def __repr__(self):
		# return 'Sonar name : ' + self.name + ' Sonar Header : ' + str(self.header) + 'Sonar Distance : ' + str(self.distance)
		return f'Sonar Header : {str(self.header)} ,Name and Distance : {self.name ,str(self.distance)})'

class Imu_d :
	def __init__(self):
		self.header = None
		self.orientation = None
		self.angular_velocity = None
		self.linear_acceleration = None
		rospy.Subscriber("bno055/imu", Imu, self.Imu_callback)

	def Imu_callback(self,data):
		self.header = data.header
		self.orientation = data.orientation
		self.angular_velocity = data.angular_velocity
		self.linear_acceleration = data.linear_acceleration

	def __repr__(self):
		return f' orientation : {str(self.orientation)} , Angular_velocity : {str(self.angular_velocity)} , Linear_accelaration : {str(self.linear_acceleration)}'




###############################################################
###########initialize -> check nviroment -> action  ->? repeat ###
	####### check enviroment
			# sonar subscriber 
			# imu sub 
			# laser sub

##################################################################
 

def main():
	# rospy.Subscriber("sonar_data", Sonar, check_sonar)
	init_state()
	
	rate_s=rospy.Rate(10)
	while not rospy.is_shutdown():
		#hex1.rotate_clockwise(3,0.2,True)
		#hex1.rotate_clockwise(3,0.2,False)
		# if env.sonar_RF.distance < 30 :
		# 	print('gotsassasaassssssssssssssssss')
		print(env.imu)
		rate_s.sleep()



# key set depends of the enviroment signals (move fwd /bwd rotate clkwise/counterclkwise , init stature //pose )


# def action_set(key,stepsmtime,fwd):

# 	if key== 0 :
# 		hex1.walking(steps,time,fwd)
# 	elif key == 1:
# 		hex1.walking(steps,time,fwd)
# 	else :
# 		hex1.initialize()


# def check_sonar(data) :
# 	for i,sonar in enumerate(env.sonars):
# 		sonar.header=data.header
# 		sonar.distance = data.distance[i]
# 	print(env.sonar_RF)

def init_state():
	print('Initializing ...')
	sleep(2)
	#hex1.initialize()


if __name__ == '__main__':
	
	rospy.init_node('main_code')
	hex1 = Hexapod()
	env = Enviroment()
	main()
