#!/usr/bin/env python3

from hexapod import *
import rospy
import ros_sonar
from robot_setup_tf.msg import Sonar


class Enviroment:
	def __init__(self):
		self.sonar_RF = Sonar_d('right_front',None,0)
		self.sonar_LF = Sonar_d('left_front',None,0)
		self.sonar_RB = Sonar_d('right_back',None,0)
		self.sonar_LB = Sonar_d('left_back',None,0)
		self.imu_data = Imu(0)
		self.sonars = [self.sonar_RF,self.sonar_LF,self.sonar_RB,self.sonar_LB]

	def get_values(self):
		for sonar in self.sonars:
			sonar.distance

class Sonar_d:
	def __init__(self,name,header,distance):
		self.name = name
		self.header = header
		self.distance= distance

	def __repr__(self):
		# return 'Sonar name : ' + self.name + ' Sonar Header : ' + str(self.header) + 'Sonar Distance : ' + str(self.distance)
		return f'Sonar Header : {str(self.header)} ,Name and Distance : {self.name ,str(self.distance)})'

class Imu :
	def __init__(self,data):
		self.date=data


##################################################################
###########initialize -> check nviroment -> action  ->? repeat ###
	####### check enviroment
			# sonar subscriber 
			# imu sub 
			# laser sub

##################################################################


def main():
	rospy.Subscriber("sonar_data", Sonar, check_sonar)


	#hex1.initialize()
	rate=rospy.Rate(5)
	while not rospy.is_shutdown():

		#hex1.rotate_clockwise(3,0.2,True)
		#hex1.rotate_clockwise(3,0.2,False)
		print(env.sonars[0])
		rate.sleep()



# key set depends of the enviroment signals (move fwd /bwd rotate clkwise/counterclkwise , init stature //pose )


# def action_set(key,stepsmtime,fwd):

# 	if key== 0 :
# 		hex1.walking(steps,time,fwd)
# 	elif key == 1:
# 		hex1.walking(steps,time,fwd)
# 	else :
# 		hex1.initialize()


def check_sonar(data) :
	for i,sonar in enumerate(env.sonars):
		sonar.header=data.header
		sonar.distance = data.distance[i]
	print(env.sonar_RF)
	# env.sonars = data.dat
	# env.sonar_RF.distance=data.distance[0]
	# print(env.sonar_RF.header,env.sonar_RF.name)
	# print(env.sonars)
	# print(env.sonar_RB.distance)




if __name__ == '__main__':
	
	rospy.init_node('main_code')
	hex1 = Hexapod()
	env = Enviroment()
	main()
