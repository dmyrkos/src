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
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import tf_conversions
import tf2_ros




#FORWARD = F 
#BACKWARD = B
# LEFT = L
# RIGHT = R
#i.e.  ['F','F','L',F]
# forward /bacward = +/- 0.075
# left ot right = + /- 0.25 (aprox 30 degrees)

move_set =['F']
forward = False
backwards = True 
left =  False
right = True

class Enviroment:
	def __init__(self):
		self.sonar_RF = Sonar_d('right_front',None,1,0)
		self.sonar_LF = Sonar_d('left_front',None,0,0)
		self.sonar_RB = Sonar_d('right_back',None,2,0)
		self.sonar_LB = Sonar_d('left_back',None,3,0)
		self.imu= Imu_d()
		# self.Laser = Laser_d()
		self.planner = Planner_d()
		self.sonars = [self.sonar_RF,self.sonar_LF,self.sonar_RB,self.sonar_LB]
		self.set = None
		self.tmp_dir = None

	def check_enviroment(self,direction):
		y=36
		if direction =='F':
			self.tmp_dir = 'F'
			if self.sonar_RF.distance <=y and self.sonar_LF.distance <=y  : # 1 AND 1
				self.set = 'R'
				move_set.append(self.set)
			elif self.sonar_RF.distance <= y and self.sonar_LF.distance > y : #1 AND 0
				self.set = 'L'
				move_set.append(self.set)
			elif self.sonar_RF.distance > y and self.sonar_LF.distance<=y :  # 0 AND 1
				self.set = 'R'
				move_set.append(self.set)
			elif self.sonar_RF.distance > y and self.sonar_LF.distance > y :  # 0 AND 0
				self.set = 'F'
				move_set.append(self.set)

		elif direction == 'B':
			self.tmp_dir = 'B'
			if self.sonar_RB.distance <=y and self.sonar_LB.distance <=y  : # 1 AND 1
				self.set = 'R'
				move_set.append(self.set)
			elif self.sonar_RB.distance <= y and self.sonar_LB.distance > y : #1 AND 0
				self.set = 'L'
				move_set.append(self.set)
			elif self.sonar_RB.distance > y and self.sonar_LB.distance<=y :  # 0 AND 1
				self.set = 'R'
				move_set.append(self.set)
			elif self.sonar_RB.distance > y and self.sonar_LB.distance > y :  # 0 AND 0
				self.set = 'B'
				move_set.append(self.set)


		elif direction == 'L' :

			if self.tmp_dir == 'F' :

				if self.sonar_RF.distance <=y and self.sonar_LF.distance <=y  : # 1 AND 1
					self.set = 'L'
					move_set.append(self.set)
				elif self.sonar_RF.distance <= y and self.sonar_LF.distance > y : #1 AND 0
					self.set = 'L'
					move_set.append(self.set)
				elif self.sonar_RF.distance > y and self.sonar_LF.distance<=y :  # 0 AND 1
					self.set = 'R'
					move_set.append(self.set)
				elif self.sonar_RF.distance > y and self.sonar_LF.distance > y :  # 0 AND 0
					self.set = 'F'
					move_set.append(self.set)

			elif self.tmp_dir == 'B' :

				if self.sonar_RB.distance <=y and self.sonar_LB.distance <=y  : # 1 AND 1
					self.set = 'L'
					move_set.append(self.set)
				elif self.sonar_RB.distance <= y and self.sonar_LB.distance > y : #1 AND 0
					self.set = 'L'
					move_set.append(self.set)
				elif self.sonar_RB.distance > y and self.sonar_LB.distance<=y :  # 0 AND 1
					self.set = 'R'
					move_set.append(self.set)
				elif self.sonar_RB.distance > y and self.sonar_LB.distance > y :  # 0 AND 0
					self.set = 'B'
					move_set.append(self.set)
			

		elif direction == 'R':

			if self.tmp_dir == 'F' :

				if self.sonar_RF.distance <=y and self.sonar_LF.distance <=y  : # 1 AND 1
					self.set = 'R'
					move_set.append(self.set)
				elif self.sonar_RF.distance <= y and self.sonar_LF.distance > y : #1 AND 0
					self.set = 'L'
					move_set.append(self.set)
				elif self.sonar_RF.distance > y and self.sonar_LF.distance<=y :  # 0 AND 1
					self.set = 'R'
					move_set.append(self.set)
				elif self.sonar_RF.distance > y and self.sonar_LF.distance > y :  # 0 AND 0
					self.set = 'F'
					move_set.append(self.set)

			elif self.tmp_dir == 'B' :

				if self.sonar_RB.distance <=y and self.sonar_LB.distance <=y  : # 1 AND 1
					self.set = 'R'
					move_set.append(self.set)
				elif self.sonar_RB.distance <= y and self.sonar_LB.distance > y : #1 AND 0
					self.set = 'L'
					move_set.append(self.set)
				elif self.sonar_RB.distance > y and self.sonar_LB.distance<=y :  # 0 AND 1
					self.set = 'R'
					move_set.append(self.set)
				elif self.sonar_RB.distance > y and self.sonar_LB.distance > y :  # 0 AND 0
					self.set = 'B'
					move_set.append(self.set)

		return self.set



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
		self.update_tf_imu()

	def update_tf_imu(self):
		imu_br = tf2_ros.TransformBroadcaster()
		imu_t = geometry_msgs.msg.TransformStamped()

		imu_t.header.stamp = rospy.Time.now()
		imu_t.header.frame_id = "base_link"
		imu_t.child_frame_id = "bno055/imu"
		imu_t.transform.translation.x= 0.05
		imu_t.transform.translation.y= 0
		imu_t.transform.translation.z= 0
		imu_t.transform.rotation= self.orientation
		imu_br.sendTransform(imu_t)


	def __repr__(self):
		return f' orientation : {str(self.orientation)} , Angular_velocity : {str(self.angular_velocity)} , Linear_accelaration : {str(self.linear_acceleration)}'

class Planner_d:
	def __init__(self):
		self.l = None
		self.a = None
		self.linear_x  ,self.linear_y , self.linear_z= None , None , None
		self.angular_x ,self.angular_y  ,self.angular_z= None , None , None
		self.plan=False
		rospy.Subscriber("cmd_vel",Twist,self.Planner_callback)

	def Planner_callback(self,data):
		self.l = data.linear.x
		self.a =data.angular.z
		self.linear_x , self.linear_y  ,self.linear_z = data.linear.x ,data.linear.y ,data.linear.z
		self.angular_x , self.angular_y , self.angular_z = data.angular.x ,data.angular.y ,data.angular.z
		self.plan = True


	def __repr__(self):
		return f' twist msg : linear_acceleration :{str(self.l)} , angular_velocity{str(self.a)}  '


# class Laser_d :
# 	def __init__(self):
# 		rospy.subscriber("/scan",LaserScan,self.Laser_callback)

# 	def Laser_callback(self):
# 		pass
 

 

def main():
	init_state()
	keySet = None 
	x=0
	rate_s=rospy.Rate(20)
	while not rospy.is_shutdown():
		if env.planner.linear_x != None or env.planner.angular_z !=None:
			waypoint_action()
			print('hey')
		# keySet =  env.check_enviroment(move_set[x])
		# action_set(keySet)
		# x=x+1
		print(env.planner)
		# print(env.imu.orientation)
		rate_s.sleep()



# key set depends of the enviroment signals (move fwd /bwd rotate clkwise/counterclkwise , init stature //pose )
def waypoint_action():
	t=0.15
	seq = True
	x = env.planner.linear_x
	z = env.planner.angular_z
	step_x = 0.012 # 1 fwd step
	rot_z = 0.25 # 1 rotation right
	steps = abs(x)/step_x
	rotation = abs(z)/ rot_z
	print('--------- step and rotaion ------ ',steps,rotation)
	for i in range(int(rotation)):
		if z > 0 :
			rotate(t,left)
		else :
			rotate(t,right)
	sleep(t)
	for i in range(int(steps)):
		if x>0 :
			step(t,forward)
		else :
			step(t,backwards)
	



def action_set(key):

	t = 0.15
	if key == 'F':
		step(t,forward)
	elif key == 'B':
		step(t,backwards)
	elif key == 'R':
		rotate(t,right)
	elif key == 'L':
		rotate(t,left)
		


def step(t,fwd):
	hex1.walking(1,t,fwd)

	movemnt_pub('fwd')

def rotate(t,clockwise):
	hex1.rotate_clockwise(1,t,clockwise)
	if clockwise == True :
		movemnt_pub('Right')
	else :
		movemnt_pub('Left')

def init_state():
	print(' Initializing ...')
	sleep(2)
	hex1.initialize()
	sleep(2)


def movemnt_pub(move):
	hex_movement = rospy.Publisher("hex_movement",String,queue_size=3)
	# cmd_vel_pub = rospy.Publisher("cmd_vel",Twist,queue_size=10)
	# cmd_vel_data.linear,cmd_vel_data.angular  = Twist()
	# if move == 'fwd' :
	# 	cmd_vel_data.linear = 0.08
	# 	cmd_vel_data.angular= 0
	# elif move == 'bwd' :
	# 	cmd_vel_data.linear = -0.08
	# 	cmd_vel_data.angular= 0
	# cmd_vel_pub.publish(cmd_vel_data)
	s = String()
	s = str(move)
	hex_movement.publish(s)



if __name__ == '__main__':
	
	rospy.init_node('main_code')
	hex1 = Hexapod()
	env = Enviroment()
	main()
	print(move_set)

