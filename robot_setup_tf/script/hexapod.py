#!/usr/bin/env python3

# Python ROS
#import rospy
from time import sleep
from adafruit_servokit import ServoKit


jp = {
    'LFH': (0,90), 'LFK': (1,60), 'LFA': (2,15),
    'LMH': (3,100), 'LMK': (4,60), 'LMA': (5,15),
    'LBH': (6,90), 'LBK': (7,60), 'LBA': (8,15),
    'RFH': (0,90), 'RFK': (1,120), 'RFA': (2,165),    
    'RMH': (3,100), 'RMK': (4,120), 'RMA': (5,175),
    'RBH': (6,90), 'RBK': (7,120), 'RBA': (8,165)
    }


init_values = {
    'LFH': (0,90), 'LFK': (1,60), 'LFA': (2,15),
    'LMH': (3,100), 'LMK': (4,60), 'LMA': (5,15),
    'LBH': (6,90), 'LBK': (7,60), 'LBA': (8,15),
    'RFH': (0,90), 'RFK': (1,120), 'RFA': (2,165),    
    'RMH': (3,100), 'RMK': (4,120), 'RMA': (5,175),
    'RBH': (6,90), 'RBK': (7,120), 'RBA': (8,165)
    }


jp2 = {
    'LFH': (0,100), 'LFK': (1,110), 'LFA': (2,10),
    'LMH': (3,90), 'LMK': (4,110), 'LMA': (5,10),
    'LBH': (6,100), 'LBK': (7,110), 'LBA': (8,10),
    'RFH': (0,90), 'RFK': (1,75), 'RFA': (2,170),    
    'RMH': (3,100), 'RMK': (4,75), 'RMA': (5,170),
    'RBH': (6,90), 'RBK': (7,75), 'RBA': (8,170)
    }

left_j= ['LFH', 'LFK','LFA','LMH','LMK','LMA','LBH','LBK','LBA']

kitL = ServoKit(channels=16,address=0x41)
kitR = ServoKit(channels=16,address=0x40)

for i in range(9):
	kitL.servo[i].set_pulse_width_range(500, 2500)
	kitR.servo[i].set_pulse_width_range(500, 2500)


class Hexapod:

	def __init__(self):
		self.left_front = Leg('left front', 'LFH', 'LFK', 'LFA')
		self.right_front = Leg('right front', 'RFH', 'RFK', 'RFA')
		self.left_middle = Leg('left middle', 'LMH', 'LMK', 'LMA')
		self.right_middle = Leg('right middle', 'RMH', 'RMK', 'RMA')
		self.left_back = Leg('left back', 'LBH', 'LBK', 'LBA')
		self.right_back = Leg('right back', 'RBH', 'RBK', 'RBA')

		self.legs = [self.left_front, self.right_front,
					self.left_middle, self.right_middle,
					self.left_back, self.right_back]
		self.legs2 = [self.left_front, self.left_middle,
					 self.left_back, self.right_front,
					 self.right_middle, self.right_back]					

		self.right_legs = [self.right_front, self.right_middle, self.right_back]
		self.left_legs = [self.left_front, self.left_middle, self.left_back]

		self.tripod1 = [self.left_front, self.right_middle, self.left_back]
		self.tripod2 = [self.right_front, self.left_middle, self.right_back]

		self.hips, self.knees, self.ankles = [], [], []

		for leg in self.legs:
			self.hips.append(leg.hip)
			self.knees.append(leg.knee)
			self.ankles.append(leg.ankle)


	def initialize(self):
		for leg in self.legs:
			leg.off()
	

	def walking(self,steps,t,fwd):
		for step in range(steps) :
			print(jp)
			self.tripod_gait(self.tripod1,0,fwd)
			sleep(t)
			if steps > 0:
				self.tripod_gait(self.tripod2,3,fwd)
				sleep(t)	
			self.tripod_gait(self.tripod1,1,fwd)
			sleep(t)
			self.tripod_gait(self.tripod1,2,fwd)
			sleep(t)	
			self.tripod_gait(self.tripod2,0,fwd)
			sleep(t)
			self.tripod_gait(self.tripod1,3,fwd)
			sleep(t)
			self.tripod_gait(self.tripod2,1,fwd)
			sleep(t)
			self.tripod_gait(self.tripod2,2,fwd)
			sleep(t)
			if step == steps-1:
				self.tripod_gait(self.tripod2,3,fwd) #last step to actually go in the init position 
			
	def tripod_gait(self,tr1,seq,fwd):
		if fwd ==False :
			if seq == 0 : #set knee up
				for t in tr1:
					t.knee.move(30)
			elif seq == 1:  # move forward
				for t in tr1:
					t.hip.move(-30)
			elif seq == 2 : # set knee down
				for t in tr1:
					t.knee.move(-30)
			elif seq == 3 : # set hip back
				for t in tr1:
					t.off()
		else :
			if seq == 0 : #set knee up
				for t in tr1:
					t.knee.move(30)
			elif seq == 1:  # move forward
				for t in tr1:
					t.hip.move(30)
			elif seq == 2 : # set knee down
				for t in tr1:
					t.knee.move(-30)
			elif seq == 3 : # set hip back
				for t in tr1:
					t.off()



	def rotate_clockwise(self,steps,t,cw):
		for step in range(steps) :
			self.rotation_seq(self.tripod2,0,cw)
			sleep(t)
			if steps > 0:
				self.tripod_gait(self.tripod1,3,cw)
				sleep(0.2)	
			self.rotation_seq(self.tripod2,1,cw)
			sleep(t)
			self.rotation_seq(self.tripod2,2,cw)
			sleep(t)	
			# self.rotation_seq(self.tripod1,3,cw)
			# sleep(t)
			self.rotation_seq(self.tripod1,0,cw)
			sleep(t)
			self.rotation_seq(self.tripod2,3,cw)
			sleep(t)
			self.rotation_seq(self.tripod1,1,cw)
			sleep(t)
			self.rotation_seq(self.tripod1,2,cw)
			sleep(t)

			if step == steps-1 :
				self.rotation_seq(self.tripod1,3,cw)

	def rotation_seq(self,tr1,seq,cw):
		if seq == 0 :
			for t in tr1:
				t.knee.move(30)
		elif seq == 1:
			if cw == True : # rotation clockwise
				if tr1 == self.tripod2 :
					for t in tr1:
						if t.hip.name == 'LMH':
							t.hip.move(25)
						else :
							t.hip.move(-25)
				else:
					for t in tr1:
						if t.hip.name == 'RMH':
							t.hip.move(-25)
						else :
							t.hip.move(25)
			else : ## rotation left 
				if tr1 == self.tripod2 :
					for t in tr1:
						if t.hip.name == 'LMH':
							t.hip.move(-25)
						else :
							t.hip.move(+25)
				else:
					for t in tr1:
						if t.hip.name == 'RMH':
							t.hip.move(25)
						else :
							t.hip.move(-25)
								
		elif seq == 2 :
			for t in tr1:
				t.knee.move(-25)
		elif seq == 3 :
			for t in tr1:
				t.hip.off()
				t.knee.off()


	def smooth(self,t=0,pi=45):
		print("-->>",pi)
		while True :
			for leg in self.legs:
				leg.knee.move(25)
			sleep(2)
			for leg in self.legs:
				leg.knee.move(-25)
			sleep(2)

	def wave_gait(self,steps,t,fwd=True):
		for step in range(steps) :
			for leg in self.legs2 :
				leg.knee.move(30)
				sleep(t)
				leg.hip.move(30)
				sleep(t)
				leg.knee.move(-30)
 			# reset after 1 step 
			
			self.initialize()
 	
	

	def standUp(self):
		for leg in self.legs:
			leg.knee.move(-50)
			sleep(0.05)
			leg.ankle.move(-50)
			print(leg.ankle)
			sleep(0.05)


class Leg:
	def __init__(self, name, hip_key, knee_key, ankle_key):
		
		self.hip =Joint("hip",hip_key)
		self.knee = Joint("knee",knee_key)
		self.ankle = Joint("ankle",ankle_key)
		self.name = name
		self.joints = [self.hip, self.knee, self.ankle]


	def do_move(self):
		for joint in self.joints:
			joint.move(30)
			
	def off(self):
		for joint in self.joints:
			joint.off()

	def __repr__(self):
		return 'leg: ' + self.name


class Joint:
	
	def __init__(self, joint_type, jkey):
		self.joint_type, self.name =  joint_type, jkey
		self.channel, self.angle = jp[jkey]


	def move(self, angle):
		if self.name in left_j:
			kitL.servo[self.channel].angle = self.angle - angle
			self.angle = kitL.servo[self.channel].angle
			key = {self.name : (self.channel , self.angle)}
			#updaste dictionary values -> 
			jp.update(key)
		else:
		 	kitR.servo[self.channel].angle = self.angle + angle
		 	self.angle = kitR.servo[self.channel].angle
		 	key = {self.name : (self.channel , self.angle)}
		 	jp.update(key)
		

	def set_angle(self,angle):
		sleep(0.15)
		if self.name in left_j:
			kitL.servo[self.channel].angle = angle
			self.angle = kitL.servo[self.channel].angle
		else:
		 	kitR.servo[self.channel].angle = angle
		 	self.angle = kitR.servo[self.channel].angle
			

	def off(self):
		if self.name in left_j:
			#print("off joint :" ,self.name, self.angle)
			kitL.servo[self.channel].angle = init_values[self.name][1]
			self.angle = kitL.servo[self.channel].angle
			print(self.name, self.angle)
		else:
		 	kitR.servo[self.channel].angle = init_values[self.name][1]
		 	self.angle = kitR.servo[self.channel].angle
		 	print(self.name, self.angle)


	def __repr__(self):
		return 'joint: ' + self.joint_type + ' : ' + self.name + ' angle: ' + str(self.angle)




if __name__ == '__main__':
	
	hex1 = Hexapod()
	hex1.initialize()
	#print(hex1.tripod1[1].knee)\
	sleep(1)
	#hex1.standUp()

	#hex1.left_middle.knee.set_angle(50)
	#hex1.right_front.ankle.set_angle(180)
	hex1.rotate_clockwise(3,0.2,True)
	hex1.walking(5,0.1,False)

	#hex1.wave_gait(10,0.3,False)
	


	# #two fron legsss up
	# sleep(0.2)
	# hex1.right_middle.hip.set_angle(125)
	# hex1.left_middle.hip.set_angle(65)


	# hex1.right_back.ankle.set_angle(170)
	# hex1.right_back.knee.set_angle(165)

	# hex1.left_front.ankle.set_angle(45)
	# hex1.left_front.knee.set_angle(30)
	# for x in range(4):
	# 	sleep(2)
	# 	hex1.right_back.hip.set_angle(55)
	# 	sleep(2)
	# 	hex1.right_back.hip.set_angle(125)




	# # while True :
	# 	hex1.right_front.ankle.set_angle(180)
	# 	sleep(2)
	# 	hex1.right_front.ankle.set_angle(0)
	# 	sleep(2)

	# 	hex1.left_front.ankle.set_angle(0)
	# 	sleep(2)
	# hex1.left_back.ankle.set_ankle(180)
	#hex1.wave_gait(5,0.35,False)





