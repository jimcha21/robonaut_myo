#!/usr/bin/env python

import rospy
from std_msgs.msg import UInt8, String
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Imu
from ros_myo.msg import MyoArm
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String, UInt16MultiArray, MultiArrayLayout, MultiArrayDimension
from collections import namedtuple
import os

def colour(script,clr):
    #clr
    #30-black
    #31-red
    #32-green
    #33-yellow
    #34-blue
    #35-purple
    #36-blue
    #37-default white
    return "\033["+str(clr)+"m" + script + "\033[0m"

def getmyo_IMU(data):
	global roll
	global pitch
	global yaw
	roll=data.x
	pitch=data.y
	yaw=data.z


if __name__ == '__main__':

	rospy.init_node('ARM_Limitations_node', anonymous=True)
	pub = rospy.Publisher("arm_limitations", Float32MultiArray, queue_size=10)

	global roll
	global pitch
	global yaw

	os.system("clear")
	a=raw_input("Set new Arm Pose Limitations? [y for yes, anything else to quit]\n\n--> ")
	if a=="y":
		print ("Setting the Arm Pose limits..")
		rospy.Subscriber("myo_imuEulerAngles_raw", Vector3,getmyo_IMU)
		#rospy.Subscriber("myo_arm", MyoArm,getarm)
		
		while True:
			a=raw_input("Is Myo Face Up ? (press y or n for face down) : ") #face up is on thalmic labs logo is upside down when myo is worn on hand, face down the opposite
			if a=="y":
				print ("\nMyo is face up .\nMoving on Arm Limitations..\n")
				myo_face=1
				break
			elif a=="n":
				print ("\nMyo is face down .\nMoving on Arm Limitations..\n")
				myo_face=0
				break

			
		#Gia to yaw - th peristrofh tou xeriou (sto myo einai to yaw, kathws einai foremeno plagiws ws pros ton aksona y'y)
		while True:
			a=raw_input("Please roll your hand anti-clockwise, to your body.. (press y when finish) : ")
			if a=="y":
				#saving the imu data value from the myo..
				roll_max=yaw
				
				a=raw_input("Please roll your hand clockwise, away from your body.. (press y when finish) : ")
				if a=="y":
					roll_min=yaw
					if roll_max<roll_min: # myo is on left-hand
						a=roll_max
						roll_max=roll_min
						roll_min=a
					print(colour("Got Roll Max Limit : " + str(roll_max),33))
					print(colour("Got Roll Min Limit : " + str(roll_min),33))
					break
		print("\nContinuing on the Arm, left and right, movement...")
		while True:
			a=raw_input("Please stretch your hand at your right.. (press y when finish) : ")
			if a=="y":
				#saving the imu data value from the myo..
				yaw_max=roll
				print(colour("Got Yaw Max Limit : " + str(yaw_max),33))
				a=raw_input("And at your left.. (press y when finish) : ")
				if a=="y":
					yaw_min=roll
					print(colour("Got Yaw Min Limit : " + str(yaw_min),33))
					if yaw_max<yaw_min: # myo is on left-hand
						yaw_init=18-yaw_min
						yaw_max=yaw_max+yaw_init
						yaw_min=yaw_min+yaw_init
						#print("dinw ta telika l="+ str(b) +" h="+ str(a) )
						if yaw_min>=18:
							yaw_min=yaw_min-18
					else:
						yaw_init=0
					break
		print("\nContinuing on the Arm, up and down, movement...")
		while True:
			a=raw_input("Please stretch up your hand.. (press y when finish) : ")
			if a=="y":
				#saving the imu data value from the myo..
				pitch_min=pitch
				a=raw_input("Please stretch down your hand... (press y when finish) : ")
				if a=="y":
					pitch_max=pitch
					if pitch_max<pitch_min: # myo is on left-hand
						a=pitch_max
						pitch_max=pitch_min
						pitch_min=a
					print(colour("Got Pitch Min Limit : " + str(pitch_min),33))
					print(colour("Got Pitch Max Limit : " + str(pitch_max),33))
					break		
		print(colour("\nArm Pose Limits received succesfully and just published..",32))
		pub.publish(Float32MultiArray(MultiArrayLayout(data_offset=1), [myo_face,yaw_init,roll_max,roll_min,pitch_max,pitch_min,yaw_max,yaw_min]))
	else:
		print(colour("\nexiting now..",31))

	rospy.loginfo('Terminating the node...')
	#rospy.spin()
