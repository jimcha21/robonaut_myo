#!/usr/bin/env python

## Simple myo demo that listens to std_msgs/UInt8 poses published 
## to the 'myo_gest' topic

import rospy
from std_msgs.msg import UInt8, Float32,String
from sensor_msgs.msg import Imu
from ros_myo.msg import MyoArm

########## Data Enums ###########
# MyoArm.arm___________________ #
#    UNKNOWN        = 0 	#
#    RIGHT          = 1		#
#    Left           = 2		#
# MyoArm.xdir___________________#
#    UNKNOWN        = 0		#
#    X_TOWARD_WRIST = 1		#
#    X_TOWARD_ELBOW = 2		#
# myo_gest UInt8________________#
#    REST           = 0		#
#    FIST           = 1		#
#    WAVE_IN        = 2		#
#    WAVE_OUT       = 3		#
#    FINGERS_SPREAD = 4		#
#    THUMB_TO_PINKY = 5		#
#    UNKNOWN        = 255	#
#################################


if __name__ == '__main__':

    global armState
    global xDirState
    global repeats
    global param
    global miss
    global last_pose
    global block_size #every 280 gest data, a pose decision is made.
    global i
    repeats=0;
    param=0.0;
    miss=0;
    sampling_block=280;
    i=10
    last_pose="rest"
    armState = 0;
    rospy.init_node('hand_gesture_node', anonymous=True)

    msg = rospy.Publisher("myo_handgest", String, queue_size=10)
    #r = rospy.Rate(10)
    # set the global arm states
    def setArm(data):
        global armState
        global xDirState
        armState = data.arm
        xDirState = data.xdir
    	rospy.sleep(2.0)

    def pose_decide(gest_num):
        
        global repeats
        global param
        global miss
        global block_size
        global i
        global last_pose

        repeats+=1
        if repeats==1:
            param=gest_num.data
        elif repeats<sampling_block:
            if param!=gest_num.data:
                miss=miss+1
                #msg.publish(str(param)+" sygkrish me " + str(gest_num.data))
        elif repeats==sampling_block: #the decision time
            if miss<80:
                if param==0:
                    pose="rest"
                    i=0
                elif param==1:
                    pose="fist"
                    i=1
                elif param==2:
                    pose="wave in"
                    i=2
                elif param==3:
                    pose="wave out"
                    i=3
                elif param==4:
                    pose="finger spread"
                    i=4
                elif param==5:
                    pose="thumps up"
                    i=5
                elif param==6:
                    pose="grab"
                    i=6
            else: ##remains the same last pose
                if i==0:
                    pose="rest"
                elif i==1:
                    pose="fist"
                elif i==2:
                    pose="wave in"
                elif i==3:
                    pose="wave out"
                elif i==4:
                    pose="finger spread"
                elif i==5:
                    pose="thumps up"
                elif i==6:
                    pose="grab"
                else:
                    pose="UNKNOWN pose"

            #lets see if the pose is similar to the previous posted..
            if pose!=last_pose:
                msg.publish(pose)
                last_pose=pose
            #else: do nothing

        else:
            repeats=0
            miss=0
	    
    rospy.Subscriber("myo_arm", MyoArm, setArm)
    rospy.Subscriber("myo_gest", Float32, pose_decide)
    rospy.loginfo('Please sync the Myo')

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
