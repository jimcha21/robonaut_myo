#!/usr/bin/env python

## Simple myo demo that listens to std_msgs/UInt8 poses published 
## to the 'myo_gest' topic

import rospy
import os
from std_msgs.msg import UInt8, Float32,String
from sensor_msgs.msg import Imu
from ros_myo.msg import MyoArm
from geometry_msgs.msg import Vector3
import sys,tty,termios

########## Menu Controller ########################
# Options___________________                      #
#    Reset the Simulation = 1 	                  #
#    Wake up the Robonaut = 2		              #
#    Set Hand Pose Limits = 3	  	              #
#    Set the Power of the Joint Efforts = 4       #
#    Enable Hand Control  = 5                     #
#    Disable Hand Control = 6                     #
#    Set Arm Pose Limits  = 7                     #
#    Enable Arm Control   = 8                     #
#    Disable Arm Control  = 9                     #
#    {Set Robonauts Default Position} = 10       #
#    Exit = 0                                     #
###################################################


class _GetArrowKey:
    def __call__(self):
            fd = sys.stdin.fileno()
            old_settings = termios.tcgetattr(fd)
            try:
                tty.setraw(sys.stdin.fileno())
                ch = sys.stdin.read(3)
            finally:
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            return ch

def getArrowForWaistMovement():
        inkey = _GetArrowKey()
        while(1):
                k=inkey()
                if k!='':break
        if k=='\x1b[C':
            arrow_msg.publish("right")
            return "right"  
        elif k=='\x1b[D':
            arrow_msg.publish("left")
            return "left"
        elif k=='\x1b[A':
            arrow_msg.publish("up")
            return "up"
        elif k=='\x1b[B':
            arrow_msg.publish("down")
            return "down"
        else:
            arrow_msg.publish("unknown")
            return "unknown"  

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



if __name__ == '__main__':

    rospy.init_node('Simulation_Controller_Menu_node', anonymous=True)

    msg = rospy.Publisher("user_messages", UInt8, queue_size=10)
    effort_msg = rospy.Publisher("user_effort_specification", Float32, queue_size=10)
    arrow_msg = rospy.Publisher("user_waist_pose_specification", String, queue_size=10)
    imus_limitations=rospy.Publisher("imus_limitations", Vector3, queue_size=10)

    os.system("clear")
    selection=0
    #effort_value=0
    while selection==0:
        selection = input(colour("# # # # # # # # # # # # # # # # # # # # # # # #\
 # # # # # # # # # # # # # #",32)+"\n\n\
Select one Option from below:\n\n\
*Reset the Simulation\t\t\t [1]\n\
*Wake up the " + colour("Robonaut",35) + " [" + colour("-R2-",31) + "]" +"\t\t [2]\n\
*Initialization of Robonaut's Joints\t [3]\n\n\
*Set the Power of the Finger's Joint Efforts, to adjust their stiffness\t [4]\n\
*"+ colour("Enable Hand",36)+ " Control\t [5]\n\
*"+ colour("Disable Hand",31)+ " Control\t [6]\n\n\
*Set Arm Pose Limits [ rosrun armlimit_spec.py ] [7]\n\
*"+ colour("Enable Arm ",36)+ "Control\t [8]\n\
*"+ colour("Disable Arm ",31)+ "Control\t [9]\n\n\
*Set Robonaut's Default Position"+colour(" -Waist and Arm Level-",34)+" [10]\n\n\
* Exit [0]\n\n->> ")
        if selection != 0 and selection<=20 and selection>=1: ##ascii char '5' is equal to #53 and '0' is #48..
            if selection==4:
                effort_value= input("\nPlease give me a number, in the range [0 to 10], to specify the stiffness\n of the fingers. ( 10 is the most sensitive,0.1 more stiff )\n\nEnter Value-> ")
                effort_msg.publish(effort_value)
            elif selection==10:
                print("\nPress the keyboard Arrows (left <- and -> right) to set the new Robonaut's waist position and (up and\
 down arrows) to set Robonaut's Arm Start height. (press one another key, to quit)\n")
                arrow_value="noth"
                while arrow_value!="unknown":
                    arrow_value=getArrowForWaistMovement()
                                  
            msg.publish(selection)

        else: break
        os.system("clear")
        print (colour("# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #",32)+"\n\n->\tOption '" + str(selection) + "' just published...\n")
        selection=0


    print("Quiting now..\nThe node can be terminated..")
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
