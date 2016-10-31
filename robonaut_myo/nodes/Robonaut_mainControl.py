#!/usr/bin/env python

import sys
import rospy
from beginner_tutorials.srv import *
from gazebo_msgs.srv import *
from std_srvs.srv import Empty
import time
from std_msgs.msg import UInt8, Float32,String
from sensor_msgs.msg import Imu
from ros_myo.msg import MyoArm
from collections import namedtuple
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float32MultiArray
from gazebo_msgs.msg import ODEJointProperties

global handTopic_subscription
global armTopic_subscription

def set_globalimuL(imu):
    global myo_face
    global roll_high
    global roll_low
    global pitch_high
    global pitch_low
    global yaw_high
    global yaw_low
    global yaw_init
    myo_face=imu.data[0]
    yaw_init=imu.data[1]
    roll_high=imu.data[2]
    roll_low=imu.data[3]
    pitch_high=imu.data[4]
    pitch_low=imu.data[5]
    yaw_high=imu.data[6]
    yaw_low=imu.data[7]
    msg.publish("IMU data just applied..")

def set_globaleffort(effort):
    global effort_param
    effort_param=effort.data

def set_R2Position_withArrows(arrow):
    global arrow_value
    global last_waistpose
    global last_armpose
    arrow_value=arrow.data
    msg.publish("New Arrow data just received! Rotating -" + colour(arrow.data,34)+"-")
    #msg.publish(str(last_waistpose))
    #msg.publish(str(last_armpose))
    if arrow.data=="left" and last_waistpose<2.8:
        msg.publish("Rotating Robonaut Left")
        last_waistpose=last_waistpose+0.1
    elif arrow.data=="right" and last_waistpose>-2.8:
        msg.publish("Rotating Robonaut Right")
        last_waistpose=last_waistpose-0.1
    elif arrow.data=="up" and last_armpose>-1.5:
        msg.publish("Lifting the Robonaut's Arm")
        last_armpose=last_armpose-0.1
    elif arrow.data=="down" and last_armpose<1.5:
        msg.publish("Lowering the Robonaut's Arm")
        last_armpose=last_armpose+0.1
    # if unknown, do nothing

    rospy.wait_for_service('/gazebo/set_joint_properties')
    try:
         # for the waist position
        serv_call=rospy.ServiceProxy('/gazebo/set_joint_properties', gazebo_msgs.srv.SetJointProperties)
        obj = ODEJointProperties(damping=[], hiStop=[last_waistpose+0.1], loStop=[last_waistpose-0.1], erp=[], cfm=[], stop_erp=[], stop_cfm=[], fudge_factor=[], fmax=[], vel=[])
        response = serv_call("/r2/waist/joint0",obj)
        msg.publish(" **Properties set , for joint -> " + "/r2/waist/joint0"+ " [\033[32m" + "ok" + "\033[0m]")
        # for the arm height
        obj.hiStop=[last_armpose+0.05]
        obj.loStop=[last_armpose-0.05]
        response = serv_call("/r2/left_arm/joint0",obj)
        msg.publish(" **Properties set , for joint -> " + "/r2/left_arm/joint0"+ " [\033[32m" + "ok" + "\033[0m]")
    except rospy.ServiceException, e:
        msg.publish("Service call failed: %s"%e)
       
def unsubscribe_handgest(message):
    #do processing here
    #handTopic_subscription.unregister()
    global handTopic_subscription
    handTopic_subscription.unregister()
    handTopic_subscription=None

    #msg.publish("Rospy just unsubscribed from /myo_handgest ...")

def tryprint_effort():
    global effort_param
    try:
        msg.publish("Received new joint effort : " + colour(str(effort_param),33))
        return True
    except NameError:
        msg.publish(colour("No Effort Parameter was found!!! ",31)+ "Please set it, by choosing the '4' on the menu. ")
        return False

def tryprint_IMULimitations():
    global myo_face
    global roll_high
    global roll_low
    global pitch_high
    global pitch_low
    global yaw_high
    global yaw_low
    global yaw_init
    try:
        msg.publish("The using IMU Limitations are : ")
        msg.publish("Myo Face : " + str(myo_face))
        msg.publish("The Roll High Limitation is ' " + str(roll_high) + " ' and the low ' " + str(roll_low) + " '.")
        msg.publish("The Pitch High Limitation is ' " + str(pitch_high) + " ' and the low ' " + str(pitch_low) + " '.")
        msg.publish("The Yaw High Limitation is ' " + str(yaw_high) + " ' and the low ' " + str(yaw_low) + " '.")
        return True
    except NameError:
        msg.publish(colour("No IMU Limitations found!!! ",31)+ "Please ROSRun the arm_limits_setup.py to set them up. ")
        return False

def colour(script,clr):
    #30-black
    #31-red
    #32-green
    #33-yellow
    #34-blue
    #35-purple
    #36-blue
    #37-default white
    return "\033["+str(clr)+"m" + script + "\033[0m"
# # # # # # # # # # # # # # # #

def callback(d):

    global handTopic_subscription
    global armTopic_subscription
    global effort_param
    global arrow_value
    global last_waistpose
    global last_armpose


    response="None"
    msg.publish("------------------------------------------------------------")
    
    if d.data==1: # Reseting the Gazebo World..
        
        #waiting for roslaunch
        msg.publish("Waiting for roslaunch")
        rospy.wait_for_service('/gazebo/reset_simulation')
        try:
            serv_call=rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
            response=serv_call()
            last_waistpose=0
        except rospy.ServiceException, e:
            msg.publish("Service call failed: %s"%e)
            response=False

    elif d.data==2: ##Waking up the Robonaut..
        msg.publish("Waking up the Robonaut...")
        #waiting for rosslaunch
        rospy.wait_for_service('/gazebo/apply_joint_effort')
        rospy.wait_for_service('/gazebo/set_joint_properties')
        try:
            head_raise=rospy.ServiceProxy('/gazebo/apply_joint_effort', gazebo_msgs.srv.ApplyJointEffort)
            head_pose=rospy.ServiceProxy('/gazebo/set_joint_properties', gazebo_msgs.srv.SetJointProperties)
            effort=8 # N-m

            j0="/r2/neck/joint0"
            j1="/r2/neck/joint1"
            j2="/r2/neck/joint2"

            MyStruct = namedtuple("MyStruct", "secs nsecs")
            m = MyStruct(secs = 0, nsecs = 0)
            z = MyStruct(secs = 2, nsecs = 0)
            response = head_raise(j0,effort,m,z)
            rospy.sleep(1.)
            z = MyStruct(secs = 2, nsecs = 0)
            response = head_raise(j2,effort,m,z)

            msg.publish("Head raised [\033[32m" + "ok" + "\033[0m] moving on Pose limits set up...")
            rospy.sleep(2.)
            # # # set pose limits
            obj = ODEJointProperties(damping=[], hiStop=[0.1], loStop=[-0.2], erp=[], cfm=[], stop_erp=[], stop_cfm=[], fudge_factor=[], fmax=[], vel=[])
            response = head_pose(j0,obj)
            msg.publish(" **Properties set , for joint -> " + j0 + " [\033[32m" + "ok" + "\033[0m]")
            rospy.sleep(0.2)
            obj = ODEJointProperties(damping=[], hiStop=[0.1], loStop=[-0.1], erp=[], cfm=[], stop_erp=[], stop_cfm=[], fudge_factor=[], fmax=[], vel=[])
            response = head_pose(j1,obj)
            msg.publish(" **Properties set , for joint -> " + j1 + " [\033[32m" + "ok" + "\033[0m]")
            rospy.sleep(0.2)
            obj = ODEJointProperties(damping=[], hiStop=[0.2], loStop=[0.15], erp=[], cfm=[], stop_erp=[], stop_cfm=[], fudge_factor=[], fmax=[], vel=[])
            response = head_pose(j2,obj)
            msg.publish(" **Properties set , for joint -> " + j2 + " [\033[32m" + "ok" + "\033[0m]")
            rospy.sleep(0.2)

        except rospy.ServiceException, e:
            msg.publish("Service call failed: %s"%e)
            response=False

    elif d.data==3: ##seting gazebo hand pose limitations, due to some design mistakes of Robonaut
 
        #waiting for rosslaunch
        rospy.wait_for_service('/gazebo/set_joint_properties')
        ##2 loops for each hand, at a time...
        for i in range(0, 2):
            if i==1:
                msg.publish("Setting Left Hand pose Limits...")
                arm_string="/r2/left_arm/"
            else:
                msg.publish("Setting Right Hand pose Limits...")
                arm_string="/r2/right_arm/"

            #same calls for both hands
            ind="index/"
            mid="middle/"
            rin="ring/"
            lit="little/"
            thu="thumb/"
            hand="hand/"
            j0="joint0"
            j1="joint1"
            j2="joint2"
            j3="joint3"
            j4="joint4"
            j5="joint5"
            j6="joint6"
            

            try:
                serv_call=rospy.ServiceProxy('/gazebo/set_joint_properties', gazebo_msgs.srv.SetJointProperties)
                delay=0.2

                # # # # joint 0

                obj = ODEJointProperties(damping=[], hiStop=[0.5], loStop=[], erp=[], cfm=[], stop_erp=[], stop_cfm=[], fudge_factor=[], fmax=[], vel=[])
                #response = serv_call(hand_string+ind+j0,obj)
                msg.publish(" **Properties set , for joint -> " + arm_string+hand+ind+j0+ " [\033[32m" + "ok" + "\033[0m]")
                rospy.sleep(delay)
                obj.hiStop=[0.2]
                #response = serv_call(hand_string+mid+j0,obj)
                msg.publish(" **Properties set , for joint -> " + arm_string+hand+mid+j0+ " [\033[32m" + "ok" + "\033[0m]")
                rospy.sleep(delay)
                obj.hiStop=[1.7]
                response = serv_call(arm_string+hand+rin+j0,obj)
                msg.publish(" **Properties set , for joint -> " + arm_string+hand+rin+j0+ " [\033[32m" + "ok" + "\033[0m]")
                rospy.sleep(delay)
                response = serv_call(arm_string+hand+lit+j0,obj)
                msg.publish(" **Properties set , for joint -> " + arm_string+hand+lit+j0+ " [\033[32m" + "ok" + "\033[0m]")
                rospy.sleep(delay)
                obj.hiStop=[0.6]
                response = serv_call(arm_string+hand+thu+j0,obj)
                msg.publish(" **Properties set , for joint -> " + arm_string+hand+thu+j0+ " [\033[32m" + "ok" + "\033[0m]")
                rospy.sleep(delay)

                # # # # joint 1
                
                obj.hiStop=[1.7]
                response = serv_call(arm_string+hand+ind+j1,obj)
                msg.publish(" **Properties set , for joint -> " + arm_string+hand+ind+j1 + " [\033[32m" + "ok" + "\033[0m]")
                rospy.sleep(delay)
                response = serv_call(arm_string+hand+mid+j1,obj)
                msg.publish(" **Properties set , for joint -> " + arm_string+hand+mid+j1 + " [\033[32m" + "ok" + "\033[0m]")
                rospy.sleep(delay)
                obj.hiStop=[2]
                response = serv_call(arm_string+hand+rin+j1,obj)
                msg.publish(" **Properties set , for joint -> " + arm_string+hand+rin+j1 + " [\033[32m" + "ok" + "\033[0m]")
                rospy.sleep(delay)
                response = serv_call(arm_string+hand+lit+j1,obj)
                msg.publish(" **Properties set , for joint -> " + arm_string+hand+lit+j1 + " [\033[32m" + "ok" + "\033[0m]")
                rospy.sleep(delay)
                obj.hiStop=[1]
                response = serv_call(arm_string+hand+thu+j1,obj)
                msg.publish(" **Properties set , for joint -> " + arm_string+hand+thu+j1 + " [\033[32m" + "ok" + "\033[0m]")
                rospy.sleep(delay)

                # # # # joint 2
                
                obj.hiStop=[1.6]
                response = serv_call(arm_string+hand+rin+j2,obj)
                msg.publish(" **Properties set , for joint -> " + arm_string+hand+rin+j2 + " [\033[32m" + "ok" + "\033[0m]")
                rospy.sleep(delay)
                response = serv_call(arm_string+hand+lit+j2,obj)
                msg.publish(" **Properties set , for joint -> " + arm_string+hand+lit+j2 + " [\033[32m" + "ok" + "\033[0m]")
                rospy.sleep(delay)
                response=True

                ### for the hand produce
                obj.hiStop=[0.05]
                obj.loStop=[-0.05]
                response = serv_call(arm_string+j6,obj)
                msg.publish(" **Properties set , for joint -> " + arm_string+j6 + " [\033[32m" + "ok" + "\033[0m]")
                rospy.sleep(delay)

                ### for the arm stand * * *
                last_armpose=0
                obj.hiStop=[0.05]
                obj.loStop=[-0.05]
                response = serv_call(arm_string+j0,obj)
                msg.publish(" **Properties set , for joint -> " + arm_string+j0 + " [\033[32m" + "ok" + "\033[0m]")
                rospy.sleep(delay)

                ## for the waist
                last_waistpose=0
                obj.hiStop=[0.1]
                obj.loStop=[-0.1]
                response = serv_call("/r2/waist/"+j0,obj)
                msg.publish(" **Properties set , for joint -> " + "/r2/waist/"+j0 + " [\033[32m" + "ok" + "\033[0m]")
                rospy.sleep(delay)

            except rospy.ServiceException, e:
                msg.publish("Service call failed: %s"%e)
                response=False

    elif d.data==4: #Checking for Hand Joint's Effort parameter availabillity..

        rospy.sleep(0.1) # waiting for set_globaleffort to set the global effort parameter, and
        response=tryprint_effort()

    elif d.data==5:

        #waiting for rosslaunch
        rospy.wait_for_service('/gazebo/apply_joint_effort')
        response=tryprint_effort()
        #if by mistake two subcription are going to open, unsubscribe from the older one..
        try:
            handTopic_subscription.unregister()
        except NameError:
            msg.publish("Unsubscribed from the older one.")

        if response:
            handTopic_subscription=rospy.Subscriber("/myo_handgest",String,Robonaut_HandPose)
            #rospy.sleep(10) ##Enables Hand CoNTROL

        
    elif d.data==6: ##disable subscribtion

        msg.publish("Unsubscribing from the Hand topic. " +colour("completed",32))
        handTopic_subscription.unregister()
        response=True

    elif d.data==7: ## checks if imu limitations were given...

        response=tryprint_IMULimitations()
    
    elif d.data==8: ##IMU controlling - Enables Arm control

        response=tryprint_IMULimitations()
        #if by mistake two subcription are going to open, unsubscribe from the older one..
        try:
            armTopic_subscription.unregister()
        except NameError:
            msg.publish("Unsubscribed from the older one.")

        if response:
            armTopic_subscription=rospy.Subscriber("/myo_imuEulerAngles_raw",Vector3,Robonaut_ArmPose)
        #armTopic_subscription=rospy.Subscriber("/myo_imuEulerAngles_raw",Vector3,joint3_witheffort)

    elif d.data==9: ##disable subscribtion

        msg.publish("Unsubscribing from the Arm topic. [ \033[32m" + "completed" + "\033[0m ] ")
        armTopic_subscription.unregister()
        clear_joint_forces()

    if response!="None" and response!=False:
        info_message=colour(" Success!",32)
    else:
        info_message=colour(" Nothing happened...",31)

    msg.publish("Service call response: "+ str(response) + info_message)

# # # # # # # # # # # # # # # #
def Robonaut_ArmPose(vector):
    global myo_face
    global yaw_init
    global roll_high
    global roll_low
    global pitch_high
    global pitch_low
    global yaw_high
    global yaw_low

    
    get_jprop=rospy.ServiceProxy('/gazebo/get_joint_properties', gazebo_msgs.srv.GetJointProperties)
    current_armProp=get_jprop("/r2/left_arm/joint3")
    serv_call=rospy.ServiceProxy('/gazebo/set_joint_properties', gazebo_msgs.srv.SetJointProperties)
    
    a=-1
    b=-3

    lol=yaw_high-yaw_low

    m=vector.x+yaw_init ## for the correct initialization in range 0 to 17.

    if m>=18:
        m=m-18

    if m>yaw_low and m<yaw_high:
        num=a+((b-a)/lol)*(m-yaw_low)
        #msg.publish("yaw -> "+str(num))
        obj = ODEJointProperties(damping=[], hiStop=[num], loStop=[num-0.1], erp=[], cfm=[], stop_erp=[], stop_cfm=[], fudge_factor=[], fmax=[], vel=[])
        response = serv_call("/r2/left_arm/joint2",obj)
    #else:
    #    msg.publish("off limits the yawin")

    if myo_face==0: # roll parameters depend on myo position...
        roll_par1=3
        roll_par2=2.9
    else:
        roll_par1=0
        roll_par2=0.1
    
    #print(str(current_armProp.position[0]))

    lol=roll_high-roll_low

    if vector.z>roll_low and vector.z<roll_high:
        num=(3/lol)*(vector.z-roll_low)
        #msg.publish("roll high - > "+str(abs(num-roll_par1)))
        #msg.publish("roll low - > "+str(abs(num-roll_par2)))

        obj = ODEJointProperties(damping=[], hiStop=[abs(num-roll_par1)], loStop=[abs(num-roll_par2)], erp=[], cfm=[], stop_erp=[], stop_cfm=[], fudge_factor=[], fmax=[], vel=[])
        response = serv_call("/r2/left_arm/joint4",obj)
    #else:
    #    msg.publish("off limit the rollin ")

  
    if myo_face==1:
        pitch_par1=2.6
        pitch_par2=2.7
    else:
        pitch_par1=0.1
        pitch_par2=0
    
    a=0
    b=-2.8

    lol=pitch_high-pitch_low

    if vector.y>pitch_low and vector.y<pitch_high:
        num=a+((b-a)/lol)*(vector.y-pitch_low)
        #msg.publish("pitch high - >" +str(-abs(abs(num)-pitch_par1)))
        #msg.publish("pitch low - >" +str(-abs(abs(num)-pitch_par2)))
        obj = ODEJointProperties(damping=[], hiStop=[-abs(abs(num)-pitch_par1)], loStop=[-abs(abs(num)-pitch_par2)], erp=[], cfm=[], stop_erp=[], stop_cfm=[], fudge_factor=[], fmax=[], vel=[])
        response = serv_call("/r2/left_arm/joint3",obj)
    #else:
    #    msg.publish("off limit the pitchin ")

def joint3_witheffort(vector):  # Controlling arm Up-Down Possition with power efforts.. dokimastiko
    global trufal
    global elite
    get_jprop=rospy.ServiceProxy('/gazebo/get_joint_properties', gazebo_msgs.srv.GetJointProperties)
    current_armProp=get_jprop("/r2/left_arm/joint3")
    
    apply_force=rospy.ServiceProxy('/gazebo/apply_joint_effort', gazebo_msgs.srv.ApplyJointEffort)
    serv_call=rospy.ServiceProxy('/gazebo/set_joint_properties', gazebo_msgs.srv.SetJointProperties)
    
    kat=-2.7+vector.y*(2.7/17)-0.7
    msg.publish("Tha paei gia to " + str(kat))
    
    MyStruct = namedtuple("MyStruct", "secs nsecs")
    m = MyStruct(secs = 0, nsecs = 0)
    z = MyStruct(secs = 1, nsecs = 0)

    if current_armProp.position[0]>kat:
        trufal=True
        msg.publish("paei gia true")
    else:
        trufal=False
        msg.publish("paei gia false")

    effort=10

    if kat==elite or elite==0:

        if trufal:
            obj = ODEJointProperties(damping=[], hiStop=[], loStop=[kat-0.001], erp=[], cfm=[], stop_erp=[], stop_cfm=[], fudge_factor=[], fmax=[], vel=[])
            response = serv_call("/r2/left_arm/joint3",obj)
            #rospy.sleep(delay)
            msg.publish("ISA KATW ORIO ")
            effort=-effort    
        else:
            obj = ODEJointProperties(damping=[], hiStop=[kat+0.001], loStop=[], erp=[], cfm=[], stop_erp=[], stop_cfm=[], fudge_factor=[], fmax=[], vel=[])
            response = serv_call("/r2/left_arm/joint3",obj)
            #rospy.sleep(delay)
            msg.publish("ISA ANW ORIO ")
             

    elif kat>elite:
        if trufal:
            obj = ODEJointProperties(damping=[], hiStop=[], loStop=[kat-0.001], erp=[], cfm=[], stop_erp=[], stop_cfm=[], fudge_factor=[], fmax=[], vel=[])
            response = serv_call("/r2/left_arm/joint3",obj)
            #rospy.sleep(delay)
            msg.publish("1 KATW ORIO ")
            effort=-effort 
        else:
            obj = ODEJointProperties(damping=[], hiStop=[elite+0.001], loStop=[], erp=[], cfm=[], stop_erp=[], stop_cfm=[], fudge_factor=[], fmax=[], vel=[])
            response = serv_call("/r2/left_arm/joint3",obj)
            #rospy.sleep(delay)
            msg.publish("1 ANW ORIO ")
            
    else:
        if trufal==False:
            obj = ODEJointProperties(damping=[], hiStop=[kat+0.001], loStop=[], erp=[], cfm=[], stop_erp=[], stop_cfm=[], fudge_factor=[], fmax=[], vel=[])
            response = serv_call("/r2/left_arm/joint3",obj)
            #rospy.sleep(delay)
            msg.publish("2 ANW ORIO ")
            
        else:
            obj = ODEJointProperties(damping=[], hiStop=[], loStop=[elite-0.001], erp=[], cfm=[], stop_erp=[], stop_cfm=[], fudge_factor=[], fmax=[], vel=[])
            response = serv_call("/r2/left_arm/joint3",obj)
            #rospy.sleep(delay)
            msg.publish("2 KATW ORIO ")
            effort=-effort  
    
    if current_armProp.position[0]>-0.4:
        obj = ODEJointProperties(damping=[], hiStop=[-0.2], loStop=[], erp=[], cfm=[], stop_erp=[], stop_cfm=[], fudge_factor=[], fmax=[], vel=[])
        response = serv_call("/r2/left_arm/joint3",obj)
        msg.publish("edw mpike se mia akraia periptwsh katw")
    elif current_armProp.position[0]<-2.5:
        obj = ODEJointProperties(damping=[], hiStop=[], loStop=[-2.6], erp=[], cfm=[], stop_erp=[], stop_cfm=[], fudge_factor=[], fmax=[], vel=[])
        response = serv_call("/r2/left_arm/joint3",obj)
        msg.publish("edw mpike se mia akraia periptwsh anw")

    elite=kat
    #msg.publish("effort " + str(effort) + " KAI THESI " + str(current_armProp.position[0]))
    response = apply_force('/r2/left_arm/joint3',effort,m,z)

def clear_joint_forces():
    clear=rospy.ServiceProxy('/gazebo/clear_joint_forces', gazebo_msgs.srv.JointRequest)
    
    #same joints at both hands
    ind="index/"
    mid="middle/"
    rin="ring/"
    lit="little/"
    thu="thumb/"
    hand="hand/"
    j0="joint0"
    j1="joint1"
    j2="joint2"
    j3="joint3"
    j4="joint4"
    j5="joint5"
    j6="joint6"

    try:
        for i in range(0, 2): #mirrored hand movement
            if i==0:
                arm_string="/r2/right_arm/"
            else:
                arm_string="/r2/left_arm/"

            response = clear(arm_string+hand+ind+j0)
            response = clear(arm_string+hand+ind+j1)
            response = clear(arm_string+hand+ind+j2)
            response = clear(arm_string+hand+ind+j3)
            response = clear(arm_string+hand+mid+j0)            
            response = clear(arm_string+hand+mid+j1)
            response = clear(arm_string+hand+mid+j2)
            response = clear(arm_string+hand+mid+j3)                       
            response = clear(arm_string+hand+rin+j0)
            response = clear(arm_string+hand+rin+j1)
            response = clear(arm_string+hand+rin+j2)
            response = clear(arm_string+hand+lit+j0)
            response = clear(arm_string+hand+lit+j1)
            response = clear(arm_string+hand+lit+j2)
            response = clear(arm_string+hand+thu+j0)
            response = clear(arm_string+hand+thu+j1)
            response = clear(arm_string+hand+thu+j2)
            response = clear(arm_string+hand+thu+j3)
            response = clear(arm_string+j5)

        #msg.publish("clear done")
        return True    
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        return False

def Robonaut_HandPose(pose):
    
    a = rospy.get_rostime()
    global last_pose

    #msg.publish(str(effort_param))
    msg.publish("Hand Pose [ \033[35m" + pose.data + "\033[0m ] received...")

    apply_force=rospy.ServiceProxy('/gazebo/apply_joint_effort', gazebo_msgs.srv.ApplyJointEffort)
    clear=rospy.ServiceProxy('/gazebo/clear_joint_forces', gazebo_msgs.srv.JointRequest)

    #same joints at both hands
    ind="index/"
    mid="middle/"
    rin="ring/"
    lit="little/"
    thu="thumb/"
    hand="hand/"
    j0="joint0"
    j1="joint1"
    j2="joint2"
    j3="joint3"
    j4="joint4"
    j5="joint5"
    j6="joint6"

    if pose.data=="rest": # # relaxed hand
        
        clear_joint_forces()

    elif pose.data=="fist": # # fist

        try:
            effort=1*effort_param # N-m
            clear_joint_forces()
            MyStruct = namedtuple("MyStruct", "secs nsecs")
            m = MyStruct(secs = 0, nsecs = 0)
            z = MyStruct(secs = 30, nsecs = 0)
            for i in range(0, 2): #mirrored hand movement
                if i==0:
                    hand_string="/r2/right_arm/hand/"
                else:
                    hand_string="/r2/left_arm/hand/"

                response = apply_force(hand_string+ind+j1,effort,m,z)
                response = apply_force(hand_string+mid+j1,effort,m,z)
                response = apply_force(hand_string+rin+j0,effort,m,z)
                response = apply_force(hand_string+lit+j0,effort,m,z)
                response = apply_force(hand_string+thu+j1,effort,m,z) # !!!!

                if effort<3.5:
                    get_jprop=rospy.ServiceProxy('/gazebo/get_joint_properties', gazebo_msgs.srv.GetJointProperties)
                    
                    index_prop=get_jprop(hand_string+ind+j1)
                    middle_prop=get_jprop(hand_string+mid+j1)
                    ring_prop=get_jprop(hand_string+rin+j0)
                    little_prop=get_jprop(hand_string+lit+j0)
                    #if index_prop.position[0]>1.30:
                    response = apply_force(hand_string+ind+j2,effort,m,z)
                    #if middle_prop.position[0]>1.30:
                    response = apply_force(hand_string+mid+j2,effort,m,z)
                    #if ring_prop.position[0]>1.30:
                    response = apply_force(hand_string+rin+j1,effort,m,z)
                    #if little_prop.position[0]>1.15:
                    response = apply_force(hand_string+lit+j1,effort,m,z)

            msg.publish("Movement [ " + colour(pose.data,32) + " ] completed..")
            #msg.publish("With effort "+ str(effort))    
        except rospy.ServiceException, e:
            msg.publish("Service call failed: %s"%e)

    elif pose.data=="wave in" or pose.data=="wave out": # # wave in - out

        try:
            if pose.data=="wave in":
                effort=-5-effort_param # N-m
            else: effort=5+effort_param 
            clear_joint_forces()
            MyStruct = namedtuple("MyStruct", "secs nsecs")
            m = MyStruct(secs = 0, nsecs = 0)
            z = MyStruct(secs = 30, nsecs = 0)
            for i in range(0, 2): #mirrored hand movement
                if i==0:
                    arm_string="/r2/right_arm/"
                else:
                    arm_string="/r2/left_arm/"

                response = apply_force(arm_string+j5,effort,m,z)

            msg.publish("Movement [ " + colour(pose.data,32) + " ] completed..")    
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
    
    elif pose.data=="fingers spread": # # fingers spread

        try:
            #negative_effort=-1 
            effort=0.1+effort_param
            clear_joint_forces()
            MyStruct = namedtuple("MyStruct", "secs nsecs")
            m = MyStruct(secs = 0, nsecs = 0)
            z = MyStruct(secs = 30, nsecs = 0)
            for i in range(0, 2): #mirrored hand movement
                if i==0:
                    hand_string="/r2/right_arm/hand/"
                    response = apply_force(hand_string+ind+j0,-effort,m,z)
                    response = apply_force(hand_string+mid+j0,-effort,m,z)
                else:
                    hand_string="/r2/left_arm/hand/"
                    response = apply_force(hand_string+ind+j0,effort,m,z)
                    response = apply_force(hand_string+mid+j0,effort,m,z)

                response = apply_force(hand_string+ind+j1,-effort,m,z)
                response = apply_force(hand_string+ind+j2,-effort,m,z)
                response = apply_force(hand_string+ind+j3,-effort,m,z)
                response = apply_force(hand_string+mid+j1,-effort,m,z)
                response = apply_force(hand_string+mid+j2,-effort,m,z)
                response = apply_force(hand_string+mid+j3,-effort,m,z)
                response = apply_force(hand_string+rin+j0,-effort,m,z)
                response = apply_force(hand_string+rin+j1,-effort,m,z)
                response = apply_force(hand_string+rin+j2,-effort,m,z)
                response = apply_force(hand_string+lit+j0,-effort,m,z)
                response = apply_force(hand_string+lit+j1,-effort,m,z)
                response = apply_force(hand_string+lit+j2,-effort,m,z)
                response = apply_force(hand_string+thu+j1,-effort,m,z) ## !!!
                response = apply_force(hand_string+thu+j2,-effort,m,z)

            msg.publish("Movement [ " + colour(pose.data,32) + " ] completed..")
            #msg.publish("With effort "+ str(effort))    
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
      
    ## elif or a new hand pose , that can be programmed below... 

    else:
        msg.publish("[ " + colour("Unknown Pose",31) + " ]")
    msg.publish(colour(last_pose,34))
    last_pose=pose.data

if __name__ == "__main__":

    rospy.init_node('Robonaut_mainCont_node', anonymous=True)
    global last_pose
    global last_waistpose
    global last_armpose
    global elite
    global trufal
    elite=0
    trufal=False
    last_waistpose=0
    last_armpose=0
    last_pose="rest"
    msg = rospy.Publisher("movement_info", String, queue_size=10) ##the messages topic publication..
    
    rospy.Subscriber("/user_effort_specification",Float32, set_globaleffort)
    rospy.Subscriber("/user_waist_pose_specification",String, set_R2Position_withArrows)
    rospy.Subscriber("arm_limitations",Float32MultiArray, set_globalimuL)
    rospy.Subscriber("/user_messages",UInt8, callback)
    rospy.loginfo('Please sync the Myo')

    rospy.spin()