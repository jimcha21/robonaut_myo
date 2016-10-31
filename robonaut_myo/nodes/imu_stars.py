#!/usr/bin/env python

## Simple myo demo that listens to sensor_msgs.msg/Imu raw data in Quaternion form
## and publishes them in euler angles. (in myo_xyz_axes_raw topic) + also displays
## them in a separate topic in graphic mode with *.

import rospy
from std_msgs.msg import UInt8, String
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Imu
from ros_myo.msg import MyoArm
from Quaternion import *
import math

if __name__ == '__main__':

    rospy.init_node('myo_QuaternionToEuler_transf', anonymous=True)
    global c
    c=0
    imu_xyz_disp = rospy.Publisher("myo_displayimuStars", String, queue_size=10)
    imu_xyz_data = rospy.Publisher("myo_imuEulerAngles_raw", Vector3, queue_size=10)

    #Quaternion to euler transformation
    def quat2euler(quat):
        global c
        q0=quat.orientation.w
        q1=quat.orientation.x
        q2=quat.orientation.y
        q3=quat.orientation.z

        # Calculate Euler angles (roll, pitch, and yaw) from the unit quaternion.
        roll = math.atan2(2.0 * (q0 * q1 + q2 * q3), 1.0 - 2.0 * (q1 * q1 + q2 * q2));
        pitch = math.asin(max(-1.0, min(1.0, 2.0 * (q0 * q2 - q3 * q1))));
        yaw = atan2(2.0 * (q0 * q3 + q1 * q2),1.0 - 2.0 * (q2 * q2 + q3 * q3));

        # Convert the floating point angles in radians to a scale from 0 to 18.
        M_PI=3.1456
        roll_w =(roll + M_PI)/(M_PI * 2.0) * 18
        pitch_w = (pitch + M_PI/2.0)/M_PI * 18
        yaw_w = (yaw + M_PI)/(M_PI * 2.0) * 18

        #poka= Quat(q)
        #turtlesimPub.publish("roll -> " + str(poka._get_ra()) + "pitch -> " + str(poka._get_dec()) + "yaw -> " + str(poka._get_roll()))
        imu_xyz_disp.publish("roll -> [" + "*"*(int(math.floor(roll_w))) + " "*(18-int(math.floor(roll_w))) + "] pitch -> [" + "*"*(int(math.floor(pitch_w))) + " "*(18-int(math.floor(pitch_w))) + "] yaw -> [" + "*"*(int(math.floor(yaw_w))) + " "*(18-int(math.floor(yaw_w))) +"] ")
        if c>5:
            #imu_xyz_disp.publish("roll -> [" + "*"*(int(math.floor(roll_w))) + " "*(18-int(math.floor(roll_w))) + "] pitch -> [" + "*"*(int(math.floor(pitch_w))) + " "*(18-int(math.floor(pitch_w))) + "] yaw -> [" + "*"*(int(math.floor(yaw_w))) + " "*(18-int(math.floor(yaw_w))) +"] ")
            imu_xyz_data.publish(roll_w,pitch_w,yaw_w) #in scale
            c=0
        c=c+1

    #Subscribe to the myo's raw imu data.
    rospy.Subscriber("myo_imuQuat", Imu,quat2euler)
    #rospy.Subscriber("myo_arm", MyoArm,getarm)
    rospy.loginfo('Please sync the Myo')

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
