#!/usr/bin/env python

## Simple myo demo that listens to std_msgs/UInt8 poses published 
## to the 'myo_gest' topic

import rospy
from std_msgs.msg import UInt8, String
#from __future__ import print_function

from collections import Counter
import struct
import sys
import time
from std_msgs.msg import String,Float32, UInt8, Header, MultiArrayLayout, MultiArrayDimension, Float64MultiArray
from geometry_msgs.msg import Quaternion, Vector3
from sensor_msgs.msg import Imu
from ros_myo.msg import MyoArm, EmgArray
import math
import serial
from serial.tools.list_ports import comports


import numpy as np
import os

try:
    from sklearn import neighbors, svm
    HAVE_SK = True
except ImportError:
    HAVE_SK = False

try:
    import pygame
    from pygame.locals import *
    while True:
        os.system("clear")
        opt=raw_input("Do you want a Graphic EMG data display? [yes or no : y or n]\n\n--> ")
        if opt=="yes" or opt=="y":
            HAVE_PYGAME = True
            break
        elif opt=="no" or opt=="n":
            HAVE_PYGAME = False
            break
except ImportError:
    HAVE_PYGAME = False

from common import *
import myo

class EMGHandler(object):
    def __init__(self, m):
        self.recording = -1
        self.m = m
        self.emg = (0,) * 8

    def __call__(self, emg, moving):
        self.emg = emg
        if self.recording >= 0:
            self.m.cls.store_data(self.recording, emg)



if __name__ == '__main__':
    rospy.init_node('Myo_node', anonymous=True)

    handgest = rospy.Publisher('myo_handgest', String, queue_size=10)
    imuPub = rospy.Publisher('myo_imuQuat', Imu, queue_size=10)
    emgPub = rospy.Publisher('myo_emg', EmgArray, queue_size=10)
    armPub = rospy.Publisher('myo_arm', MyoArm, queue_size=10)
    #r = rospy.Rate(0.0001)

    if HAVE_PYGAME:
        pygame.init()
        w, h = 950, 320
        scr = pygame.display.set_mode((w, h))
        font = pygame.font.Font(None, 35)

    m = myo.Myo(myo.NNClassifier(), sys.argv[1] if len(sys.argv) >= 2 else None)
    hnd = EMGHandler(m)
    m.add_emg_handler(hnd)
    last_pose=0
    counter=0
    last_puvblished_pose="rest" # initialaiztion
    handgest.publish(last_puvblished_pose)

    def proc_emg(emg, moving, times=[]):
        ## create an array of ints for emg data
        emgPub.publish(emg)

        ## print framerate of received data
        times.append(time.time())
        if len(times) > 20:
            #print((len(times) - 1) / (times[-1] - times[0]))
            times.pop(0)
    # Package the IMU data into an Imu message
    def proc_imu(quat1, acc, gyro):
        # New info: https://github.com/thalmiclabs/myo-bluetooth/blob/master/myohw.h#L292-L295
        # Scale values for unpacking IMU data
        # define MYOHW_ORIENTATION_SCALE   16384.0f ///< See myohw_imu_data_t::orientation
        # define MYOHW_ACCELEROMETER_SCALE 2048.0f  ///< See myohw_imu_data_t::accelerometer
        # define MYOHW_GYROSCOPE_SCALE     16.0f    ///< See myohw_imu_data_t::gyroscope
        h = Header()
        h.stamp = rospy.Time.now()
        h.frame_id = 'myo'
        # We currently do not know the covariance of the sensors with each other
        cov = [0, 0, 0, 0, 0, 0, 0, 0, 0]
        quat = Quaternion(quat1[0]/16384.0, quat1[1]/16384.0, quat1[2]/16384.0, quat1[3]/16384.0)
        ## Normalize the quaternion and accelerometer values
        quatNorm = math.sqrt(quat.x*quat.x+quat.y*quat.y+quat.z*quat.z+quat.w*quat.w)
        normQuat = Quaternion(quat.x/quatNorm, quat.y/quatNorm, quat.z/quatNorm, quat.w/quatNorm)
        normAcc = Vector3(acc[0]/2048.0, acc[1]/2048.0, acc[2]/2048.0)
        normGyro = Vector3(gyro[0]/16.0, gyro[1]/16.0, gyro[2]/16.0)
        imu = Imu(h, normQuat, cov, normGyro, cov, normAcc, cov)
        imuPub.publish(imu)

    # Package the arm and x-axis direction into an Arm message  
    def proc_arm(arm, xdir):
        #When the arm state changes, publish the new arm and orientation
        calibArm=MyoArm(arm.value, xdir.value)
        armPub.publish(calibArm)


    m.add_emg_handler(proc_emg)
    m.add_imu_handler(proc_imu)
    m.add_arm_handler(proc_arm)
    m.connect()

    try:
        while True:
            m.run()

            # !!!
            r = m.history_cnt.most_common(1)[0][0]
            
            #Determine which is the correct current hand pose, without any noises from the 
            #hand pose switching movement..
            if r==last_pose:
                if r==0:
                    pose="rest"
                    counter=counter+1
                elif r==1:
                    pose="fist"
                    counter=counter+1
                elif r==2:
                    pose="wave in"
                    counter=counter+1
                elif r==3:
                    pose="wave out"
                    counter=counter+1
                elif r==4:
                    pose="fingers spread"
                    counter=counter+1
                elif r==5:
                    pose="thumbs up"
                    counter=counter+1
                elif r==6:
                    pose="grab"
                    counter=counter+1
                elif r==7:
                    pose="[new]"
                    counter=counter+1
                elif r==8:
                    pose="[new]"
                    counter=counter+1
                elif r==9:
                    pose="[new]"
                    counter=counter+1
            else:
                counter=0
            last_pose=r

            if counter==50:
                if last_puvblished_pose!=pose:
                    handgest.publish(pose) #pose publish...
                #else do noth, don't publish the same pose again
                last_puvblished_pose=pose
                counter=0
                            

            if HAVE_PYGAME:
                for ev in pygame.event.get():
                    if ev.type == QUIT or (ev.type == KEYDOWN and ev.unicode == 'q'):
                        raise KeyboardInterrupt()
                    elif ev.type == KEYDOWN:
                        if K_0 <= ev.key <= K_9:
                            hnd.recording = ev.key - K_0
                        elif K_KP0 <= ev.key <= K_KP9:
                            hnd.recording = ev.key - K_Kp0
                        elif ev.unicode == 'r':
                            hnd.cl.read_data()
                    elif ev.type == KEYUP:
                        if K_0 <= ev.key <= K_9 or K_KP0 <= ev.key <= K_KP9:
                            hnd.recording = -1

                scr.fill((0, 0, 0), (0, 0, w, h))

                for i in range(10):
                    x = 0
                    y = 0 + 30 * i

                    if i==0:
                        pose="rest"
                    elif i==1:
                        pose="fist"
                    elif i==2:
                        pose="wave in"
                    elif i==3:
                        pose="wave out"
                    elif i==4:
                        pose="fingers spread"
                    elif i==5:
                        pose="thumbs up" #not implemented yet..
                    elif i==6:
                        pose="[new]" #slot for a new pose..
                    elif i==7:
                        pose="[new]"
                    elif i==8:
                        pose="[new]"
                    elif i==9:
                        pose="[new]"

                    clr = (0,200,0) if i == r else (255,255,255)

                    txt = font.render('%5d' % (m.cls.Y == i).sum(), True, (255,255,255))
                    scr.blit(txt, (x + 20, y))
                    
                    txt = font.render(str(i)+": "+pose, True, clr)
                    scr.blit(txt, (x + 100, y))


                    scr.fill((0,0,0), (x+300, y + txt.get_height() / 2 - 10, len(m.history) * 20, 20))
                    scr.fill(clr, (x+300, y + txt.get_height() / 2 - 10, m.history_cnt[i] * 20, 20))

                if HAVE_SK and m.cls.nn is not None:
                    dists, inds = m.cls.nn.kneighbors(hnd.emg)
                    for i, (d, ind) in enumerate(zip(dists[0], inds[0])):
                        y = m.cls.Y[myo.SUBSAMPLE*ind]
                        #if i%14==0:
                        #    handgest.publish(y)
                        text(scr, font, '%d %6d' % (y, d), (840, 20 * i))

                pygame.display.flip()

                            
    except KeyboardInterrupt or (rospy.ROSInterruptException, serial.serialutil.SerialException) as e:
        pass
    finally:
        m.disconnect()
        print()

    if HAVE_PYGAME:
        pygame.quit()