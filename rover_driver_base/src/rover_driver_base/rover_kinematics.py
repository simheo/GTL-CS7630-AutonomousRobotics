#!/usr/bin/env python
import roslib; roslib.load_manifest('rover_driver_base')
import rospy
from geometry_msgs.msg import Twist
import numpy
from numpy import fmod
from numpy.linalg import pinv
from math import atan2, hypot, pi, cos, sin

prefix=["FL","FR","CL","CR","RL","RR"]

class RoverMotors:
    def __init__(self):
        self.steering={}
        self.drive={}
        for k in prefix:
            self.steering[k]=0.0
            self.drive[k]=0.0
    def copy(self,value):
        for k in prefix:
            self.steering[k]=value.steering[k]
            self.drive[k]=value.drive[k]

class DriveConfiguration:
    def __init__(self,radius,x,y,z):
        self.x = x
        self.y = y
        self.z = z
        self.radius = radius


class RoverKinematics:
    def __init__(self):
        self.X = numpy.asmatrix(numpy.zeros((3,1)))
        self.motor_state = RoverMotors()
        self.first_run = True

    def twist_to_motors(self, twist, drive_cfg, skidsteer=False):
        motors = RoverMotors()
        if skidsteer:
            for k in drive_cfg.keys():
                # Insert here the steering and velocity of 
                # each wheel in skid-steer mode
                vx=twist.linear.x-twist.angular.z*drive_cfg[k].y
                vy=twist.linear.y+twist.angular.z*drive_cfg[k].x
                motors.steering[k] = 0
                motors.drive[k] = hypot(vy,vx)/drive_cfg[k].radius
        else:
            for k in drive_cfg.keys():
                # Insert here the steering and velocity of 
                # each wheel in rolling-without-slipping mode
                vx=twist.linear.x-twist.angular.z*drive_cfg[k].y
                vy=twist.linear.y+twist.angular.z*drive_cfg[k].x
                motors.steering[k] = atan2(vy,vx)
                motors.drive[k] = hypot(vy,vx)/drive_cfg[k].radius
        return motors

    def integrate_odometry(self, motor_state, drive_cfg):
        i,j=0,0
        
        # The first time, we need to initialise the state
        if self.first_run:
            self.motor_state.copy(motor_state)
            self.first_run = False
            return self.X
        # Insert here your odometry code
        A=numpy.zeros((12,3)) #2 rows per wheel
        B=numpy.zeros((12,1))
                
        for k in drive_cfg.keys():
            A[i][0]=1
            A[i][2]=-drive_cfg[k].y
            i+=1
            A[i][1]=1
            A[i][2]=drive_cfg[k].x
            i+=1
        
        for k in drive_cfg.keys():
            dphi=numpy.fmod(motor_state.drive[k]-self.motor_state.drive[k]+pi,2*pi)
            if dphi<0:
                dphi+=2*pi
            dphi=dphi-pi
            
            B[j]=dphi*drive_cfg[k].radius*cos(motor_state.steering[k])
            j+=1
            B[j]=dphi*drive_cfg[k].radius*sin(motor_state.steering[k])
            j+=1
            
        X=numpy.dot(pinv(A),B)
            
        self.X[0,0] +=X[0,0]*cos(self.X[2,0])-X[1,0]*sin(self.X[2,0]) 
        self.X[1,0] +=X[0,0]*sin(self.X[2,0])+X[1,0]*cos(self.X[2,0])   
        self.X[2,0] +=X[2,0]
        #rospy.loginfo(B[1])
        self.motor_state.copy(motor_state)
        return self.X



