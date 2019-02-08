#!/usr/bin/env python

import rospy

from race.msg import drive_param
from race.msg import pid_input
import math
import numpy as np

pub = rospy.Publisher('drive_parameters', drive_param, queue_size=1)

kp = 5
kd = 0.005 
kp_vel = 42.0
kd_vel = 0.0

ki = 0.0
servo_offset = 18.0*math.pi/180
prev_error = 0.0 
error = 0.0
integral = 0.0
vel_input = 1.0


def control(data):
	global integral
	global prev_error
	global vel_input
	global kp
	global ki
	global kd
	global kd_vel
	global kp_vel
	velocity = data.pid_vel
	angle = servo_offset
	error = 5*data.pid_error
	print "inital error: ", error
	#print "Error Control",error
	if error!=0.0:
                if data.pid_vel<8 and data.pid_vel>=3:
                        kp = 0.01
                        kd = 0.001

                if data.pid_vel<3 and data.pid_vel>=2:
                        kp = 1.8
                        kd = 0.014

                if data.pid_vel<2 and data.pid_vel>=1:
                        kp = 2
                        kd = 0.018
                
                if data.pid_vel<1:
                        kp = 4.25
                        kd = 0.04

                


		control_error = kp*error + kd*(error - prev_error)# + ki*integral
		
		
		
		angle = angle + control_error*np.pi/180
		print "steering angle: ", angle

		control_error_vel = kp_vel*error + kd_vel*(error - prev_error)
		
		velocity = velocity + abs(control_error_vel)
               
               # print 'pid_angle in control2.py', data.pid_angle
		print 'kp:', kp
                print 'kd:', kd

	prev_error = error
        if angle >= 10*np.pi/180 or angle <= -10*np.pi/180:
		velocity = 1.8

	if angle > 20*np.pi/180 or angle < -20*np.pi/180:
		velocity = 0.8

	if angle >= -1*np.pi/180 and angle <= 1*np.pi/180:
		velocity = 4

	if velocity < 0:
		velocity = 1.5

	if velocity > 2.5:
		velocity = 4

	msg = drive_param()
	msg.velocity = velocity #data.pid_vel
	msg.angle = angle
	pub.publish(msg)

def listener():
	rospy.init_node('pid_controller', anonymous=True)
	rospy.Subscriber("error", pid_input, control)
	global kp
	global ki
	global kd
	global vel_input
	kp = input("Enter Kp Value: ")
	ki = input("Enter Ki Value: ")
	kd = input("Enter Kd Value: ")
	vel_input = input("Enter Velocity: ")
	rospy.spin()


if __name__ == '__main__':
	print("Listening to error for PID")
	listener()
