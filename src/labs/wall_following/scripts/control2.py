#!/usr/bin/env python

import rospy

from race.msg import drive_param
from race.msg import pid_input
import math
import numpy as np

pub = rospy.Publisher('drive_parameters', drive_param, queue_size=1)

kp = 5
kd = 0.02
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
	#print "Error Control",error
	if error!=0.0:
                #if data.pid_vel<=15 and data.pid_vel>10:
                 #       kp = 0.35
                  #      kd = 0.005
                pid_vel = data.pid_vel 
                if data.pid_vel<=10 and data.pid_vel>8:
                        kp = 0.01 #pid_vel/1000 * kp
                        kd = 0.001 #pid_vel/100 * kd
                
                if data.pid_vel<=8 and data.pid_vel>6:
                        kp = 0.05 #pid_vel/90 * kp
                        kd = 0.005 #pid_vel/90 * kd

                if data.pid_vel<=6 and data.pid_vel>4:
                        kp = 0.08 #pid_vel/15 * kp
                        kd = 0.025 #pid_vel/15 * kd

                if data.pid_vel<=4 and data.pid_vel>2:
                        kp = 1.0 #pid_vel/10 * kp
                        kd = 0.035 #pid_vel/10 * kd
                if data.pid_vel<2 and data.pid_vel>=0.5:
                        kp = 1.8  #pid_vel/5 * kp
                        kd = 0.05 #pid_vel/0.4 * kd
               
                if data.pid_vel<0.5 :
                       kp =  2
                       kd =  0.07

                


		control_error = kp*error + kd*(error - prev_error)# + ki*integral
		
		
		
		angle = angle + control_error*np.pi/180
	

		control_error_vel = kp_vel*error + kd_vel*(error - prev_error)
		
		velocity = velocity + abs(control_error_vel)
               
               # print 'pid_angle in control2.py', data.pid_angle
		print 'kp:', kp
                print 'kd:', kd

	prev_error = error
        if angle >= 10*np.pi/180 or angle <= -10*np.pi/180:
		velocity = 1

	if angle > 20*np.pi/180 or angle < -20*np.pi/180:
		velocity = 0.3

	if angle >= -1*np.pi/180 and angle <= 1*np.pi/180:
		velocity = 2.5

	if velocity < 0:
		velocity = 1

	if velocity > 2.5:
		velocity = 2.5

	msg = drive_param()
	msg.velocity = data.pid_vel
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
