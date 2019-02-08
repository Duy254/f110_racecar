#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import LaserScan
from race.msg import pid_input
import numpy as np
#import matplotlib.pyplot as plt

angle_range = 180

car_length = 1.5

desired_trajectory = 0

pub = rospy.Publisher('error', pid_input, queue_size=10)

def getRange(data,angle):
	if angle > 179.9:
		angle = 179.9
	if angle < 0:
		angle = 0
	# if angle > np.pi-1e-4:
		# angle = np.pi-1e-4
	index = len(data.ranges)*angle/angle_range
	dist = data.ranges[int(index)]
	if math.isinf(dist) or math.isnan(dist):
		return 4.0
	return data.ranges[int(index)]


def followRight(data,desired_trajectory):
	global alpha

	a = getRange(data,60)
	b = getRange(data,0)
	swing = math.radians(30)
	alpha = math.atan((a*math.cos(swing)-b)/(a*math.sin(swing)))
	#print "a","b", a, b
	print "Alpha right",math.degrees(alpha)
	curr_dist = b*math.cos(alpha)

	future_dist = curr_dist+car_length*math.sin(alpha)

	#print "Right : ",future_dist
	error = desired_trajectory - future_dist
	#print "Error right: ",error
	return error, curr_dist

def followLeft(data,desired_trajectory):
	global alpha

	a = getRange(data,120)
	b = getRange(data,179.9)
	swing = math.radians(30)
	#print "a","b", a, b
	alpha = -math.atan((a*math.cos(swing)-b)/(a*math.sin(swing)))
	print "Alpha left",math.degrees(alpha)
	curr_dist = b*math.cos(alpha)
	future_dist = curr_dist-car_length*math.sin(alpha)

	#print "Left : ",future_dist

	error = future_dist - desired_trajectory
        #print "Error left: ",error
	return error, curr_dist


def followCentre(data,desired_trajectory):
	global alpha

	a = getRange(data,130)
	b = getRange(data,179.9)
	swing = math.radians(50)
	#print "center distances: ", a, b
	alpha = -math.atan((a*math.cos(swing)-b)/(a*math.sin(swing)))
	#print "Alpha left",math.degrees(alpha)
	curr_dist1 = b*math.cos(alpha)
	future_dist1 = curr_dist1-car_length*math.sin(alpha)



	a = getRange(data,50)
	b = getRange(data,0)
	swing = math.radians(50)
	alpha = math.atan((a*math.cos(swing)-b)/(a*math.sin(swing)))
	#print "Alpha right",math.degrees(alpha)
	curr_dist2 = b*math.cos(alpha)

	future_dist2 = curr_dist2+car_length*math.sin(alpha)

	desired_trajectory = (future_dist1 + future_dist2)/2

	#print "dist 1 : ",future_dist1
	#print "dist 2 : ",future_dist2
	# print "dist : ",future_dist
	error = future_dist1 - future_dist2
	#print "Error centre: ",error
	return error, curr_dist2-curr_dist1


def callback(data):
    
    print 'In demo.py' 
    front_values = []
    k= -1
    index_count = len(data.ranges)
    print 'number of indexes:', index_count

   
    for i in data.ranges:
             k = k+1
             if k >= 180 and k<= 900:
                front_values.append(i)

    newindex_count=len(front_values)
    print 'number of indexes for 0 to 180 degrees:', newindex_count
   
    max_value= max(front_values)
    print 'distance of farthest obstacle', max_value

    max_index= front_values.index(max_value)
    print 'index of farthest obstacle', max_index

    if max_index == 0:
          max_index = 1 

    angle = (max_index * 180) / len(front_values)
    print 'angle of farthest obstacle', angle
    

    error_right, curr_dist_right = followRight(data,0.08)#followRight(data,2.)
    error_left, curr_dist_left = followRight(data,0.08)#followLeft(data,1.)
    error_centre, curr_dist_centre= followCentre(data,0.05)    

    #determines trajectory based on angular position of farthest obstacle
    if angle> 120 and angle <= 180: #if left turn required, follow left
		error = error_right
		print 'Following right, left turn'
		print 'Error', error
                if max_value>10:
                     vel = 1.2
                if max_value<=10 and max_value>=4:
                      vel = (max_value/10) * 2
                if max_value<4 and max_value>=2:
                      vel = (max_value/10) * 2
                if max_value<2 and max_value>=0.5:
                      vel = (max_value/10) * max_value
                

    if angle>= 0 and angle <= 60: #if right turn required, follow right
		error = error_left
		print 'Following left, right turn'
		print 'Error', error
                if max_value>10:
                     vel = 1
                if max_value<=10 and max_value>=4:
                      vel = (max_value/10) * 1
                if max_value<4 and max_value>=2:
                      vel = (max_value/10) * max_value
                if max_value<2 and max_value>=0.5:
                      vel = (max_value/10) * max_value
                      

    if angle>60 and angle <= 120 :#no turn required
		error = error_centre
		print 'Following centre'
		print 'Error', error
                if max_value>10:
                     vel = 6
                if max_value<=10 and max_value>=4:
                      vel = (max_value/10) *4
                if max_value<4 and max_value>=2:
                      vel = (max_value/5) * max_value
                if max_value<2 and max_value>=0.5:
                      vel = (max_value/5) * max_value
                      

    
           
    




    print 'pid_angle in demo.py', angle
    print 'velocity', vel
    print '\n'
    msg = pid_input()
    msg.pid_error = error
    msg.pid_vel = vel
    #msg.pid_angle = angle
    #msg.pid_alpha = alpha
    
    pub.publish(msg)
	

if __name__ == '__main__':
	print("Laser node started")
	rospy.init_node('dist_finder',anonymous = True)
	rospy.Subscriber("scan",LaserScan,callback)
	rospy.spin()
