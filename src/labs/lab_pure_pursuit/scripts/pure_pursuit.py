#!/usr/bin/env python

import rospy
from race.msg import drive_param
#from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import math
import numpy as np
from numpy import linalg as LA
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import csv
import os 

#############
# CONSTANTS #
#############

LOOKAHEAD_DISTANCE = 1.5 # meters
velocity = 1.5 # m

wheelbase=0.33
max_turn_degree=24

###########
# GLOBALS #
###########

# Import waypoints.csv into a list (path_points)
dirname = os.path.dirname(__file__)
filename = os.path.join(dirname, '../waypoints/levine-waypoints.csv')
with open(filename) as f:
    path_points = [tuple(line) for line in csv.reader(f)]

# Turn path_points into a list of floats to eliminate the need for casts in the code below.
#path_points = [(float(point[0]), float(point[1]), float(point[2])) for point in path_points]
        
# Publisher for 'drive_parameters' (speed and steering angle)
pub = rospy.Publisher('drive_parameters', drive_param, queue_size=1)


#############
# FUNCTIONS #
#############
    
# Computes the Euclidean distance between two 2D points p1 and p2.
#def dist(p1, p2):
#    return np.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)

# Input data is PoseStamped message from topic /pf/viz/inferred_pose.
# Runs pure pursuit and publishes velocity and steering angle.
def callback(data):

    # Note: These following numbered steps below are taken from R. Craig Coulter's paper on pure pursuit.

    # 1. Determine the current location of the vehicle (we are subscribed to vesc/odom)
    # Hint: Read up on PoseStamped message type in ROS to determine how to extract x, y, and yaw.
    orientation = data.pose.pose.orientation
    quaternion = (
	orientation.x, 
	orientation.y, 
	orientation.z, 
	orientation.w)
    euler = euler_from_quaternion(quaternion)
    linear = data.twist.twist.linear
    speed = LA.norm([linear.x, linear.y, linear.z], 2)
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    roll, pitch, yaw = euler[0], euler[1], euler[2]
    print "x:", x
    print "y:", y
    print "yaw:", yaw
    print "speed:", speed
	
    # 2. Find the path point closest to the vehicle that is >= 1 lookahead distance from vehicle's current location.
    for point in reversed(path_points):
	point_x = float(point[0])
        point_y = float(point[1])
	dist = np.sqrt((x-point_x)**2 + (y-point_y)**2)
	if dist <= LOOKAHEAD_DISTANCE:
		break
	
    # 3. Transform the goal point to vehicle coordinates. 
    goal_point = point
    print "goal_point: ", point_x, point_y
    print "distance: ", dist
    
    theta = yaw
    beta = math.atan2((point_x - x), (point_y - y))
    gamma = math.pi / 2 - theta - beta
    	
    print "theta: ", theta
    print "beta: ", beta
    print "gamma: ", gamma
	
    point_x_wrt_car = dist*math.sin(gamma) * -1.0
    point_y_wrt_car = dist*math.cos(gamma)
    # 4. Calculate the curvature = 1/r = 2x/l^2
    # The curvature is transformed into steering wheel angle by the vehicle on board controller.
    # Hint: You may need to flip to negative because for the VESC a right steering angle has a negative value.
    angle = 2 * point_x_wrt_car / dist**2
    angle = -1.0 * angle 
    print "angle: ", angle
    print "Velocity: ", velocity
    print "Angle in degrees", angle*180/np.pi
    
    if angle > 0.5236: # 0.5236 radians = 30 degrees
		print "Left steering angle too large!"
		angle = 0.5236
    if angle < -0.5236:
		print "Right steering angle too large!"
		angle = -0.5236
	
    #angle = np.clip(angle, -0.4189, 0.4189) # 0.4189 radians = 24 degrees because car can only turn 24 degrees max

    msg = drive_param()
    msg.velocity = velocity
    msg.angle = angle
    pub.publish(msg)
    
if __name__ == '__main__':
    rospy.init_node('pure_pursuit', anonymous =True)
    #rospy.Subscriber('/pf/viz/inferred_pose', PoseStamped, callback, queue_size=1)
    rospy.Subscriber("vesc/odom", Odometry, callback, queue_size=1)
    rospy.spin()

