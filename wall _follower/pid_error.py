#! /usr/bin/env python3
import math
import pdb
import sys
import numpy as np
import rospy
import yaml
from f1tenth_simulator.msg import PIDmsgs
from sensor_msgs.msg import LaserScan

ANGLE_RANGE = 270  # Hokuyo 10LX has 270 degrees scan
DESIRED_DISTANCE_RIGHT = 1  
DESIRED_DISTANCE_LEFT = 1  
VELOCITY = 1.00  
CAR_LENGTH = 0.50  
alpha = 0
error = 0
final_direction = 0
prev_direction = 0
pub = rospy.Publisher('error', PIDmsgs, queue_size=1)

def getRange(data, angle):
    
    # angle: between -45 to 225 degrees, where 0 degrees is directly to the right
    if angle > 179.9:
        angle = 179.9
    index = len(data.ranges) * (angle + 45) / ANGLE_RANGE
    dist = data.ranges[int(index)]
    if math.isinf(dist):
        return 10.0
    if math.isnan(dist):
        return 4.0
    return data.ranges[int(index)]

def collision_distance(data,angle):
    index = len(data.ranges) * (angle + 45) / ANGLE_RANGE
    dist_to_collision = data.ranges[int(index)]
    return dist_to_collision

def followLeft(data, desired_trajectory):
    # desired_trajectory: desired distance to the left wall [meters]
    global alpha
    a = getRange(data, 120)
    b = getRange(data, 179.9)
    swing = math.radians(60)
    alpha = -math.atan((a*math.cos(swing)-b)/(a*math.sin(swing)))
    
    curr_dist = b*math.cos(alpha)
    future_dist = curr_dist - CAR_LENGTH * math.sin(alpha)
    error = future_dist - desired_trajectory
    
    print("Current Distance Left: ", curr_dist)
    
    return error, curr_dist



def followRight(data, desired_trajectory):
    # desired_trajetory: desired distance to the right wall [meters]
    global alpha

    a = getRange(data, 60)
    b = getRange(data, 0)
    swing = math.radians(60)
    alpha = math.atan((a*math.cos(swing)-b)/(a*math.sin(swing)))
    
    curr_dist = b*math.cos(alpha)

    future_dist = curr_dist + CAR_LENGTH * math.sin(alpha)
    
    error = desired_trajectory - future_dist

    print("Current Distance Right: ", curr_dist)
    return error, curr_dist

def centerfollower(data):
    global alpha
    a = getRange(data, 120)
    b = getRange(data, 179.9)
    swing = math.radians(60)
    alpha1 = -math.atan((a*math.cos(swing)-b)/(a*math.sin(swing)))
    curr_dist1 = b*math.cos(alpha1)
    if curr_dist1 > 10:
        curr_dist1= 10
    future_dist1 = curr_dist1-CAR_LENGTH*math.sin(alpha1)

    a = getRange(data, 60)
    b = getRange(data, 0)
    swing = math.radians(60)
    alpha2 = math.atan((a*math.cos(swing)-b)/(a*math.sin(swing)))
    
    curr_dist2 = b*math.cos(alpha2)
    
    if curr_dist1 > 3:
        curr_dist2 = curr_dist2-2
    if curr_dist2 > 3:
        curr_dist1 = curr_dist2-2
    
    
    future_dist1 = curr_dist1-CAR_LENGTH*math.sin(alpha1)
    future_dist2 = curr_dist2+CAR_LENGTH*math.sin(alpha2)



    error = future_dist1 - future_dist2
    print("Error : ", error)
    return error, curr_dist2 - curr_dist1

    
def callback(data):
   global error
   global alpha
   global final_direction
   global prev_direction

   error,current_distance= centerfollower(data)
   
   dist_colli=collision_distance(data,90)
   msg= PIDmsgs()
   msg.pid_error = error
   msg.pid_vel = VELOCITY
   msg.dist_collision=dist_colli
   pub.publish(msg)




if __name__ == '__main__':
    print("error_calc started")
    rospy.init_node('dist_finder', anonymous=True)
    rospy.Subscriber("scan", LaserScan, callback)
    rospy.spin()
