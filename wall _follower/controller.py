#! /usr/bin/env python3
import math
import numpy as np
import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from f1tenth_simulator.msg import PIDmsgs
pub = rospy.Publisher('/drive', AckermannDriveStamped, queue_size=1)
kp = 10
kd = 0.01
kp_vel = 35.0
kd_vel = 0.0
ki = 0.0
servo_offset = 18.0*math.pi/180
prev_error = 0.0
error = 0.0
integral = 0.0
avg_vel = 0.0
iteration = 0.0
total = 0.0

def control(data):
    global integral
    global prev_error
    global kp
    global kd
    global kd_vel
    global kp_vel
    global avg_vel
    global iteration
    global total
    velocity = data.pid_vel
    angle = servo_offset
    error = 5*data.pid_error
    collision = data.dist_collision
    print("Error Control", error)
    if error != 0.0:
       
        error_angle = kp*error + kd*(error - prev_error)  
        print("Control", error_angle)
       
        
        angle = angle + error_angle*np.pi/180
        error_vel = kp_vel*error + kd_vel*(error - prev_error)
        velocity = velocity + abs(error_vel)

    prev_error = error

    if angle > 30*np.pi/180:
        angle = 30*np.pi/180
    if angle < -30*np.pi/180:
        angle = -30*np.pi/180

    if angle > 15*np.pi/180  or angle < -15*np.pi/180:
        velocity = 1.5
    elif  angle > 10*np.pi/180  or angle < -10*np.pi/180:
        velocity = 2.5
    elif angle > 5*np.pi/180  or angle < -5*np.pi/180: 
        velocity = 3
    else:
        velocity = 6
     
    if collision < 3.0:
        velocity =1    
    

    print("Velocity", velocity)
    print("Angle", angle*180/np.pi)
    msg = AckermannDriveStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = 'cmd_vel'
    msg.drive.speed = velocity
    msg.drive.steering_angle = angle
    pub.publish(msg)


def listener():
    rospy.init_node('pid_controller', anonymous=True)
    rospy.Subscriber("error", PIDmsgs, control)
    rospy.spin()


if __name__ == '__main__':
    print("Listening to error for PID")
    listener()