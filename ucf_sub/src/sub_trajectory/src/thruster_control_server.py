#! /usr/bin/env python
import rospy
import actionlib
import tf

import numpy as np
from numpy.linalg import norm

import sub_trajectory.msg
import geometry_msgs.msg
import nav_msgs.msg

def slerp(p0, p1, t):
        omega = arccos(dot(p0/norm(p0), p1/norm(p1)))
        so = sin(omega)
        return sin((1.0-t)*omega) / so * p0 + sin(t*omega)/so * p1
        
class ThrusterServer:
    def __init__(self):
        rospy.loginfo("Thruster server spooling up")
        self.server = actionlib.SimpleActionServer('drive_thrusters', sub_trajectory.msg.ExecuteWaypoint, self.execute, False)
        self.server.start()
        
        self.tfListener = tf.TransformListener()
        
        self.thrusterPublisher = rospy.Publisher('thrusters/cmd_vel', geometry_msgs.msg.Twist, queue_size=1)
        
        self.odomListener = rospy.Subscriber("/odom", nav_msgs.msg.Odometry, self.odomCallback)
        
    def odomCallback(msg):
        
    def execute(self, goal):
        self.complete = False
        rate = rospy.rate(50)
        while not complete and not self.server.is_preempt_requested():
        
            if self.server.is_new_goal_available():
                pass
            
            if self.odom is None:
                continue
                
        
