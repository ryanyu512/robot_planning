#!/usr/bin/env python  

import sys

import numpy as np

import rospy
import threading
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

import tf

STRAIGHT = 0
LEFT  = 1
RIGHT = 2

class Pose():
    def __init__(self):
        self.lock = threading.Lock()
        self.x = None
        self.y = None
        self.yaw = None

    def update(self, pose):
        self.lock.acquire()
        self.x = pose[0]
        self.y = pose[1]
        self.yaw = pose[2]
        self.lock.release()

    def get_pose(self):

        self.lock.acquire()
        
        pose = (self.x, self.y, self.yaw)

        self.lock.release()

        return pose

    def compute_dist(self, target):

        diff = np.array(target) - np.array([self.x, self.y])

        return np.linalg.norm(diff)


class ScanData():
    def __init__(self):

        self.data = None
        self.f_dist = None
        self.r_dist = None
        self.l_dist = None
    
    def get_min(self, s_ind, e_ind):
        tmp_data = [d for d in self.data.ranges[s_ind:e_ind + 1] if self.data.range_min < d < self.data.range_max]
        return np.min(tmp_data)

    def update(self, data):
        self.data = data
        self.f_dist = np.min([self.get_min(0, 5), self.get_min(360, 365)])
        self.r_dist = self.get_min(265, 275)
        self.l_dist = self.get_min(35, 45)

    def get(self):

        return (self.f_dist, self.r_dist, self.l_dist)

    def at(self, ang):
        return 
        

cur_pose = Pose()
cur_scan = ScanData()

def init_listener():
    rospy.init_node('listener', anonymous = True)
    rospy.Subscriber('/odom', Odometry, pose_sub_callback)
    rospy.Subscriber('/scan', LaserScan, scan_sub_callback)

def pose_sub_callback(data):

    #obtain yaw angle
    q = (data.pose.pose.orientation.x, 
         data.pose.pose.orientation.y, 
         data.pose.pose.orientation.z, 
         data.pose.pose.orientation.w)
    
    yaw  = tf.transformations.euler_from_quaternion(q)[2] #roll, pitch, yaw

    #obtain pose
    pose = (data.pose.pose.position.x, 
            data.pose.pose.position.y, 
            yaw) 

    cur_pose.update(pose)

def scan_sub_callback(data):

    cur_scan.update(data)


class Bug():
    def __init__(self, 
                 goal, 
                 goal_threshold = 0.1, 
                 hit_wall_threshold = 0.5, 
                 face_target_delta = 0.05):
        
        self.cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.goal  = goal
        self.goal_threshold = goal_threshold
        self.hit_wall_threshold = hit_wall_threshold
        self.face_target_delta = face_target_delta

    def move(self, action):
        
        cmd_vel = Twist()
        if action == 'STRAIGHT':
            cmd_vel.linear.x  =  1.0
        elif action == 'LEFT':
            cmd_vel.angular.z =  0.25
        elif action == 'RIGHT':
            cmd_vel.angular.z = -0.25
        
        self.cmd_pub.publish(cmd_vel)

    def choose_actions(self):

        (cx, cy, cyaw) = cur_pose.get_pose()

        if None in (cx, cy, cyaw):
            return False
        
        desired_heading = self.compute_desired_heading(cx, cy)

        if desired_heading - self.face_target_delta <= cyaw <= desired_heading + self.face_target_delta:
            return STRAIGHT
        elif desired_heading - cyaw > 0:
            return LEFT
        else:
            return RIGHT

    def compute_desired_heading(self, cx, cy):

        return np.math.atan2(self.goal[1] - cy, self.goal[0] - cx)

    def move_until_hit_wall(self):
        print("=== move_until_hit_wall ===")

        while cur_pose.compute_dist(self.goal) > self.delta:

            (f_dist, l_dist, r_dist) = cur_scan.get()

            if f_dist < self.hit_wall_threshold:
                return True

            action = self.choose_actions()
            rospy.sleep(0.01)

        return False

    def is_leave_wall(self):

        (cx, cy, cyaw) = cur_pose.get_pose()
        desired_heading = self.compute_desired_heading(cx, cy)
        rel_ang = desired_heading - cyaw

        scan_dist = cur_scan.at(rel_ang)

    def follow_wall(self):
        print("=== follow_wall ===")
        
        while cur_scan.get()[0] < self.hit_wall_threshold:
            self.move(RIGHT)
            rospy.sleep(0.01)



    def navigate(self):

        init_listener()
        print("=== wait for the sensors ready ===")
        rospy.sleep(1)

        while cur_pose.compute_dist(self.goal) > self.delta:
            
            if self.move_until_hit_wall():
                self.follow_wall()

            break

if __name__ == '__main__':

    goal = [1, 1]
    bug = Bug(goal)

    bug.navigate()
