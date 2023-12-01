#!/usr/bin/env python  

import sys
import math
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
ROTATE = 3
NO_ACTION = 4

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
        self.lock = threading.Lock()
        self.data = None
        self.f_dist = None
        self.r_dist = None
        self.l_dist = None
    
    def get_min(self, s_ind, e_ind):
        tmp_data = [d for d in self.data.ranges[s_ind:e_ind + 1] if self.data.range_min <= d <= self.data.range_max]

        if len(tmp_data) > 0:
            return np.min(tmp_data)
        else:
            return np.inf
        
    def update(self, data):
        self.lock.acquire()
        self.data = data
        self.f_dist = np.min([self.get_min(0, 20), self.get_min(340, 359)])
        self.r_dist = self.get_min(300, 329)
        self.l_dist = self.get_min(30, 69)
        #print(data.angle_increment)
        self.lock.release()

    def get(self):
        self.lock.acquire()
        tmp = (self.f_dist, self.r_dist, self.l_dist)
        self.lock.release()
        return tmp

    def at(self, ang, range = 35):
        
        if self.data is None:
            return np.inf

        self.lock.acquire()
        max_ind = len(self.data.ranges) - 1
        if ang - range < 0:
            m1 = self.get_min(max_ind + (ang - range), max_ind)
            m2 = self.get_min(0, ang)
            dist = min([m1, m2])
        elif ang + range > max_ind:
            m1 = self.get_min(ang, max_ind)
            m2 = self.get_min(0, ang + range - max_ind)
            dist = min([m1, m2])
        else:
            dist = self.get_min(ang, ang+range)

        self.lock.release()
        
        return dist
        
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
                 goal_threshold = 0.10, 
                 hit_wall_threshold = 0.5, 
                 fol_wall_threshold = 0.4,
                 face_target_delta = 20./180*np.math.pi,
                 max_lin_spd_x_go2goal = 0.15,
                 max_lin_spd_x_folwall = 0.05,
                 max_ang_spd_z = 1.0, 
                 ang_K = 2,
                 lin_K = 0.1):
        
        self.cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.goal  = goal
        self.goal_threshold = goal_threshold
        self.hit_wall_threshold = hit_wall_threshold 
        self.fol_wall_threshold = fol_wall_threshold
        self.face_target_delta = face_target_delta
        self.lin_spd_x_go2goal = max_lin_spd_x_go2goal
        self.lin_spd_x_folwall = max_lin_spd_x_folwall
        self.max_ang_spd_z = max_ang_spd_z
        self.ang_K = ang_K
        self.lin_K = lin_K
        self.focus_sensor = None

    def move(self, action):
        
        cmd_vel = Twist()
        if action == STRAIGHT:
            cmd_vel.linear.x  =  self.lin_spd_x_go2goal

            (f_dist, r_dist, l_dist) = cur_scan.get()
            
            #dis_e => -ve => too closed to the wall => turn right
            #dis_e => +ve => too far from the wall  => turn left
            if l_dist < r_dist:
                dis_e =   l_dist - self.hit_wall_threshold
            else:
                dis_e =  r_dist - self.hit_wall_threshold

            if dis_e < 0:
                if l_dist < r_dist:
                    dis_e_sign = math.copysign(1, dis_e)
                else:
                    dis_e_sign = math.copysign(1, -dis_e)

                cmd_vel.angular.z = dis_e_sign*min(abs(dis_e*self.ang_K), self.max_ang_spd_z)
                cmd_vel.linear.x  = 0.05


        elif action == LEFT:
            cmd_vel.angular.z =  self.max_ang_spd_z
        elif action == RIGHT:
            cmd_vel.angular.z = -self.max_ang_spd_z
        elif action == ROTATE:
            
            (cx, cy, cyaw) = cur_pose.get_pose()
            desired_heading = self.compute_desired_heading(cx, cy)
            dh_sign = math.copysign(1, desired_heading)
            cy_sign = math.copysign(1, cyaw)

            if dh_sign != cy_sign:
                if cy_sign == -1:
                    desired_heading -= 2*np.math.pi
                else:
                    desired_heading += 2*np.math.pi

            ang_e = desired_heading - cyaw

            if abs(ang_e) > np.math.pi:
                if cy_sign == 1:
                    ang_e -= 2*np.pi
                else:
                    ang_e += 2*np.pi

            sign = 1 if ang_e >= 0 else -1
            cmd_vel.angular.z = sign*min(abs(ang_e)*self.ang_K, self.max_ang_spd_z)       

        self.cmd_pub.publish(cmd_vel)

    def follow_wall_p_control(self):
            
        (f_dist, r_dist, l_dist) = cur_scan.get()
        cmd_vel = Twist()

        if self.focus_sensor is None:
            self.focus_sensor = 'L' if l_dist < r_dist else 'R'

        print(f"F: {f_dist}, R: {r_dist}, L: {l_dist}")
        if self.focus_sensor == 'L':
            print('focus on left sensor')
            #dis_e => -ve => too closed to the wall => turn right
            #dis_e => +ve => too far from the wall  => turn left
            dis_e = l_dist - self.fol_wall_threshold
            dis_e_sign = math.copysign(1, dis_e)

            cmd_vel.angular.z = dis_e_sign*min(abs(dis_e*self.ang_K), self.max_ang_spd_z)
            cmd_vel.linear.x  = self.lin_K*(abs(1/dis_e) - 1./f_dist)
            cmd_vel.linear.x  = max(min(cmd_vel.linear.x, self.lin_spd_x_folwall), 0)
            self.cmd_pub.publish(cmd_vel)
        else:
            print('focus on right sensor')
            #dis_e => -ve => too closed to the wall => turn left
            #dis_e => +ve => too far from the wall  => turn right
            dis_e = -(r_dist - self.fol_wall_threshold)
            dis_e_sign = math.copysign(1, dis_e)

            cmd_vel.angular.z = dis_e_sign*min(abs(dis_e*self.ang_K), self.max_ang_spd_z)
            cmd_vel.linear.x  = self.lin_K*(abs(1/dis_e) - 1./f_dist)
            cmd_vel.linear.x  = max(min(cmd_vel.linear.x, self.lin_spd_x_folwall), 0)
            self.cmd_pub.publish(cmd_vel)

    def choose_actions(self):

        (cx, cy, cyaw) = cur_pose.get_pose()

        if None in (cx, cy, cyaw):
            return NO_ACTION
        
        desired_heading = t = self.compute_desired_heading(cx, cy)
        dh_sign = math.copysign(1, desired_heading)
        cy_sign = math.copysign(1, cyaw)
        if dh_sign != cy_sign:
            if cy_sign == -1:
                desired_heading -= 2*np.math.pi
            else:
                desired_heading += 2*np.math.pi

        ang_e = desired_heading - cyaw

        if abs(ang_e) > np.math.pi:
            if cy_sign == 1:
                ang_e -= 2*np.pi
            else:
                ang_e += 2*np.pi

        f_dist, r_dist, l_dist = cur_scan.get()
        if abs(ang_e) <= self.face_target_delta:
            print("move to goal - straight")
            return STRAIGHT
        else:
            print("move to goal - rotate")
            return ROTATE

    def compute_desired_heading(self, cx, cy):

        return np.math.atan2(self.goal[1] - cy, self.goal[0] - cx)

    def move_until_hit_wall(self):
        print("=== move_until_hit_wall ===")

        while cur_pose.compute_dist(self.goal) > self.goal_threshold:

            action = self.choose_actions()

            (f_dist, r_dist, l_dist) = cur_scan.get()

            if f_dist < self.hit_wall_threshold and action == STRAIGHT:
                print("=== meet wall ===")
                return True

            self.move(action)
            rospy.sleep(0.01)

        return False

    def is_leave_wall(self):

        (cx, cy, cyaw) = cur_pose.get_pose()
        desired_heading = self.compute_desired_heading(cx, cy)
        #make the desired_heading relative to the robot heading/laser heading
        rel_ang = desired_heading - cyaw 
        if rel_ang < 0:
            
            rel_ang += 2*np.math.pi
        
        scan_dist = cur_scan.at(int(rel_ang/cur_scan.data.angle_increment))

        f_dist, r_dist, l_dist  = cur_scan.get()

        if scan_dist > self.hit_wall_threshold*1:
            print("=== leave wall ===")
            self.focus_sensor = None
            return True
        
        return False

    def follow_wall(self):
        print("=== follow_wall ===")
        (cx, cy, cyaw) = cur_pose.get_pose()
        (f_dist, r_dist, l_dist) = cur_scan.get()
        desired_heading = self.compute_desired_heading(cx, cy)

        while cur_scan.get()[0] <= self.hit_wall_threshold:
            print(f"=== follow_wall: rotate ===: front scan: {cur_scan.get()[0]}, threshold:{self.hit_wall_threshold}")

            if desired_heading - cyaw  < 0:
                self.move(RIGHT)
            else:
                self.move(LEFT)
            rospy.sleep(0.01)
            
        while not self.is_leave_wall():
            print("=== follow wall p control ===")
            self.follow_wall_p_control()
            rospy.sleep(0.01)
         
    
    def navigate(self):
        
        print("=== setup sensor ===")
        init_listener() 
        rospy.sleep(1)

        print("=== navigate ===")
        while cur_pose.compute_dist(self.goal) > self.goal_threshold:
            if self.move_until_hit_wall():
                self.follow_wall()
        self.move(None)
        rospy.sleep(0.01)

if __name__ == '__main__':
    
    print(sys.argv)
    if len(sys.argv) == 3:
        argv = sys.argv[1:3]
        tx, ty = map(float, [argv[0], argv[1]])
        bug = Bug([tx, ty])

        bug.navigate()
