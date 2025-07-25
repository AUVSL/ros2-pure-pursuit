#!/usr/bin/env python3

# Python header
import csv
import math
import numpy as np
import os
from numpy import linalg as la
import matplotlib.pyplot as plt

# ROS header
import rclpy
from rclpy.node import Node
import pymap3d as pm

# Message header
from dbw_msgs.msg import Dbw
from vectornav_msgs.msg import CommonGroup

# DDS header
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy


class PurePursuit(Node):

    def __init__(self):
        super().__init__('purepursuit')

        qos_control = QoSProfile(     
            # reliability=QoSReliabilityPolicy.BEST_EFFORT,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # init control period
        self.delta_t = 0.05

        # init publisher
        self.control_pub= self.create_publisher(Dbw, '/control_cmd', qos_control)

        # init subscriber
        self.imu_common = self.create_subscription(CommonGroup, '/vectornav/raw/common', self.imu_common_callback, 10)
        
        # init timer
        self.timer_ = self.create_timer(self.delta_t, self.timer_callback)
        
        # init control command
        self.cmd_msg = Dbw()
        self.cmd_msg.parkbrake = 1
        self.cmd_msg.gear      = 0
        self.cmd_msg.throttle  = 0.0
        self.cmd_msg.steering  = 0.0
        self.cmd_msg.brake     = 0.0

        # init Pure Pursuit
        self.olat       = 40.09285379 # 0_track starting point
        self.olon       = -88.23597160
        self.oalt       = 201
        self.look_ahead = 5.0 # m
        self.wheelbase  = 1.65 # meters, distance between front axle and the middle of rear axles
        self.offset     = 1.65 # meters, distance between imu and the middle of rear axles
        self.goal       = 100
        self.k          = 0.4
        
        # init waypoints in local North East Down Forward (NEDF)
        dirname  = os.path.dirname(__file__)
        filename = os.path.join(dirname, '../waypoints/0_track.csv')

        with open(filename) as f:
            path_points = [tuple(line) for line in csv.reader(f)]
        path_points.pop(0)
        self.path_points_x       = np.array([float(point[0]) for point in path_points]) # east
        self.path_points_y       = np.array([float(point[1]) for point in path_points]) # north
        self.path_points_heading = np.array([float(point[2]) for point in path_points]) # heading
        self.wp_size             = len(self.path_points_x)
        self.dist_arr            = np.zeros(self.wp_size)

    def imu_common_callback(self, imu_msg):
        self.yaw       = imu_msg.yawpitchroll.x
        self.latitude  = imu_msg.position.x
        self.longitude = imu_msg.position.y
        self.altitude  = imu_msg.position.z
        
        self.x_ned, self.y_ned, self.z_ned = pm.geodetic2enu(self.latitude, self.longitude, self.altitude, self.olat, self.olon, self.oalt)

    def pure_pursuit(self):
        # calculate distance array from last goal point to the end
        for i in range(self.goal, self.wp_size):
            self.dist_arr[i] = round(np.sqrt((self.path_points_x[i] - self.x_ned)**2 + (self.path_points_y[i] - self.y_ned)**2), 5)
        
        # only find point near the distance of look_ahead point
        goal_area = np.where( (self.dist_arr < self.look_ahead + 0.3) & (self.dist_arr > self.look_ahead - 0.3) )[0]

        # find goal pont in the goal area and check the direction is correct
        for idx in goal_area:
            v1 = [self.path_points_x[idx]-self.x_ned , self.path_points_y[idx]-self.y_ned]
            v2 = [np.cos(self.curr_yaw), np.sin(self.curr_yaw)]
            cosang = np.dot(v1, v2)
            sinang = la.norm(np.cross(v1, v2))
            temp_angle = np.arctan2(sinang, cosang) # [-pi, pi]

            if abs(temp_angle) < np.pi/2:
                self.goal = idx
                break

        # real look ahead distance
        L = self.dist_arr[self.goal]

        # pure pursuit control law
        alpha = self.path_points_heading[self.goal] - self.yaw 
        while alpha > 360:
            alpha -= 360
        
        while alpha < -360:
            alpha += 360        
 
        print("path heading: ", self.path_points_heading[self.goal], "yaw: ", self.yaw)
        front_wheel_angle = -2*math.atan((2*self.k*self.wheelbase*math.sin(alpha)) / L) 
        print("target: ", self.path_points_x[self.goal], self.path_points_y[self.goal], "dis:", self.dist_arr[self.goal])
        print("current: ", self.x_ned, self.y_ned)
        # print("alpha:", alpha, "front wheel angle: ", front_wheel_angle)
        # convert to control command
        steering_cmd = 100.0*(front_wheel_angle*57.3)/31.5
        if steering_cmd > 100.0:
            steering_cmd = 100.0
        if steering_cmd < -100.0:
            steering_cmd = -100.0

        print("steer_cmd", steering_cmd)

        return steering_cmd

    def timer_callback(self):
        self.cmd_msg.parkbrake = 0
        self.cmd_msg.gear      = 1
        self.cmd_msg.steering  = self.pure_pursuit()
        self.cmd_msg.throttle  = 50.0
        self.cmd_msg.brake     = 0.0

        self.control_pub.publish(self.cmd_msg)
       
        
def main():
    rclpy.init()
    tracking_node = PurePursuit()
    print("Pure Pursuit Controller Tracking")
    rclpy.spin(tracking_node)
    tracking_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
