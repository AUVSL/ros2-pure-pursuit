#!/usr/bin/env python3

# import ROS header
import rclpy
from rclpy.node import Node

# import msg header
from vectornav_msgs.msg import CommonGroup

# import Python header
import csv
import datetime
import pymap3d as pm

class TrackLoger(Node):
    def __init__(self):
        super().__init__('data_log')
        
        #init subscriber
        self.imu_common = self.create_subscription(CommonGroup, '/vectornav/raw/common', self.imu_common_callback, 10)

        # initial csv file
        # Get current date and time
        now = datetime.datetime.now()

        # Format it for filenames (e.g., 2025-07-25_17-35-20)
        timestamp = now.strftime("%Y-%m-%d_%H-%M-%S")

        # Create filename Comma Seperated Value (CSV)
        filename = f"data_{timestamp}.csv"
        file            = open(filename, 'w', encoding='UTF8', newline='')
        self.writer     = csv.writer(file)
        self.log_header = ['x', 'y', 'yaw']
        self.writer.writerow(self.log_header)

        # set reference latitude, longitude, and altitude  
        self.olat = 40.0928319 # track starting point
        self.olon = -88.2356109
        self.oalt = 203

    def imu_common_callback(self, imu_msg):
        self.yaw       = imu_msg.yawpitchroll.x
        self.latitude  = imu_msg.position.x
        self.longitude = imu_msg.position.y
        self.altitude  = imu_msg.position.z
        
        self.x, self.y, self.z = pm.geodetic2enu(self.latitude, self.longitude, self.altitude, self.olat, self.olon, self.oalt)

        self.writer.writerow([self.x, self.y, self.yaw])
        

def main():
    rclpy.init()
    logger_node = TrackLoger()
    print("Start logging")
    rclpy.spin(logger_node)
    logger_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
