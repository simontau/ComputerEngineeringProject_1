#!/usr/bin/env python3
#
# Copyright 2018 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Jeonggeun Lim, Ryan Shim, Gilbert

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile
from sensor_msgs.msg import LaserScan
import numpy as np

class Turtlebot3ObstacleDetection(Node):

    def __init__(self):
        super().__init__('turtlebot3_obstacle_detection')
        print('TurtleBot3 Obstacle Detection - Auto Move Enabled')
        print('----------------------------------------------')
        print('stop angle: -90 ~ 90 deg')
        print('stop distance: 0.5 m')
        print('----------------------------------------------')

        self.scan_ranges = []
        self.has_scan_received = False

        self.stop_distance = 0.2
        self.turn_distance = 0.5
        self.tele_twist = Twist()
        self.tele_twist.linear.x = 0.1
        self.tele_twist.angular.z = 0.0

        qos = QoSProfile(depth=10)

        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', qos)

        self.scan_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile=qos_profile_sensor_data)

        self.cmd_vel_raw_sub = self.create_subscription(
            Twist,
            'cmd_vel_raw',
            self.cmd_vel_raw_callback,
            qos_profile=qos_profile_sensor_data)

        self.timer = self.create_timer(0.1, self.timer_callback)

    def scan_callback(self, msg):
        self.scan_ranges = msg.ranges
        self.has_scan_received = True

    def cmd_vel_raw_callback(self, msg):
        self.tele_twist = msg

    def timer_callback(self):
        if self.has_scan_received:
            self.detect_obstacle()

    def detect_obstacle(self):
        # Filtering the readings:
        # Looping through the array and substituing each 0 reading with 3.5
        for i in range(len(self.scan_ranges)):
            if self.scan_ranges[i] == 0.0:
                self.scan_ranges[i] = 3.5

        # Splitting into cones:
        #left_range = int(len(self.scan_ranges) / 4)
        #right_range = int(len(self.scan_ranges) * 3 / 4)
        
        # New points in the circle of the sensor.
        # Each name is: px (where x is x/20)
        p1 = int(len(self.scan_ranges) * 1/20)   # 18° from center
        p3 = int(len(self.scan_ranges) * 3/20)   # 54° from center
        p5 = int(len(self.scan_ranges) * 5/20)   # 90° from center (quarter circle)
        p7 = int(len(self.scan_ranges) * 7/20)   # 126° from center
        p9 = int(len(self.scan_ranges) * 9/20)   # 162° from center
        p11 = int(len(self.scan_ranges) * 11/20) # 198° from center
        p13 = int(len(self.scan_ranges) * 13/20) # 234° from center
        p15 = int(len(self.scan_ranges) * 15/20) # 270° from center (three-quarter circle)
        p17 = int(len(self.scan_ranges) * 17/20) # 306° from center
        p19 = int(len(self.scan_ranges) * 19/20) # 342° from center

        # Testing:
        #print(f"Left readings: {self.scan_ranges[0:left_range]}")
        #print(f"Right readings: {self.scan_ranges[right_range:len(self.scan_ranges)]}")

        # Taking the minimum of each reading of each cone:
        front = min(min(self.scan_ranges[p19:]), min(self.scan_ranges[:p1]))
        front1v = min(self.scan_ranges[p1:p3])
        front2v = min(self.scan_ranges[p3:p5])
        back2v = min(self.scan_ranges[p5:p7])
        back1v = min(self.scan_ranges[p7:p9])
        back = min(self.scan_ranges[p9:p11])
        back1h = min(self.scan_ranges[p11:p13])
        back2h = min(self.scan_ranges[p13:p15])
        front2h = min(self.scan_ranges[p15:p17])
        front1h = min(self.scan_ranges[p17:p19])

        # Decision making variable to decide whether to turn left or right:
        # Right variable:
        turn_left = min(front1v, front2v)
        turn_right = min(front1h, front2h)

        # Angular velocity calculation with safety check
        if front > 0.1: # Avoid division by zero
            # If we need the robot to turn right we will calculate the angular velocity to be negative:
            if turn_right > turn_left:
                self.tele_twist.angular.z = (-1)*((np.pi / 2) * (self.tele_twist.linear.x / front) / 5) # Adjust the division here to modify the turning speed
            else:
                 self.tele_twist.angular.z = ((np.pi / 2) * (self.tele_twist.linear.x / front) / 5) # Adjust the division here to modify the turning speed
        else:
            self.tele_twist.angular.z = 0.5  # Default turn rate if too close

        # Defining the obstacle distance to make the robot turn:
        obstacle_distance = min(
            # Taking the minimum of the front three cones:
            front, front1v, front1h
        )

        twist = Twist()

        # Better logic flow with elif statements
        if obstacle_distance < self.stop_distance:
            twist.linear.x = 0.0
            twist.angular.z = self.tele_twist.angular.z
            self.get_logger().info(f'Obstacle detected! Stopping. Distance: {obstacle_distance:.2f} m')
        elif obstacle_distance < self.turn_distance:
            # Slower forward. Adjust the angular velocity subtracted to modify:
            twist.linear.x = 0.15 - abs(self.tele_twist.angular.z)
            twist.angular.z = self.tele_twist.angular.z
            self.get_logger().info(f'Obstacle ahead, slowing. Distance: {obstacle_distance:.2f} m')
        else:
            twist.linear.x = 0.1
            twist.angular.z = 0.0

        # Publishing the twist
        self.cmd_vel_pub.publish(twist)

        
def main(args=None):
    rclpy.init(args=args)
    turtlebot3_obstacle_detection = Turtlebot3ObstacleDetection()
    rclpy.spin(turtlebot3_obstacle_detection)

    turtlebot3_obstacle_detection.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()