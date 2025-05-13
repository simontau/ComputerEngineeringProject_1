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
import threading
import time
import smbus
import RPi.GPIO as GPIO
from gpiozero import LED

class Turtlebot3ObstacleDetection(Node):

    # Constants applied throughout the program:
    CONSTANTS = {
        'RUN_DURATION': 120, # Seconds
        'STOP_DISTANCE': 0.2, # Meters
        'TURN_DISTANCE': 0.4, # Meters
        'MIN_SCAN_DISTANCE': 0.05, # Meters, Used for filtering invalid scans
        'DEFAULT_DISTANCE': 3.5, # Meters, Replaced instead of invalid scans
        'COLLISION_THRESHOLD': 0.65, # Seconds, Used for collision prediction
        'POCKET_THRESHOLD': 0.2, # Meters, Used for special navigation cases
        'VICTIM_READING_THRESHOLD': 4, # Number of consecutive red readings in order to start a thread
        'COLOR_TOLERANCE_PERCENT': 12, # Percent, Color deviation threshold
        'WHEEL_PROTECTION_THRESHOLD': 0.15 # Meters, to avoid wheel getting stuck
    }

    def __init__(self):
        super().__init__('turtlebot3_obstacle_detection')
        self.run_duration = self.CONSTANTS['RUN_DURATION']
        self.start_time = None

        self.scan_ranges = []
        self.has_scan_received = False

        self.stop_distance = self.CONSTANTS['STOP_DISTANCE']
        self.turn_distance = self.CONSTANTS['TURN_DISTANCE']
        self.tele_twist = Twist()
        self.tele_twist.linear.x = 0.21
        self.tele_twist.angular.z = 0.0

        # Variables to calculate average speed:
        self.speed_updates = 0
        self.speed_accumulation = 0

        # Collusion counter variable:
        self.collision_count = 0
        self.collision_flag = False

        # Victim counter variable:
        self.victim_flag = False
        self.victim_readings = 0
        self.victims_counted = 0

        # Led object (GPIO 18)
        self.led = LED(18)

        # Get I2C bus
        self.bus = smbus.SMBus(1)  # or smbus.SMBus(0)
        # ISL29125 address, 0x44(68)
        # Select configuration-1 register, 0x01(01)
        # 0x05: Operation: RGB, Range: 360 lux, Res: 16 Bits (as per datasheet)
        self.bus.write_byte_data(0x44, 0x01, 0x05)

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

        self.timer = self.create_timer(0.05, self.timer_callback)

    def scan_callback(self, msg):
        self.scan_ranges = msg.ranges
        self.has_scan_received = True

    def cmd_vel_raw_callback(self, msg):
        self.tele_twist = msg

    # Global thread function to detect colissions, pause and then register 1 collision:
    def collision_counter(self):
        # Checks if a flag should open:
        if not self.collision_flag:
            self.collision_flag = True
            self.get_logger().info('A collision registered.')
            # Increment the counter by exactly 1 collision:
            self.collision_count += 1
            # Debounce periode
            time.sleep(2)
            self.collision_flag = False

    # Global thread victim counter function going into victim detection state:
    def victim_counter(self):
        if not self.victim_flag:
            self.victim_flag = True
            # Turning on led:
            self.led.on()
            print(f"red={self.red:.3f}, green={self.green:.3f}, blue={self.blue:.3f}")
            self.get_logger().info('We have registered a victim.')
            # Increment counter by 1:
            self.victims_counted += 1
            # Debounce period:
            time.sleep(2)
            self.led.off()
            self.victim_flag = False

    # Function for LED sensor:
    def getAndUpdateColour(self):

        # Data variables for LED:
        # Read 6 bytes starting from register 0x09
        data = self.bus.read_i2c_block_data(0x44, 0x09, 6)
        
        # Correctly convert the data to red, green, and blue int values
        self.green = 0.9 * (data[1] + data[0] / 256) # Index 0 (green low) and Index 1 (green high)
        self.red = data[3] + data[2] / 256 # Index 2 (red low) and Index 3 (red high)
        self.blue = (data[5] + data[4] / 256) # Index 4 (blue low) and Index 5 (bue high).
        tolerance_percent = self.CONSTANTS['COLOR_TOLERANCE_PERCENT']

        # Count average to ensure detection of undefined colors:
        avg = (self.red + self.green + self.blue) / 3

        # Avoid division by 0:
        if avg == 0:
            self.get_logger().warn('Error reading equals 0')
            return

        # Calculation of difference between the colors:
        diff_r = 1.25 * (abs(self.red - avg) / avg * 100)
        diff_g = abs(self.green - avg) / avg * 100
        diff_b = (abs(self.blue - avg) / avg * 100)

        # Checking the different conditions and printing the color:
        if diff_r < tolerance_percent and diff_g < tolerance_percent and diff_b < tolerance_percent:
            # Setting victim reading to 0:
            self.victim_readings = 0
        elif self.red > (self.green * 1.07) and self.red > (self.blue * 1.07) and self.red > 40:
            # Incrementing victim count:
            self.victim_readings += 1
            # If scanned victims count exceed the threadhold  then start the threading:
            if self.victim_readings > self.CONSTANTS['VICTIM_READING_THRESHOLD']:
                # Starting the threading
                threading.Thread(target=self.victim_counter).start()
        elif self.green > self.red and self.green > self.blue:
            # Setting victim to 0:
            self.victim_readings = 0
        else:
            # Setting victim to 0:
            self.victim_readings = 0

    def timer_callback(self):
        # Record the start time when this function is first called
        if self.start_time is None:
            self.start_time = self.get_clock().now()

        current_time = self.get_clock().now()
        elapsed_time = (current_time - self.start_time).nanoseconds * 1e-9

        # If time exceeds run_duration, stop and shut down
        if elapsed_time >= self.run_duration:
            self.stop_and_shutdown()
            return

        # Otherwise, do normal obstacle detection
        if self.has_scan_received:
            self.detect_obstacle()

        # Call the function to start reading and updating colour values
        self.getAndUpdateColour()

    def detect_obstacle(self):
        # Filtering the readings:
        # Looping through the array and substituing each 0 reading with 3.5
        for i in range(len(self.scan_ranges)):
            if self.scan_ranges[i] < self.CONSTANTS['MIN_SCAN_DISTANCE']:
                self.scan_ranges[i] = self.CONSTANTS['DEFAULT_DISTANCE']
        
        # Define cone indices for 360° scan (each cone is x/20 of the circle)
        cone_225_deg = int(len(self.scan_ranges) * 22.5/360)  # 25° position
        cone_18_deg = int(len(self.scan_ranges) * 1/20)   # 18°
        cone_54_deg = int(len(self.scan_ranges) * 3/20)   # 54°
        cone_90_deg = int(len(self.scan_ranges) * 5/20)   # 90°
        cone_270_deg = int(len(self.scan_ranges) * 15/20) # 270°
        cone_306_deg = int(len(self.scan_ranges) * 17/20) # 306°
        cone_342_deg = int(len(self.scan_ranges) * 19/20) # 342°
        cone_3375_deg = int(len(self.scan_ranges) * 337.5/360) # 360°

        # Calculate minimum distances in each cone
        front_cone = min(min(self.scan_ranges[cone_3375_deg:]), min(self.scan_ranges[:cone_225_deg]))

        small_front_cone = min(min(self.scan_ranges[cone_342_deg:]), min(self.scan_ranges[:cone_18_deg]))
        front_left_cone = min(self.scan_ranges[cone_18_deg:cone_54_deg])
        front_left_mid_cone = min(self.scan_ranges[cone_54_deg:cone_90_deg])
        front_right_mid_cone = min(self.scan_ranges[cone_270_deg:cone_306_deg])
        front_right_cone = min(self.scan_ranges[cone_306_deg:cone_342_deg])

        # Decision making variable to decide whether to turn left or right:
        turn_left = front_left_cone
        turn_right = front_right_cone

        # Special case: U-shaped pocket:
        u_pocket_escape_values = [front_cone, front_left_cone, front_left_mid_cone, front_right_cone, front_right_mid_cone]

        # Check for collisions in front cones
        collision_cones = min(front_cone, front_left_cone, front_right_cone)

        #collision counter thread function call
        if collision_cones < self.CONSTANTS['POCKET_THRESHOLD'] and self.tele_twist.linear.x > 0:
            # Starting the thread to register one collusion only
            prediction_time = abs(front_cone / self.tele_twist.linear.x)
            if prediction_time < self.CONSTANTS['COLLISION_THRESHOLD']:
                # Start threading of collision state:
                threading.Thread(target=self.collision_counter).start()
        
        twist = Twist()
        
        # Calculate angular velocity for turning
        if front_cone > 0.15: # Avoid division by zero
            # Special case: If the robot is stuck in a U-shaped pocket it needs to turn around really fast:
            if all(value < self.CONSTANTS['POCKET_THRESHOLD'] for value in u_pocket_escape_values):
                # Turn really fast:
                self.tele_twist.angular.z = 2.0
            # Special case 2 (Straight wall case):
            elif front_cone < self.CONSTANTS['POCKET_THRESHOLD'] and twist.linear.x < 0.05:
                # Case if we hit a straight wall then we turn fast with the determined turning direction:
                self.tele_twist.angular.z = (self.tele_twist.angular.z / abs(self.tele_twist.angular.z)) * 1.5
            # Wheel protection case to the left:
            elif front_left_cone < self.CONSTANTS['WHEEL_PROTECTION_THRESHOLD'] and front_left_mid_cone > self.CONSTANTS['WHEEL_PROTECTION_THRESHOLD']:
                self.tele_twist.angular.z = -1.0
            # Wheel protection case to the right:
            elif front_right_cone < self.CONSTANTS['WHEEL_PROTECTION_THRESHOLD'] and front_right_mid_cone > self.CONSTANTS['WHEEL_PROTECTION_THRESHOLD']:
                self.tele_twist.angular.z = 1.0
            # If we need the robot to turn right we will calculate the angular velocity to be negative:
            elif turn_right > turn_left:
                # Linear regrssion analysis
                self.tele_twist.angular.z = (-1) * (-14.0948 * front_cone**3 + 15.3461 * front_cone**2 - 8.9312 * front_cone + 2.1339)
            else:
                # Linear regrssion analysis
                self.tele_twist.angular.z = (-14.0948 * front_cone**3 + 15.3461 * front_cone**2 - 8.9312 * front_cone + 2.1339)
        else:
            self.tele_twist.angular.z = 1.5  # Default turn rate if too close

        # Obstacle distance narrowed down to front cone:
        obstacle_distance = front_cone

        # Adjust velocity based on obstacle distance
        if obstacle_distance < self.stop_distance:
            twist.linear.x = 0.0
            twist.angular.z = self.tele_twist.angular.z
        elif small_front_cone < self.turn_distance or obstacle_distance < 0.25:
            # Slower forward. Adjust the angular velocity subtracted to modify:
            #Linear regrssion analysis
            twist.linear.x = -0.3064 * front_cone**2 + 0.6223 * front_cone - 0.0188
            # If linear speed is very low we will multiply the turning speed by a factor of x.
            if twist.linear.x < 0.07:
                # Multiplying the angular speed with a factor of x:
                twist.angular.z = 4*self.tele_twist.angular.z
            else:
                twist.angular.z = self.tele_twist.angular.z
        else:
            twist.linear.x = 0.2
            twist.angular.z = 0.0

        self.speed_updates += 1
        self.speed_accumulation += twist.linear.x

        # Publishing the twist
        self.cmd_vel_pub.publish(twist)
    
    def average_speed_calculation(self):
        if self.speed_updates > 0:
            return self.speed_accumulation / self.speed_updates
        else:
            return 0.0
        
    def stop_and_shutdown(self):
        # Publish zero velocity to stop the robot
        stop_twist = Twist()
        stop_twist.linear.x = 0.0
        stop_twist.angular.z = 0.0
        self.cmd_vel_pub.publish(stop_twist)

        # Print the average speed
        avg_speed = self.average_speed_calculation()
        self.get_logger().info(f'The average speed was {avg_speed: } m/s over {self.run_duration} seconds.')
        self.get_logger().info(f'Total collisions registered: {self.collision_count}')
        self.get_logger().info(f'Amount of victims found: {self.victims_counted}')

        # Now shut down
        rclpy.shutdown()
        
def main(args=None):

    rclpy.init(args=args)
    turtlebot3_obstacle_detection = Turtlebot3ObstacleDetection()
    rclpy.spin(turtlebot3_obstacle_detection)
    turtlebot3_obstacle_detection.destroy_node()

if __name__ == '__main__':
    main()