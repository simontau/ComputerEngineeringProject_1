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

    def __init__(self):
        super().__init__('turtlebot3_obstacle_detection')
        print('TurtleBot3 Obstacle Detection - Auto Move Enabled')
        print('----------------------------------------------')
        print('stop angle: -90 ~ 90 deg')
        print('stop distance: 0.5 m')
        print('----------------------------------------------')

        self.run_duration = 60 # seconds
        self.start_time = None

        self.scan_ranges = []
        self.has_scan_received = False

        self.stop_distance = 0.2
        self.turn_distance = 0.4
        self.tele_twist = Twist()
        self.tele_twist.linear.x = 0.21
        self.tele_twist.angular.z = 0.0

        # Variables to calculate average speed:
        self.average_linear_speed = 0
        self.speed_updates = 0
        self.speed_accumulation = 0

        # Collusion counter variable:
        self.collision_count = 0
        self.collision_flag = False

        # Victim counter variable:
        self.victim_flag = False
        self.victim_reading_threshold= 4
        self.victim_readings = 0
        self.victims_counted = 0

        # Variable for LED light:
        # Led object (GPIO 18)
        self.led = LED(18)

        # Variables for LED sensor:
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
            print('A collision registered.')
            # Increment the counter by exactly 1 collision:
            self.collision_count += 1
            # Debounce periode
            time.sleep(2)
            self.collision_flag = False

    # Global thread victim counter function going into victim detection state:
    def victim_counter(self):
        if not self.victim_flag:
            self.led.on()
            print('We have registered one victim.')
            self.victim_flag = True
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
        self.data = self.bus.read_i2c_block_data(0x44, 0x09, 6)
        
        # Correctly convert the data to red, green, and blue int values
        self.green = 0.9 * (self.data[1] + self.data[0] / 256) # Index 0 (green low) and Index 1 (green high)
        self.red = self.data[3] + self.data[2] / 256 # Index 2 (red low) and Index 3 (red high)
        self.blue = (self.data[5] + self.data[4] / 256) # Index 4 (blue low) and Index 5 (blue high).

        # Tolerence percent to detect undefined colors:
        self.tolerance_percent = 12

        # Count average to ensure detection of undefined colors:
        self.avg = (self.red + self.green + self.blue) / 3

        # Avoid division by 0:
        if self.avg == 0:
            print('Error reading equals 0')
            return

        # Calculation of difference between the colors:
        self.diff_r = 1.25 * (abs(self.red - self.avg) / self.avg * 100)
        self.diff_g = abs(self.green - self.avg) / self.avg * 100
        self.diff_b = (abs(self.blue - self.avg) / self.avg * 100)

        # Checking the different conditions and printing the color:
        if self.diff_r < self.tolerance_percent and self.diff_g < self.tolerance_percent and self.diff_b < self.tolerance_percent:
            #print("Undefined color")
            # Setting victim to 0:
            self.victim_readings = 0
        elif (self.red + 1) > self.green and (self.red + 1) > self.blue:
            #print("Red")
            #print("RGB(%d, %d, %d)" % (self.red, self.green, self.blue))
            # Incrementing victim count:
            self.victim_readings += 1
            # If scanned victims count exceed the threadhold  then start the threading:
            if self.victim_readings > self.victim_reading_threshold:
                # Starting the threading
                threading.Thread(target=self.victim_counter).start()
        elif self.green > self.red and self.green > self.blue:
            #print("Green")
            # Setting victim to 0:
            self.victim_readings = 0
        else:
            #print("Blue")
            # Setting victim to 0:
            self.victim_readings = 0

        # Output data to the console as RGB values

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
            if self.scan_ranges[i] < 0.05:
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
        #front_test = min(min(self.scan_ranges[int(len(self.scan_ranges)*37/40):]), min(self.scan_ranges[:int(len(self.scan_ranges)*3/40)]))
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
        turn_left = front1v # min(front1v,front2v) Add this in the paranthesses if needed. We removed it to do some test
        turn_right = front1h # min(front1h) front2h) Add this in the paranthesses if needed. We removed it to do some test

        # Wheel protection to make sure we have no colissions:
        #wheel_protection = min(front1v, front1h)
        
        # Threshold variable for special cases:
        pocket_threshold = 0.2

        # Special case for first condition beneath:
        u_pocket_escape_values = [front, front1v, front2v, front1h, front2h]

        # Special case for second condition beneath:
        l_shape_escape_values_left = [front, front1h, front2h]

        # Special case for second condition beneath:
        l_shape_escape_values_right = [front, front1v, front2v]

        # Expanding the collision detection cones:
        collision_cones = min(front, front1v, front1h)

        #collition counter thread function call
        if collision_cones < 0.2 and self.tele_twist.linear.x > 0:
            # Starting the thread to register one collusion only
            prediction_number = abs(front / self.tele_twist.linear.x)
            #print(prediction_number)
            if prediction_number < 0.65:
                # Start threading of collision state:
                threading.Thread(target=self.collision_counter).start()
        
        
        twist = Twist()
        
        # Angular velocity calculation with safety check
        if front > 0.15: # Avoid division by zero
            # Special case: If the robot is stuck in a u-shaped pocket it needs to turn around really fast:
            if all(value < pocket_threshold for value in u_pocket_escape_values):
                # Turn really fast:
                self.tele_twist.angular.z = 2.0
            # Special case 2:
            elif all(value < pocket_threshold for value in l_shape_escape_values_left):
                self.tele_twist.angular.z = 1.5
            # Special case 3:
            elif all(value < pocket_threshold for value in l_shape_escape_values_right):
                self.tele_twist.angular.z = -1.5
            # Special case 4:
            elif front < pocket_threshold and twist.linear.x < 0.05:
                # Case if we hit a straight wall then we turn fast with the determined turning direction:
                print('Straight wall case')
                self.tele_twist.angular.z = (self.tele_twist.angular.z / abs(self.tele_twist.angular.z)) * 1.5
            # If we need the robot to turn right we will calculate the angular velocity to be negative:
            elif turn_right > turn_left:
                self.tele_twist.angular.z = (-1)*((np.pi / 2) * (self.tele_twist.linear.x / front) / 7) # Adjust the division here to modify the turning speed
            else:
                 self.tele_twist.angular.z = ((np.pi / 2) * (self.tele_twist.linear.x / front) / 7) # Adjust the division here to modify the turning speed
        else:
            self.tele_twist.angular.z = 1.5  # Default turn rate if too close

        # Testing. Narrowing down the cone in front. It works pretty good.
        obstacle_distance = front
        #print(f"p1 readings: {self.scan_ranges[p1]}")
        #print(f"p19 readings: {self.scan_ranges[p19]}")


        # Better logic flow with elif statements
        if obstacle_distance < self.stop_distance:
            twist.linear.x = 0.0
            twist.angular.z = self.tele_twist.angular.z
            #self.get_logger().info(f'Obstacle detected! Stopping. Distance: {obstacle_distance:.2f} m')
        elif obstacle_distance < self.turn_distance:
            # Slower forward. Adjust the angular velocity subtracted to modify:
            twist.linear.x = 0.31 - (abs(self.tele_twist.angular.z) *6/7 ) # Multiply by 6/7 to keep the original linear speed despite of new angular speed.
            # Making the turns happen faster:
            # If linear speed is very low we will multiply the turning speed by a factor of x.
            if twist.linear.x < 0.07:
                # Multiplying the angular speed with a factor of x:
                twist.angular.z = 4*self.tele_twist.angular.z
            else:
                twist.angular.z = self.tele_twist.angular.z
            #self.get_logger().info(f'Obstacle ahead, slowing. Distance: {obstacle_distance:.2f} m')
        else:
            twist.linear.x = 0.2
            twist.angular.z = 0.0

        print(twist.linear.x)

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
        print(f'The average speed was {avg_speed} m/s over {self.run_duration} seconds.')

        # Print the amount of collision:
        print(f'Total collisions registered: {self.collision_count}')

        # Print the amount of victims found:
        print(f'Amount of victims found: {self.victims_counted}')

        # Now shut down
        rclpy.shutdown()
        
def main(args=None):

    rclpy.init(args=args)
    turtlebot3_obstacle_detection = Turtlebot3ObstacleDetection()
    rclpy.spin(turtlebot3_obstacle_detection)
    turtlebot3_obstacle_detection.destroy_node()

if __name__ == '__main__':
    main()