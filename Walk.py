import rclpy
from rclpy.node import Node
from datetime import datetime
import math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class Walk(Node):
    def __init__(self):
        super().__init__('walk')
        #PID values
        self.g = 0.3
        self.K_P = 100

        self.cmd_pub = self.create_publisher(Twist,'/cmd_vel', 10)
        self.left = 0
        self.right = 0
        self.e_prev = self.g - self.right
        self.e_sum = 0
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.linear_speed = 0.2
        self.move_cmd = Twist()
        self.move_cmd.linear.x = self.linear_speed
        self.subscription = self.create_subscription(
			LaserScan,
			'/base_scan',
			self.sensor_callback,
			10)

    def sensor_callback(self, msg):
        n = len(msg.ranges)
        middle_sensor = n // 2
        left_sensor = int(n * (180 / 270))
        right_sensor = 0
        left_sector = msg.ranges[middle_sensor : left_sensor]
        right_sector = msg.ranges[right_sensor : middle_sensor] #right sector is intentionally larger than left sector
        self.right = min(right_sector)
        self.left = min(left_sector)        

		
    def forward(self):
	    self.move_cmd.linear.x = self.linear_speed 

    def timer_callback(self): 
        if min(self.left, self.right) < self.g:
            self.e = self.g - self.right # force right when close to walls
        else: self.e = self.g - min(self.left, self.right)

        u = self.K_P * self.e
        
        if min(self.left, self.right) > 2*self.g:
             self.move_cmd.angular.z = 0.0
             self.move_cmd.linear.x = self.linear_speed
        elif min(self.left, self.right) < self.g:
             self.e = self.g - self.right
             self.move_cmd.linear.x = 0.05
             self.move_cmd.angular.z = u
        else:
             self.move_cmd.angular.z = u
             self.move_cmd.linear.x = 0.1

        self.cmd_pub.publish(self.move_cmd) 

        
def main(args=None):
    rclpy.init()
    walk_node = Walk()
    rclpy.spin(walk_node)
    walk_node.destroy_node()
    rclpy.shutdown()
