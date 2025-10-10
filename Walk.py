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
        self.g = 1
        self.K_P = 10
        self.K_I = 0
        self.K_D = 0
        self.start = datetime.now()

        self.cmd_pub = self.create_publisher(Twist,'/cmd_vel', 10)
        self.whisker = 0
        self.e_prev = self.g - self.whisker
        self.e_sum = 0
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.linear_speed = 0.8
        self.move_cmd = Twist()
        self.move_cmd.linear.x = self.linear_speed
        self.subscription = self.create_subscription(
			LaserScan,
			'/base_scan',
			self.sensor_callback,
			10)

    def sensor_callback(self, msg): #make this more robust for repersentation
        middle_sensor = int(len(msg.ranges) / 2)
        left_sensor = int(len(msg.ranges) / 5)
        right_sensor = int(len(msg.ranges) / 5) * 4

        middle = msg.ranges[middle_sensor]
        middle_sector = msg.ranges[left_sensor : right_sensor]
        min_point = min(middle_sector)
        
        print("Sensor: " + str(self.whisker) + "; U(t): " + str(self.move_cmd.angular.z))
        
        if min_point < 1:
            self.whisker = ((sum(middle_sector) / (right_sensor-left_sensor)) * .25) + min_point * .75
        else:
            self.whisker = ((sum(middle_sector) / (right_sensor-left_sensor)) * .80) + middle * 0.2

		
    def forward(self):
	    self.move_cmd.linear.x = self.linear_speed 

    def timer_callback(self): #change this to use the PID formula (uses information from sensor_callback)
        normal_range = 2500
        self.e = self.g - self.whisker

        timeDiff = (datetime.now() - self.start).total_seconds()

        self.e_sum = self.e_sum + self.e  * ((self.e - self.e_prev) / max(timeDiff,0.01))

        dedt = (self.e - self.e_prev) / (max(self.e - self.e_prev, 0.01) / max(timeDiff, 0.01))

        u = self.K_P * self.e + self.K_I * self.e_sum + self.K_D * dedt
        
        self.move_cmd.angular.z = u

        if self.whisker < 1:
            self.move_cmd.linear.x = 0.1
        else:
            self.move_cmd.linear.x = 0.8
        #self.move_cmd.angular.z = ((u + normal_range) / (normal_range + normal_range)) * (5 + 5) + -5
        
        self.e_prev = self.e

        self.cmd_pub.publish(self.move_cmd) 

        
def main(args=None):
    rclpy.init()
    walk_node = Walk()
    rclpy.spin(walk_node)
    walk_node.destroy_node()
    rclpy.shutdown()

