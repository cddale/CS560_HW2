import rclpy
from rclpy.node import Node
from datetime import datetime
import math
import np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class Walk(Node):
    def __init__(self):
        super().__init__('walk')
        #PID values
        self.g = 2.5
        self.K_P = 5
        self.K_I = 4
        self.K_D = 2
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
        #left_sensor = int(len(msg.ranges) / 4)
        #right_sensor = int(len(msg.ranges) / 4) * 3

        #left = msg.ranges[left_sensor]
        #right = msg.ranges[right_sensor]
        middle_sector = msg.ranges[middle_sensor - 25 : middle_sensor + 25]
        weights = np.hanning(len(middle_sector))  # bell curve for weights
        self.whisker = np.average(middle_sector, weights=weights)
        
        print("Sensor: " + str(self.whisker) + "; U(t): " + str(self.move_cmd.angular.z))
        #self.whisker = (right + left)/2
		
    def forward(self):
	    self.move_cmd.linear.x = self.linear_speed 

    def timer_callback(self): #change this to use the PID formula (uses information from sensor_callback)
        self.e = self.g - self.whisker

        timeDiff = datetime.now() - self.start

        self.e_sum = self.e_sum + self.e  * ((self.e - self.e_prev) / max(timeDiff.total_seconds(),0.01))

        dedt = (self.e - self.e_prev) / (max(self.e - self.e_prev, 0.01) / max(timeDiff.total_seconds(), 0.01))

        u = self.K_P * self.e + self.K_I * self.e_sum + self.K_D * dedt
        self.move_cmd.angular.z = ((u + 3000) / (3000 + 3000)) * (5 + 5) + -5
        
        self.e_prev = self.e

        self.cmd_pub.publish(self.move_cmd) 

        """ if(self.whisker < 2.0):
            self.move_cmd.angular.z = 2.0
            self.move_cmd.linear.x = 0.2
        else:
            self.move_cmd.angular.z = 0.0
            self.move_cmd.linear.x = 0.8
        self.cmd_pub.publish(self.move_cmd) 
		 """
def main(args=None):
    rclpy.init()
    walk_node = Walk()
    rclpy.spin(walk_node)
    walk_node.destroy_node()
    rclpy.shutdown()

