import rclpy
from rclpy.node import NODe
from geometre_msgs.msg import Twist
from sensor_msgs.msp import LaserScan

class Walk(Node):
    def __init__(self):
		#create publisher using twist (uses timer callback the same way sub uses sensor callback)
        self.timer = self.create_timer(""" needs some stuff """)
        self.linear_speed = 0.8
        self.move_cmd = Twist()
        self.move_cmd.linear.x = self.linear_speed
        self.subscription = self.create_subscription(
			LaserScan,
			'/base_scan',
			self.sensor_callback,
			10)

    def sensor_callback(self, msg): #make this more robust for repersentation
        middle_sensor = int(len(msg.ranges / 2))
        front = msg.ranges[middle_sensor]
        print("Sensor: " + str(front))
        self.whisker = front
		
    def forward(self):
	    self.move_cmd.linear.x = self.linear_speed 

    def timer_callback(self): #change this to use the PID formula (uses information from sensor_callback
        if(self.whisker < 2.0):
            self.move_cmd.angular.z = 2.0
        else:
            self.move_cmd.angular.s = 0.0
        self.cmd_pub.publish(self.move_cmd) 
		
def main(args=None):
    rclpy.init(arg=args)
    #didnt manage to get his main