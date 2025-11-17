import rclpy
from rclpy.node import Node
from std_msgs.msg import String
#from sensor_msgs.msg import LaserScan
#from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
import time

class Locomotion(Node):

    def __init__(self):
        super().__init__('turtlebot3_locomotion')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            qos_profile_sensor_data)
        qos = QoSProfile(depth=10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 1)
        self.subscription  # prevent unused variable warning
        self.do_360 = True
        self.turn_360_time = 2.764 - 0.65
        self.turn_deadline = time.time() + self.turn_360_time

    def move(self, x, z):
        t = Twist()
        t.linear.x = x
        t.angular.z = z
        self.pub.publish(t)

    def log_message(self, msg):
        self.get_logger().info(str(msg))
    
    def listener_callback(self, msg):
        turn_speed = 2.272
        drive_speed = 0.176
        
        laser_count = len(msg.ranges)
        for i in range(0, laser_count):
            if msg.ranges[i] == 0.0:
                msg.ranges[i] = 5.0

        middle_distance = min(msg.ranges[0], msg.ranges[-1])

        if middle_distance < 0.75:
            self.move(0.0, turn_speed)
            return

        if time.time() >= self.turn_deadline:
            self.do_360 = not self.do_360
            if self.do_360:
                self.turn_deadline = time.time() + self.turn_360_time * 2
            else:
                self.turn_deadline = time.time() + 10

        if self.do_360:
            self.move(0.0, turn_speed / 2)
        else:
            self.move(drive_speed, 0.0)

def main(args=None):
    rclpy.init(args=args)

    tb3_locomotion = Locomotion()

    rclpy.spin(tb3_locomotion)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
