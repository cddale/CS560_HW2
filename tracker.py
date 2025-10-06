import rclpy
from rclpy.node import Node
import time
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class Tracker(Node):
    def __init__(self):
        super().__init__('Track')
        self.startTime = time.time()
        self.time = 0
        self.maxDistance = 0
        self.current_distance = 0
        self.isStart = False
        self.startPos = [0, 0]
        self.subscription = self.create_subscription(
            Odometry,
            '/ground_truth',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        if self.isStart == False:
            self.startPos = [x, y]
            self.isStart = True

        dx = x - self.startPos[0]
        dy = y - self.startPos[1]
        distanceTraveled = math.sqrt(dx * dx + dy * dy)

        curTime = time.time()
        elapsed = curTime - self.startTime

        print("Elapsed Time: " + str(elapsed))
        print("Distance: " + str(distanceTraveled))
    
def main(args=None):
    rclpy.init(args=args)
    tracker_node = Tracker()
    rclpy.spin(tracker_node)
    tracker_node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()