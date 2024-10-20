import rclpy
from rclpy.time import Time
from rclpy.node import Node
from rclpy.logging import LoggingSeverity
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor

class Handler(Node):
    def __init__(self):
        super().__init__("obstacle_avoidance_node") #Init node
        self.get_logger().set_level(LoggingSeverity.DEBUG)
        self.th_front = self.th_side = 1.5
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10) #Publisher
        self.scan_pub = self.create_subscription(LaserScan, "/scan", self.scan_callback, 10) #Laser Scan Subscriber

    def rotate(self, clockwise=True):
        factor = -1 if clockwise else 1
        msg = Twist()
        msg.angular.z = 1.0 * factor #rad/s
        self.cmd_pub.publish(msg)

    def go(self):
        msg = Twist()
        msg.linear.x = 0.31 #m/s
        self.cmd_pub.publish(msg)

    def scan_callback(self, msg: LaserScan):
        front = msg.ranges[157] #Frontal lidar data
        right = msg.ranges[0] #Right lidar data
        left = msg.ranges[314] #Left lidar data
        self.get_logger().debug(f"front: {front}\nright: {right}\nleft: {left}")
        if front > self.th_front and right > self.th_side and left > self.th_side:
            self.go()
        else:
            self.rotate()


def main():
    rclpy.init()
    param = rclpy.Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True) #Creating 'use_sim_time' node parameter
    executor = SingleThreadedExecutor()
    handller = Handler()
    handller.set_parameters([param]) #Setting 'use_sim_time' node parameter
    executor.add_node(handller) #Adding node to executor

    try:
        executor.spin() #Running loop - bocking call
    except KeyboardInterrupt:
        pass

    handller.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()