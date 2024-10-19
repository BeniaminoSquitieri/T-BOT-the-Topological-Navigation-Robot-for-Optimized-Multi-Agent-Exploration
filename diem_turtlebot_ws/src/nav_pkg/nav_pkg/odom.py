import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class Handler(Node):
    def __init__(self):
        super().__init__("odom_node") #Init node
        self.sub_odom = self.create_subscription(Odometry, "/odom", self.odom_callback, 10)
        self.sub_vel = self.create_subscription(Twist, "/cmd_vel", self.vel_callback, 10)

    def odom_callback(self, msg: Odometry):
        self._log_info("ODOMETRY: Lin, Ang")
        lin = msg.twist.twist.linear.x
        ang = msg.twist.twist.angular.z
        text = "Linear: {}\t Angular: {}".format(lin, ang)
        self._log_info(text)
        self._log_info("COVARIANCE")
        text = "covariance: {}".format(msg.twist.covariance)
        self._log_info(text)
    
    def vel_callback(self, msg: Twist):
        self._log_info("COMMAND: Lin, Ang")
        lin = msg.linear.x
        ang = msg.angular.z
        text = "Linear: {}\t Angular: {}".format(lin, ang)
        self._log_info(text)
    
    def _log_info(self, text: str):
        self.get_logger().info(text)

def main():
    rclpy.init()
    handller = Handler()
    try:
        rclpy.spin(handller)
    except KeyboardInterrupt:
        pass
    handller.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()