import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class ObstacleMover(Node):

    def __init__(self):
        super().__init__('Obstacle_Mover')
        # Publisher for obstacle movement commands
        self.obstacle_pub = self.create_publisher(Twist, '/moving_box/cmd_vel', 10)
       
        # Subscriber for obstacle location
        self.obstacle_odom_sub = self.create_subscription(Odometry, '/moving_box/odom', self.odom_callback, 10)
       
        self.factor = 1.0
        self.y = 0
       
        # Timer for obstacle movement
        self.timer_ = self.create_timer(0.01, self.obstacle_mover)
        
    def odom_callback(self, msg):
        self.y = msg.pose.pose.position.y
    
    def obstacle_mover(self):
        twist_msg = Twist()
        if 0 < self.y < 1.5:
            self.factor = -1.0
        elif -4.0 > self.y > -4.5:
            self.factor = 1.0
        twist_msg.linear.y = self.factor * 0.5
        self.obstacle_pub.publish(twist_msg)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ObstacleMover())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
