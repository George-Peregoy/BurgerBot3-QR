import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import numpy as np

class RobotController(Node):

    def __init__(self):
        super().__init__('robot_controller_node')

        # list of path poses
        self.path_subscription = self.create_subscription(
            PoseStamped,
            'pose',
            self.listener_callback,
            10
        )

        # get current pose from odom
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # publish if at goal pose
        self.at_pose_publisher = self.create_publisher(
            Bool, 
            '/at_pose', 
            10
        )
        
        self.pose = None
        self.odom = None
        self.at_pose = False

        # publish 'cmd_vel'
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        
        timer_period = 0.5 # 2hz (may increase later - lets me see begining logs)
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.epsilon = 0.1 # 10 cm

        # coefficients for control
        self.kp = 0.8
        self.k_theta = 2.5

    def listener_callback(self, msg):
        
        self.pose = msg.pose
        self.at_pose = False
        self.get_logger().info("RECEIVED POSE")
        
    def odom_callback(self, odom_msg):
        self.odom = odom_msg # just stores odom msg
            
    def timer_callback(self):
        if self.odom is None:
            self.get_logger().warn("No odom yet")
            return
        
        if self.pose is None:
            self.get_logger().warn("No pose yet")
            return
        
        if self.at_pose == True:
            vel_msg = Twist() # if done do nothing
            self.publisher_.publish(vel_msg)
            return
 
        # get current pose
        odom_x = self.odom.pose.pose.position.x
        odom_y = self.odom.pose.pose.position.y
        odom_theta = self._quaternion_to_yaw(self.odom.pose.pose.orientation)
        
        self.get_logger().info(f"Current Odom -> x: {odom_x} y: {odom_y} theta: {odom_theta}\n")

        # get desired pose
        goal_x = self.pose.position.x
        goal_y = self.pose.position.y
        goal_theta = np.arctan2(goal_y - odom_y, goal_x - odom_x)

        self.get_logger().info(f"Current goal -> x: {goal_x} y: {goal_y} theta: {goal_theta}\n")

        # controller stuff
        position_error = np.sqrt((goal_x - odom_x)**2 + (goal_y - odom_y)**2)
        heading_error = goal_theta - odom_theta

        # normalize to [-pi, pi]
        while heading_error > np.pi:
            heading_error -= 2*np.pi
        while heading_error < -np.pi:
            heading_error += 2*np.pi

        self.get_logger().info(f"Heading error: {heading_error}\n")

        # assign cmd_vel
        vel_msg = Twist()
        vel_msg.linear.x = np.clip(self.kp * position_error * np.cos(heading_error), 0, 0.15) # max translation vel is 0.22 m/s
        vel_msg.angular.z = np.clip(self.k_theta * heading_error, -2.0, 2.0) # max rotational vel is 2.84 rad/s

        self.get_logger().info(f"linear_vel: {vel_msg.linear.x}\n")
        self.get_logger().info(f"Angular_vel: {vel_msg.angular.z}\n")

        if position_error < self.epsilon:
            
            self.at_pose = True
            self.at_pose_msg = Bool()
            self.at_pose_msg.data = True
            vel_msg = Twist() # stop moving
            self.get_logger().info("At Pose")
            self.get_logger().info("SENDING AT POSE")
            self.at_pose_publisher.publish(self.at_pose_msg)
            return
 
        self.publisher_.publish(vel_msg)

    def _quaternion_to_yaw(self, q):
        """
        Convert quaternion to heading.
        """
        theta = np.arctan2(
            2.0 * (q.w*q.z + q.x*q.y), 
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            )
        return theta

def main(args=None):
    rclpy.init(args=args)
    path_publisher = RobotController()
    rclpy.spin(path_publisher)
    path_publisher.destroy_node()
    rclpy.shutdown()

if __name__=="__main__":
    main()