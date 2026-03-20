from path_planning import config
from path_planning.rrtsharp import RRTSharp, error
from greedy_ell import prune_path
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from ament_index_python.packages import get_package_share_directory
import numpy as np
import pickle
import os

class PathPublisher(Node):

    def __init__(self):
        super().__init__('path_publisher_node')

        # get env_file from launch 
        self.declare_parameter('env_file', '')
        self.env_file = self.get_parameter('env_file').value


        self.publisher_ = self.create_publisher(Path, 'path', 10)
        timer_period = 0.5 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

        path_points = self._get_path()
        self.path_msg = self._path_to_poses(path_points)

    def timer_callback(self):
        self.publisher_.publish(self.path_msg)
        if len(self.path_msg.poses) > 0:
            start = self.path_msg.poses[0].pose.position
            goal = self.path_msg.poses[-1].pose.position
            self.get_logger().info(
                f'Publishing path: ({start.x:.2f}, {start.y:.2f}) -> ({goal.x:.2f}, {goal.y:.2f})'
            )

    def _get_path(self):
        """
        Finds path using RRT#, converts to nav_msgs.msg.Path for publishing.

        Returns
        -------
        path : list
            List of points from RRT#.
        """
        with open(self.env_file, 'rb')as f:
            obstacles = pickle.load(f)

        obstacles = [np.array(poly) for poly in obstacles]

        error_matrix = np.zeros((config.ENV_X_BOUNDS[1], config.ENV_Y_BOUNDS[1]))
        e_env = 0.0
        e = error(e_env, error_matrix)

        rrt = RRTSharp(start = config.START, 
                       goal = config.GOAL, 
                       bounds = config.BOUNDS, 
                       obstacles=obstacles,
                       e = e,
                       )
        
        path, nodes, e = rrt.rrt_sharp()

        """
        Pruning uses 
        prune_path(path, obstacles)
        score_nodes(short_path, e, step_size)
        greedy_ell(short_path, scores, char_limit)
        _build_qr_for_indicies(short_path, params['f1_idx'], params['f2_idx'], params['s'], char_limit)
        """


        return path

    def _path_to_poses(self, path_points):
        """
        Converts path from RRT# to nav_msgs.msgs.Path

        Parameters
        ----------
        path_points : list
            List of points from RRT#.

        Returns
        -------
        nav_path : Path
            nav_msgs.msg.Path object, contains array of PoseStamped.
        """

        # Path needs pose.orientation x|y|z|w
        # pose.postion x|y|z, set z=0 for all poses.

        path_msg = Path()
        path_msg.header.frame_id = 'map' # header and stamp needed for msg
        path_msg.header.stamp = self.get_clock().now().to_msg()

        # convert each point to PoseStamped
        for i in range(len(path_points)): 
            pose = PoseStamped()
            pose.header.frame_id = 'map' # header and stamp needed for msg
            pose.header.stamp = self.get_clock().now().to_msg()

            # assign position
            pose.pose.position.x = float(path_points[i][0]) * config.WORLD_SCALE # convert to world size
            pose.pose.position.y = float(path_points[i][1]) * config.WORLD_SCALE# must be float for msg
            pose.pose.position.z = 0.0 # always 0 for this case

            # assign orientation 
            if i < len(path_points) - 1:
                dx = path_points[i + 1][0] - path_points[i][0]
                dy = path_points[i + 1][1] - path_points[i][1]
                theta = np.arctan2(dy, dx)
            else:
                if i > 0:
                    # if last point, use orientation of previous point 
                    # doesn't matter where it's facing at end
                    dx = path_points[i][0] - path_points[i - 1][0]
                    dy = path_points[i][1] - path_points[i - 1][1]
                else:
                    theta = 0.0
                
            # convert to quaternion. (yaw=theta)
            q = self._yaw_to_quaternion(theta)
            pose.pose.orientation.x = q[0]
            pose.pose.orientation.y = q[1]
            pose.pose.orientation.z = q[2]
            pose.pose.orientation.w = q[3]

            # append to path 
            path_msg.poses.append(pose)

        return path_msg
    
    def _yaw_to_quaternion(self, yaw):
        """
        Convert yaw to quaternion (x, y, z, w)

        Parameters
        ----------
        yaw : float
            Yaw angle in radians.
        
        Returns
        x, y, z, w
        """
        return (
            0.0,
            0.0,
            np.sin(yaw / 2.0),
            np.cos(yaw / 2.0)
        )

def main(args=None):
    rclpy.init(args=args)
    path_publisher = PathPublisher()
    rclpy.spin(path_publisher)
    path_publisher.destroy_node()
    rclpy.shutdown()

if __name__=="__main__":
    main()