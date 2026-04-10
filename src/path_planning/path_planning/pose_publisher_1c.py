from path_planning import config
from path_planning.rrtsharp_c import RRTSharp, error
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from  std_msgs.msg import Bool
from path_planning.path_to_qr import path_to_qr
from path_planning.path_pruning import fit_to_qr
from path_planning.astar import Astar
import numpy as np
import os

class PathPublisher(Node):

    def __init__(self):
        super().__init__('path_publisher_node')

        self.publisher_ = self.create_publisher(PoseStamped, 'pose', 10)

        timer_period = 0.5 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.odom = None

        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )

        # map msg details
        self.map_data = None # flat list of cell values
        self.map_width = None
        self.map_height = None
        self.map_resolution = None
        self.map_origin_x = None
        self.map_origin_y = None

        self.pose_msg = None

        # subscribe to at_goal and then generate qr
        self.at_pose_sub = self.create_subscription(
            Bool,
            '/at_pose',
            self.pose_callback,
            10
        )

        self.at_pose = True # Initial pose is start 

        self.done = False # if done don't sample anymore
        self.saved = False # if saved once don't resave 

        # goal in world scale
        self.world_goal = (config.GOAL[0]*config.WORLD_SCALE, config.GOAL[1]*config.WORLD_SCALE)

        self.visited_frontiers = set()

    def timer_callback(self):

        if self.at_pose and self.done and not self.saved:
            
            # get inflated obstacles - one last time for redundancy
            inflated_map_data = self._inflate_map(self.map_data, self.map_width, self.map_height, self.radius)
            self.inflated_map = OccupancyGrid()
            self.inflated_map.data = inflated_map_data  
            self.inflated_map.info.width = self.map_width
            self.inflated_map.info.height = self.map_height
            self.inflated_map.info.resolution = self.map_resolution
            self.inflated_map.info.origin.position.x = self.map_origin_x
            self.inflated_map.info.origin.position.y = self.map_origin_y

            path = self._get_path(self.inflated_map)

            # save to qr 
            path_str = fit_to_qr(path=path, 
                                obstacles=self.obstacles, 
                                e=self.e,
                                step_size=config.STEP_SIZE,
                                char_limit=config.CHAR_LIMIT)
            
            self.get_logger().info(f"Saving path as {path_str}\n")
            
            base_dir = os.path.abspath(os.path.dirname(__file__)) # reads share dir 
            root_dir = os.path.join(base_dir, '..', '..', '..', '..', '..', '..')
            qr_dir = os.path.join(root_dir, 'src', 'path_planning', 'qrcodes')
            
            path_to_qr(path=path_str, output_dir=qr_dir, env_number=self.qr_num)

            self.saved = True

        # if not done sample
        if self.at_pose and not self.done or self.pose_msg is None:
            self.get_logger().info("SAMPLING POSE")
            self.pose_msg = self.sample()
            self.at_pose = False

        # get pose and publish it to 
        if self.pose_msg:
            self.get_logger().info("SENDING POSE")
            self.publisher_.publish(self.pose_msg)

    def odom_callback(self, odom_msg):
        self.odom = odom_msg # just stores odom msg
        self.odom_x = self.odom.pose.pose.position.x
        self.odom_y = self.odom.pose.pose.position.y
        self.odom_theta = self._quaternion_to_yaw(self.odom.pose.pose.orientation)

    def _get_path(self, map):
        """
        Finds path using RRT#, converts to nav_msgs.msg.Path for publishing.

        Returns
        -------
        path : list
            List of points from RRT#.
        """

        error_matrix = np.zeros((config.ENV_X_BOUNDS[1] * config.WORLD_SCALE, config.ENV_Y_BOUNDS[1] * config.WORLD_SCALE))
        e_env = 0.0
        e = error(e_env, error_matrix)

        rrt = RRTSharp(start = config.START * config.WORLD_SCALE, 
                       goal = self.world_goal, 
                       bounds = config.BOUNDS * config.WORLD_SCALE, 
                       obstacles=map,
                       e = e,
                       )
        
        path, nodes, self.e = rrt.rrt_sharp()
        return path
    
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

    def _quaternion_to_yaw(self, q):
        """
        Convert quaternion to heading.
        """
        theta = np.arctan2(
            2.0 * (q.w*q.z + q.x*q.y), 
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            )
        return theta

    def pose_callback(self, at_pose_msg):
        self.get_logger().info("IN POSE CALLBACK")
        if at_pose_msg.data == True:
            self.get_logger().info("CHANGING AT POSE")
            self.at_pose = True
        return 

    def map_callback(self, msg):

        # just stores map data
        self.map_data = msg.data  # flat list of cell values
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_resolution = msg.info.resolution
        self.map_origin_x = msg.info.origin.position.x
        self.map_origin_y = msg.info.origin.position.y
        self.radius = int(config.BUFFER / self.map_resolution)

    def sample(self):
        if self.map_data is None:
            self.get_logger().warn("No map data\n")
            return None
        if self.odom is None:
            self.get_logger().warn("No odom\n")
            return None

        # inflate once with latest map data
        inflated = self._inflate_map(
            self.map_data, self.map_width, self.map_height, self.radius)
        
        self.inflated_map = self._build_occupancy_grid(inflated)

        # find candidates
        candidates = self._get_candidates(inflated)

        if not candidates:
            self.get_logger().info('NO CANDIDATES\n')
            return None

        candidates.sort(key=lambda c: c[0])

        # try each candidate until A* succeeds
        for chosen_dist, cx, cy in candidates:
            self.get_logger().info("Getting path\n")
            path = self._plan_path(cx, cy)
            self.get_logger().info("Found path\n")
            if path is None or len(path) < 2:
                continue

            waypoint = self._prune_path(path)
            if waypoint is None:
                continue

            if chosen_dist <= 0.1:
                self.done = True

            return self._point_to_pose(waypoint[0], waypoint[1])
        
        return None

    def _get_candidates(self, inflated):
        candidates = []
        
        # frontier cells first
        for i, cell in enumerate(inflated):
            if cell != 0:
                continue
            row = i // self.map_width
            col = i % self.map_width
            if not self._is_frontier(row, col):
                continue
            x, y = self._grid_to_world(row, col)
            dist_to_robot = np.sqrt((x-self.odom_x)**2 + (y-self.odom_y)**2)
            if dist_to_robot < 0.3:
                continue
            dist_to_goal = np.sqrt((self.world_goal[0]-x)**2 + (self.world_goal[1]-y)**2)
            candidates.append((dist_to_goal, x, y))

        if candidates:
            return candidates
        
        self.get_logger().info("NO INITIAL CANDIDATES\n")

        # fallback: unknown cells
        for i, cell in enumerate(self.map_data):
            if cell != -1:
                continue
            row = i // self.map_width
            col = i % self.map_width
            x, y = self._grid_to_world(row, col)
            dist_to_robot = np.sqrt((x-self.odom_x)**2 + (y-self.odom_y)**2)
            if dist_to_robot < 0.3:
                continue
            dist_to_goal = np.sqrt((self.world_goal[0]-x)**2 + (self.world_goal[1]-y)**2)
            candidates.append((dist_to_goal, x, y))

        return candidates

    def _plan_path(self, goal_x, goal_y):
        start_row, start_col = self._world_to_grid(self.odom_x, self.odom_y)
        start_row = max(0, min(start_row, self.map_height - 1))
        start_col = max(0, min(start_col, self.map_width - 1))
        sx, sy = self._grid_to_world(start_row, start_col)
        astar = Astar((sx, sy), (goal_x, goal_y), self.inflated_map)
        return astar.astar()

    def _prune_path(self, path):
        waypoint = path[1]
        for i in range(2, len(path)):
            if self._is_collision_free(
                from_point=(self.odom_x, self.odom_y),
                to_point=(path[i][0], path[i][1])
            ):
                waypoint = path[i]
            else:
                break
        return waypoint

    def _build_occupancy_grid(self, data):
        grid = OccupancyGrid()
        grid.data = data
        grid.info.width = self.map_width
        grid.info.height = self.map_height
        grid.info.resolution = self.map_resolution
        grid.info.origin.position.x = self.map_origin_x
        grid.info.origin.position.y = self.map_origin_y
        return grid
        
    def _is_frontier(self, row, col):
        # a free cell is a frontier if it has at least one unknown neighbor
        for dr, dc in [(-1,0),(1,0),(0,-1),(0,1)]:
            nr, nc = row + dr, col + dc
            if 0 <= nr < self.map_height and 0 <= nc < self.map_width:
                if self.map_data[nr * self.map_width + nc] == -1:
                    return True
        return False

    def _point_to_pose(self, x, y):

        pose = PoseStamped()

        # header stuff
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()

        # fill out x y z
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0 

        # fill out x y z w

        theta = np.arctan2(y - self.odom_y, x - self.odom_x + 1e-8)

        q = self._yaw_to_quaternion(theta)

        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]

        return pose
    
    def _is_collision_free(self, from_point: Node, to_point: Node, explore: bool=False):
        """
        Implements bresenham line algorithm to detect collisons

        TODO add docstrings
        """

        x0, y0 = from_point[0], from_point[1]
        x1, y1 = to_point[0], to_point[1]
        dx = x1 - x0
        dy = y1 - y0
        dist = np.sqrt(dx*dx + dy*dy)

        steps = int(dist / self.map_resolution)

        for i in range(steps + 1): # break line into many pieces
            t = i / steps if steps > 0 else 0
            x = x0 + t * dx
            y = y0 + t * dy

            # if any line is out of bounds or intersects ubstacle / unknown
            row, col = self._world_to_grid(x, y)
            if not 0 <= row < self.map_height or not 0 <= col < self.map_width:
                return False
            
            cell_value = self.get_cell(row, col, self.inflated_map.data)
            if explore:
                if cell_value == 100:
                    return False
            else:
                if cell_value == 100 or cell_value == -1:
                    return False

        return True

    def _world_to_grid(self, x, y):

        # converts point from map to location in grid
        col = int((x - self.map_origin_x) / self.map_resolution)
        row = int((y - self.map_origin_y) / self.map_resolution)
        return row, col
    
    def _grid_to_world(self, row, col):
        # converts grid to location
        x = col * self.map_resolution + self.map_origin_x
        y = row * self.map_resolution + self.map_origin_y
        return x, y
    
    def get_cell(self, row, col, map=None):

        if map is None:
            map = self.map_data
        # gets cell data from grid
        return map[row * self.map_width + col]
    
    def _inflate_map(self, map_data, width, height, radius):

        inflated = list(map_data) # init inflated list

        for i, cell in enumerate(map_data):
            if cell == 100: # if obstacle

                row = i // width
                col = i % width

                for dr in range(-radius, radius + 1):
                    for dc in range(-radius, radius + 1):

                        if dr**2 + dc**2 <= radius**2: # inflate neighboring cells
                            nr, nc = row + dr, col + dc
                            if 0 <= nr < height and 0 <= nc < width:
                                inflated[nr * width + nc] = 100

        return inflated

def main(args=None):
    rclpy.init(args=args)
    path_publisher = PathPublisher()
    rclpy.spin(path_publisher)
    path_publisher.destroy_node()
    rclpy.shutdown()

if __name__=="__main__":
    main()