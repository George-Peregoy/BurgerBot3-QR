import numpy as np
import heapq

class Node:

    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None
        self.cost = np.inf
        self.g = np.inf

    def __lt__(self, other):
        return self.cost < other.cost

class Astar:
     
    def __init__(self, start, goal, map):
        
        self.start_node = Node(start[0], start[1])
        self.goal_node = Node(goal[0], goal[1])
        
        self.map_data = map.data  # flat list of cell values
        self.map_width = map.info.width
        self.map_height = map.info.height
        self.map_resolution = map.info.resolution
        self.map_origin_x = map.info.origin.position.x
        self.map_origin_y = map.info.origin.position.y

    def astar(self):

        queue = [] # init queue
        heapq.heappush(queue, self.start_node)
        
        visited = set()
        self.start_node.cost = self._euclidean_distance(self.goal_node, self.start_node)
        self.start_node.g = 0

        # get surrouding cells, exclude 0,0
        neighbors = [(i, j) for i in range(-1, 2) for j in range(-1, 2) if (i != 0 or j != 0)]

        while queue:
            
            # prioritize lowest cost
            node = heapq.heappop(queue)
            row, col = self._world_to_grid(node.x, node.y)
            visited.add((row, col))

            if self._euclidean_distance(node, self.goal_node) < 0.1:
                path = self.get_path(node) # if at end backtrack 
                return path

            # convert to grid
            row, col = self._world_to_grid(node.x, node.y)

            # add in good neighbor cells
            for dr, dc in neighbors:    
                nr, nc = row + dr, col + dc

                # convert back to coords
                x, y = self._grid_to_world(nr, nc)

                cand_node = Node(x, y)
                
                # make sure plausible
                valid = True
                if not 0 <= nr < self.map_height or not 0 <= nc < self.map_width:
                    valid = False

                if (nr, nc) in visited:
                    valid = False

                if not self._is_collision_free(node, cand_node):
                    valid = False

                if valid:
                    cand_node.parent = node
                    move_cost = np.sqrt(2) if dr != 0 and dc != 0 else 1.0
                    cand_node.g = node.g + move_cost
                    cand_node.cost = cand_node.g + self._euclidean_distance(self.goal_node, cand_node)
                    heapq.heappush(queue, cand_node)

        return 

    def get_path(self, goal_node: Node):
        
        node = goal_node
        path = []
        while node.parent is not None:
            path.append((node.x, node.y))
            node = node.parent

        path.append((node.x, node.y)) # append start node

        return path[::-1]


    def _is_collision_free(self, from_node: Node, to_node: Node):
            """
            Implements bresenham line algorithm to detect collisons

            TODO add docstrings
            """

            row, col = self._world_to_grid(to_node.x, to_node.y)
            if not 0 <= row < self.map_height or not 0 <= col < self.map_width:
                return False
            return self.get_cell(row, col) == 0

    def _euclidean_distance(self, node1: Node, node2: Node):
            """
            Computes euclidean distance

            Parameters
            ----------
            node1 : Node
                First Node.
            node2 : Node
                Second Node.

            Returns
            -------
            euclidean_distance : float.
            """
            return np.sqrt((node1.x - node2.x)**2 + (node1.y - node2.y)**2)
    
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

if __name__=="__main__":
    start = (1, 1)
    goal = (5, 5)
    map = [0]
    a = Astar(start, goal, map)