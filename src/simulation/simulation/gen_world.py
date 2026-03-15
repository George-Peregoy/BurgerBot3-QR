from simulation.env_to_world import env_to_world
from path_planning.gen_obstacles import generate_obstacles
from path_planning import config
import os
import pickle


def main(num_worlds):

    num_obstacles = 8
    min_sides = 3
    max_sides = 4
    x_bounds = config.ENV_X_BOUNDS
    y_bounds = config.ENV_Y_BOUNDS
    max_area_ratio = 0.1
    max_side_length = 20

    base_dir = os.path.abspath(os.path.dirname(__file__))
    root_dir = os.path.join(base_dir, '..', '..')
    env_dir = os.path.join(root_dir, 'path_planning', 'environments')

    for i in range(num_worlds):

        file_name = os.path.join(env_dir, f'environment_polygon_{i}.pickle')

        obstacles = generate_obstacles(num_obstacles, min_sides, max_sides, \
                                       x_bounds, y_bounds, max_area_ratio, max_side_length)
        
        with open(file_name, 'wb') as f:
            pickle.dump(obstacles, f)
        
        env_to_world(i)
        
if __name__=="__main__":
    num_worlds = 10
    main(num_worlds=num_worlds)