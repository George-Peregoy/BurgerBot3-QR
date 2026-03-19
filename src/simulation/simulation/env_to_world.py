from path_planning import config
from ament_index_python.packages import get_package_share_directory
import numpy as np
import pickle
import os
import glob

def env_to_world(world_number: int = 0):
    """
    Converts environment used in path planning to a world 
    file used for visualization in gazebo.

    Parameters
    ----------
    world_number : str
        World number which also corresponds to env number.
    """
    base_dir = os.path.abspath(os.path.dirname(__file__)) # reads share dir 
    root_dir = os.path.join(base_dir, '..', '..', '..', '..', '..', '..')
    env_dir = os.path.join(root_dir, 'src','path_planning', 'environments', \
                           f"environment_polygon_{world_number}.pickle")
    mesh_dir = os.path.join(root_dir, 'src', 'simulation', "meshes")
    world_dir = os.path.join(root_dir, 'src', 'simulation', 'worlds')

    with open(env_dir, 'rb') as f:
        obstacles = pickle.load(f)
    obstacles = [np.array(poly) * config.WORLD_SCALE for poly in obstacles]

    for i, obstacle in enumerate(obstacles):
        mesh_name = f"world_{world_number}_obstacle_{i}.stl"
        mesh_path = os.path.join(mesh_dir, mesh_name)
        _create_stl(vertices=obstacle, height=1, filename=mesh_path)

    _create_world(mesh_dir, world_number, world_dir)

def _create_stl(vertices: np.ndarray, height: float, filename: str):
    """
    Converts vertices for one obstacle and saves the mesh as an stl file.

    Parameters
    ----------
    """
    n = len(vertices)

    print(f"STARTING FILE: {filename}\n")

    with open(filename, 'w') as f:
        
        poly_center = np.mean(vertices, axis=0) # used to fix rendering

        # start mesh
        f.write('solid obstacle\n')

        # Bottom face
        for i in range(1, n-1):
            v0 = vertices[0]
            v1 = vertices[i + 1]
            v2 = vertices[i]
            _write_triangle(
                f, 
                [v0[0], v0[1], 0.1], # conver to 3D point
                [v1[0], v1[1], 0.1], 
                [v2[0], v2[1], 0.1], 
                center=poly_center # faces down 
            )

        # top face
        for i in range(1, n-1):
            v0 = vertices[0]
            v1 = vertices[i]
            v2 = vertices[i + 1]
            _write_triangle(
                f,
                [v0[0], v0[1], height], # conver to 3D point
                [v1[0], v1[1], height], 
                [v2[0], v2[1], height], 
                center=poly_center # faces up
            )

        # side faces
        for i in range(n):
            v1_bottom = vertices[i]
            v2_bottom = vertices[(i+1)%n]
            
            _write_triangle(
                f,
                [v1_bottom[0], v1_bottom[1], 0.1],
                [v2_bottom[0], v2_bottom[1], 0.1],
                [v2_bottom[0], v2_bottom[1], height], # v2 top
                center = poly_center
            )

            _write_triangle(
                f,
                [v1_bottom[0], v1_bottom[1], 0.1],
                [v2_bottom[0], v2_bottom[1], height], # v2 top
                [v1_bottom[0], v1_bottom[1], height], # v1 top
                center = poly_center
            )

        # finish mesh
        f.write("endsolid obstacle\n")

        print(f"FINISHED FILE: {filename}\n")

def _write_triangle(f, v0, v1, v2, center=None):
    """
    Writes a single triangle to a stl file.

    Parameters
    ----------

    """
    # find normal
    edge1 = np.array(v1) - np.array(v0)
    edge2 = np.array(v2) - np.array(v0)
    arr = np.cross(edge1, edge2)
    norm = np.linalg.norm(arr)
    if norm == 0:
        normal = [0, 0, 0]
    else:
        normal = arr / norm

        if center is not None:
            tri_center_x = (v0[0] + v1[0] + v2[0]) / 3
            tri_center_y = (v0[1] + v1[1] + v2[1]) / 3

            dist_to_center_x = tri_center_x - center[0]
            dist_to_center_y = tri_center_y - center[1]

            dot_product = dist_to_center_x * normal[0] + dist_to_center_y * normal[1]
            
            if dot_product < 0:
                normal = -normal
            
    # write in stl format 
    f.write(f"  facet normal {normal[0]} {normal[1]} {normal[2]}\n")
    f.write(f"      outer loop\n")
    f.write(f"          vertex {v0[0]} {v0[1]} {v0[2]}\n")
    f.write(f"          vertex {v1[0]} {v1[1]} {v1[2]}\n")
    f.write(f"          vertex {v2[0]} {v2[1]} {v2[2]}\n")
    f.write(f"      endloop\n")
    f.write(f"  endfacet\n")

def _create_world(mesh_share_path, world_number, world_dir):
    
    # make world file
    world_name = os.path.join(world_dir, f"world_{world_number}.world")
    with open(world_name, 'w') as f:
        
        # create header
        f.write('<?xml version="1.0"?>\n')
        f.write('<sdf version="1.6">\n')
        f.write(f'   <world name="world_{world_number}">\n\n')

        # physics engine settings
        f.write('       <physics type="ode">\n')
        f.write('           <max_step_size>0.001</max_step_size>\n')
        f.write('           <real_time_factor>1</real_time_factor>\n')
        f.write('       </physics>\n\n')

        # Lighting
        f.write('       <include>\n')
        f.write('           <uri>model://sun</uri>\n')
        f.write('       </include>\n\n')

        # ground plane
        f.write('       <include>\n')
        f.write('           <uri>model://ground_plane</uri>\n')
        f.write('       </include>\n\n')

        
        # Find all stl files with pattern
        mesh_dir = os.path.join(mesh_share_path, f"world_{world_number}_*.stl")
        stl_files = glob.glob(mesh_dir)
        for stl_file in stl_files:
            filename = os.path.basename(stl_file)
            print(f"Adding: {filename}\n")
            obstacle_num = filename.split('_')[-1].split('.')[0]
            uri = f'model://meshes/{filename}'

            # add obstacles
            f.write(f'       <model name="obstacle_{obstacle_num}">\n')
            f.write('           <static>true</static>\n')
            f.write('           <pose>0 0 0 0 0 0</pose>\n')
            f.write('           <link name="link">\n')
            f.write('               <collision name="collision">\n')
            f.write('                   <geometry>\n')
            f.write('                       <mesh>\n')
            f.write(f'                           <uri>{uri}</uri>\n')
            f.write('                       </mesh>\n')
            f.write('                   </geometry>\n')
            f.write('               </collision>\n')
            f.write('               <visual name="visual">\n')
            f.write('                   <geometry>\n')
            f.write('                       <mesh>\n')
            f.write(f'                           <uri>{uri}</uri>\n')
            f.write('                       </mesh>\n')
            f.write('                   </geometry>\n')
            f.write('                   <material>\n')
            f.write('                       <ambient>0.5 0.5 0.5 1</ambient>\n')
            f.write('                       <diffuse>0.7 0.7 0.7 1</diffuse>\n')
            f.write('                   </material>\n')
            f.write('               </visual>\n')
            f.write('           </link>\n')
            f.write('       </model>\n\n')

        # end world file        
        f.write('   </world>\n')
        f.write('</sdf>')

def main():
    env_to_world()

if __name__=="__main__":
    main()