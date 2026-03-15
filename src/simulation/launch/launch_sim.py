from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
import os

def generate_launch_description():
 
    # create arg for model
    model_arg = DeclareLaunchArgument(
        'model',
        default_value='burger',
        description='TurtleBot3 model type'
    )  
    
    # where world dir is kept
    sim_share = get_package_share_directory('simulation')
    set_model_path = SetEnvironmentVariable(
        'GAZEBO_MODEL_PATH',
        sim_share
    )
    
    # create arg for world
    world_num_arg = DeclareLaunchArgument(
        'world_num',
        default_value='0',
        description='Arg for which world to use.'
        )

    world_num = LaunchConfiguration('world_num')

    world_file = PathJoinSubstitution([
        sim_share,
        'worlds',
        ['world_', world_num, '.world']
    ])

    # use env file associated with world 
    path_share = get_package_share_directory('path_planning')
    env_file = PathJoinSubstitution([
        path_share,
        'environments',
        ['environment_polygon_', world_num, '.pickle']
    ])

    path_publisher_node = Node(
        package = 'path_planning',
        executable = 'path_publisher',
        parameters = [{
            'env_file': env_file
        }]
    )

    # resolve args
    model = LaunchConfiguration('model')

    # set env variable for model
    set_turtlebot_model = SetEnvironmentVariable(
        'TURTLEBOT3_MODEL',
        model
    )

    # gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('turtlebot3_gazebo'),
                'launch',
                'turtlebot3_world.launch.py'
            ])
        ),
        launch_arguments={
            'model' : model,
            'world' : world_file
        }.items()

    )

    # Add this to see what's being passed
    from launch.actions import LogInfo
    log_world = LogInfo(
        msg=['Loading world file: ', world_file]
    )
    
    return LaunchDescription([
        set_model_path,
        model_arg,
        world_num_arg,
        set_turtlebot_model,
        log_world,
        gazebo,
        path_publisher_node
    ])