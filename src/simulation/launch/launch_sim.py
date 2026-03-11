from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
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
    
    # create arg for world
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='empty_world.world',
        description='Empty gazebo world file'
    )

    # use env file associated with world 

    path_publisher_node = Node(
        package = 'path_planning',
        executable = 'path_publisher',
        parameters = [{

        }]
    )

    # resolve args
    model = LaunchConfiguration('model')
    world = LaunchConfiguration('world')

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
            'world' : world
        }.items()

    )
    
    return LaunchDescription([
        model_arg,
        world_arg,
        set_turtlebot_model,
        gazebo,
        path_publisher_node
    ])