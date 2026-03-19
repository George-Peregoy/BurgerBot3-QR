from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, LogInfo
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from path_planning import config

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
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ),
        launch_arguments={
            'world' : world_file,
            'verbose' : 'false'
        }.items()
    )

    # Load robot description
    robot_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('turtlebot3_gazebo'),
                'launch',
                'robot_state_publisher.launch.py'
            ])
        ]),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Spawn the robot 
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'burger',
            '-topic', 'robot_description',
            '-x', f'{config.START[0]*config.WORLD_SCALE}',
            '-y', f'{config.START[1]*config.WORLD_SCALE}',
            '-z', '0.01'
        ],
        output='screen'
    )
    
    # debug world file
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
        robot_description,
        spawn_robot,
        path_publisher_node
    ])