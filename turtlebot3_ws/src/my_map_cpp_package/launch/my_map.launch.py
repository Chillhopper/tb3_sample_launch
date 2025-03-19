import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from enum import Enum

class worldEnum(Enum):
    package_name = 0
    world_folder = 1
    world_yaml_file = 2

def generate_launch_description():

    ##CONFIGS##
    configs = {
        'map_file' : os.path.join(get_package_share_directory('map_package'), 'all_maps', 'map.yaml'), #package_name, map_folder_name, map_name
        'spawn_pose': ['-0.131087', '1.882443', '0.01'], #xyz
        'robot_type': 'waffle_pi',
        'world_path': ['turtlebot3_gazebo', 'worlds', 'turtlebot3_world.world'] #package_name, world_folder_name, world_name
    }
    
    if not os.path.exists(configs['map_file']):
        print(f"ERROR: Map file not found at {map_file}. Ensure it is installed correctly.")
        sys.exit(1)  # Force halt the program

    declare_tb3_model_cmd = DeclareLaunchArgument(
        'tb3_model',
        default_value=configs['robot_type'],
        description='TurtleBot3 model type [burger, waffle, waffle_pi]')

    # === Set Environment Variable for TurtleBot3 Model ===
    set_tb3_model_env = SetEnvironmentVariable(
        name='TURTLEBOT3_MODEL',
        value=LaunchConfiguration('tb3_model')
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time')


    # === Launch Gazebo (Server & Client) ===
    gazebo_pkg_path = get_package_share_directory('gazebo_ros')
    
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_pkg_path, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': PathJoinSubstitution([FindPackageShare(configs['world_path'][worldEnum.package_name.value]), configs['world_path'][worldEnum.world_folder.value], configs['world_path'][worldEnum.world_yaml_file.value]])}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_pkg_path, 'launch', 'gzclient.launch.py'))
    )

    # === Robot State Publisher (Publishes TF) ===
    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('turtlebot3_gazebo'),
            'launch',
            'robot_state_publisher.launch.py')),
        launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items()
    )

    # === Spawn TurtleBot3 in Gazebo ===
    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('turtlebot3_gazebo'),
            'launch',
            'spawn_turtlebot3.launch.py')),
        launch_arguments={'x_pose': configs['spawn_pose'][0], 'y_pose': configs['spawn_pose'][1], 'z_pose': configs['spawn_pose'][2]}.items()
    )

    stop_robot_cmd = Node(
        package='geometry_msgs',
        executable='ros2 topic pub',
        arguments=['/cmd_vel', 'geometry_msgs/msg/Twist', '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'],
        output='screen'
    )


    # === Launch Map Server (Fixes Lifecycle Issue) ===
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': LaunchConfiguration('map'),
            'use_sim_time': True
        }]
    )

    # === Lifecycle Manager to Ensure Activation ===
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['map_server']
        }]
    )

    # === Launch RViz (To Visualize the Map) ===
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=['-d', '/opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz']
    )

    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'base_frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'global_frame_id': 'map',
            'scan_topic': 'scan',
            'transform_tolerance': 1.0  # Prevents TF timing issues
        }]
    )

    nav2_bringup = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([get_package_share_directory('nav2_bringup'),'/launch','/bringup_launch.py']),
            launch_arguments={
            'use_sim_time': 'true',
            'map':configs['map_file']}.items()
        )
    

    # === Return Full Launch Description ===
    return LaunchDescription([

        nav2_bringup,
        declare_tb3_model_cmd,
        set_tb3_model_env,
        declare_use_sim_time_cmd,
        gzserver_cmd,
        gzclient_cmd,
        robot_state_publisher_cmd,
        spawn_turtlebot_cmd,
        rviz_node

        # amcl_node
        # declare_map_yaml_cmd,
        # map_server_node,
        # lifecycle_manager,

    ])
