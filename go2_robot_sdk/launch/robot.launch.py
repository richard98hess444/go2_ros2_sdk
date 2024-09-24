import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import UnlessCondition, IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, GroupAction, DeclareLaunchArgument
from launch.launch_description_sources import FrontendLaunchDescriptionSource, PythonLaunchDescriptionSource



def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    no_rviz2 = LaunchConfiguration('no_rviz2', default='false')

    waypoint_arg = DeclareLaunchArgument(
        'waypoint', 
        default_value='False', 
        description='Enable or disable waypoint mode'
    )
    

    robot_token = os.getenv('ROBOT_TOKEN', '') # how does this work for multiple robots?
    robot_ip = 'robot0'

    # these are debug only
    map_name = os.getenv('MAP_NAME', '3d_map')
    save_map = os.getenv('MAP_SAVE', 'true')
    conn_type = os.getenv('CONN_TYPE', 'webrtc')

    rviz_config = "multi_robot_conf.rviz"
    urdf_file_name = 'multi_go2.urdf'
    urdf = os.path.join(
        get_package_share_directory('go2_robot_sdk'),
        "urdf",
        urdf_file_name)
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    robot_desc_modified_lst = []
    robot_desc_modified_lst.append(robot_desc.format(robot_num="robot0"))

    joy_params = os.path.join(
        get_package_share_directory('go2_robot_sdk'),
        'config', 'joystick.yaml'
    )

    default_config_topics = os.path.join(
        get_package_share_directory('go2_robot_sdk'),
        'config', 'twist_mux.yaml')

    foxglove_launch = os.path.join(
        get_package_share_directory('foxglove_bridge'),
        'launch',
        'foxglove_bridge_launch.xml',
    )

    slam_toolbox_config = os.path.join(
        get_package_share_directory('go2_robot_sdk'),
        'config',
        'mapper_params_online_async.yaml'
    )

    nav2_config = os.path.join(
        get_package_share_directory('go2_robot_sdk'),
        'config',
        'nav2_params.yaml'
    )

    map_dir = os.path.join(
        get_package_share_directory('go2_robot_sdk'),
        'config',
        # 'map_flat_1.yaml'
        'aic_map_2.yaml'
    )

    urdf_launch_nodes = []
    urdf_launch_nodes.append(
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            namespace="robot0",
            parameters=[{'use_sim_time': use_sim_time,
                         'robot_description': robot_desc_modified_lst[0]}], 
            arguments=[urdf]
        ),
    )
    urdf_launch_nodes.append(
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            remappings=[
                ('cloud_in', '/robot0/point_cloud2'),
                ('scan', '/scan'),
            ],
            parameters=[{
                'target_frame': 'robot0/base_link',
                'max_height': 0.5,
                'use_sim_time': use_sim_time
            }],
            output='screen',
        ),
    )

    abandon_group = GroupAction([
        Node(
            package='ros2_go2_video',
            executable='ros2_go2_video',
            parameters=[{'robot_ip': robot_ip,
                         'robot_token': robot_token}],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_pub_base_link_to_base_footprint',
            arguments=['0', '0', '0', '0', '0', '0', 'robot0/base_link', 'base_footprint'],
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        Node(
            package='nav2_tutorial',
            executable='timestamp_corrector',
            name='timestamp_corrector',
        ),
        Node(
            package='topic_tools',
            executable='relay',
            name='cmd_vel_relay',
            remappings=[
                ('/cmd_vel_nav', '/robot0/cmd_vel')
            ]
        ),
        Node(
            package='go2_robot_sdk',
            executable='go2_driver_node',
            parameters=[{'robot_ip': robot_ip, 'token': robot_token, "conn_type": conn_type}],
        ),
        Node(
            package='go2_robot_sdk',
            executable='lidar_to_pointcloud',
            parameters=[{'robot_ip_lst': robot_ip, 'map_name': map_name, 'map_save': save_map}],
        ),
        Node(
            package='joy',
            executable='joy_node',
            parameters=[joy_params]
        ),
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_node',
            parameters=[default_config_topics],
        ),
        Node(
            package='twist_mux',
            executable='twist_mux',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                default_config_topics
            ],
        ),
        IncludeLaunchDescription(
            FrontendLaunchDescriptionSource(foxglove_launch)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
            ]),
            launch_arguments={
                'params_file': slam_toolbox_config,
                'use_sim_time': use_sim_time,
            }.items(),
        ),
    ])

    nav_group = GroupAction([
        # *urdf_launch_nodes,
        Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            condition=UnlessCondition(no_rviz2),
            name='rviz2',
            arguments=['-d' + os.path.join(get_package_share_directory('go2_robot_sdk'), 'config', rviz_config)]
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('nav2_bringup'), 
                             'launch', 
                             'bringup_launch.py') # navigation_launch
            ]),
            launch_arguments={
                'map': map_dir,
                'params_file': nav2_config,
                'use_sim_time': use_sim_time,
            }.items(),
        ),
    ])

    waypoint_group = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('go2_robot_sdk'), 
                             'launch', 
                             'isaac_ros_navigation_goal.launch.py')
            ]),
        ),
    ])

    ld = LaunchDescription()
    ld.add_action(waypoint_arg)
    waypoint = LaunchConfiguration('waypoint')
    ld.add_action(nav_group)
    ld.add_action(GroupAction(actions=[waypoint_group], condition=IfCondition(waypoint)))

    return ld

