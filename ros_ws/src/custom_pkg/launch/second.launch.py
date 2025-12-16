import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import GroupAction
from launch_ros.actions import PushROSNamespace
from launch_ros.actions import SetRemap

from nav2_common.launch import ReplaceString

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # File 
    nav2_launch = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch', 'navigation_launch.py'  
    )

    nav2_config_file = os.path.join(
        get_package_share_directory('custom_pkg'),
        'config', 'nav2_config.yaml'
    )

    #nav2_config_file_2 = os.path.join(
    #    get_package_share_directory('custom_pkg'),
    #    'config', 'nav2_config2.yaml'  
    #)

    namespaced_nav1= ReplaceString(
        source_file=nav2_config_file, replacements={"namespace":("robot1")} 
    )

    namespaced_nav2= ReplaceString(
        source_file=nav2_config_file, replacements={"namespace":("robot2")} 
    )

    config = os.path.join(
        get_package_share_directory("custom_pkg"), 
        "config", "explore.yaml"
    )

    config2 = os.path.join(
        get_package_share_directory("custom_pkg"), 
        "config", "explore2.yaml"
    )
    
    # Nav2
    nav2_1 = GroupAction(
            actions=[
                PushROSNamespace('robot1'),
                SetRemap(src='/tf', dst='/tf'),
                SetRemap(src='/tf_static', dst='/tf_static'),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(nav2_launch),
                    launch_arguments={
                        'autostart':'true',
                        'namespace':'robot1',
                        'use_sim_time': use_sim_time,
                        'params_file': namespaced_nav1, 
                    }.items()
                )
            ]
    )

    nav2_2 = GroupAction(
            actions=[
                PushROSNamespace('robot2'),
                SetRemap(src='/tf', dst='/tf'),
                SetRemap(src='/tf_static', dst='/tf_static'),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(nav2_launch),
                    launch_arguments={
                        'autostart':'true',
                        'namespace':'robot2',
                        'use_sim_time': use_sim_time,
                        'params_file': namespaced_nav2,  
                    }.items()
                )
            ]
    )

    # Explore lite
    explore_lite_1 = Node(
        package="explore_lite",
        name="explore_node",
        namespace="robot1",
        executable="explore",
        parameters=[config, 
                    {"use_sim_time": use_sim_time},
                    {"robot_base_frame": "robot1/base_link"}],
        output="screen",
        # remappings=[("/tf", "/tf"), ("/tf_static", "/tf_static")],
    )

    explore_lite_2 = Node(
        package="explore_lite",
        name="explore_node",
        namespace='robot2',
        executable="explore",
        parameters=[config, 
                    {"use_sim_time": use_sim_time},
                    {"robot_base_frame": "robot2/base_link"}
                    ],
        output="screen",
        # arguments=["--ros-args", "--log-level", "robot2.explore_node:=debug"]
        # remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
    )

    # Nodi custom
    central_node = Node(
        package='custom_pkg',
        executable='central_node',
        name='central_node',
        parameters=[
                {"use_sim_time": use_sim_time}],
        output='screen',
    )

    goal_node_1 = Node(
        package='custom_pkg',
        executable='goal_node',
        name='goal_node',
        namespace='robot1',
        parameters=[
                {'namespace':'robot1'},
                {"use_sim_time": use_sim_time}],
        output='screen',
    )

    goal_node_2 = Node(
        package='custom_pkg',
        executable='goal_node',
        name='goal_node',
        namespace='robot2',
        parameters=[
                {'namespace':'robot2'},
                {"use_sim_time": use_sim_time}],
        output='screen',
    )


    # Add the commands to the launch description
    ld = LaunchDescription()

    ld.add_action(nav2_1)
    ld.add_action(nav2_2)

    ld.add_action(explore_lite_1)
    ld.add_action(explore_lite_2)

    ld.add_action(central_node)

    ld.add_action(goal_node_1)
    ld.add_action(goal_node_2)

    return ld