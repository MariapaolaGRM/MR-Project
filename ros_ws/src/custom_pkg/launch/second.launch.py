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

    # Add the commands to the launch description
    ld = LaunchDescription()

    robots = ['robot1','robot2']
    #robots = ['robot1']

    nav2_launch = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch', 'navigation_launch.py'  
    )
    nav2_config_file = os.path.join(
        get_package_share_directory('custom_pkg'),
        'config', 'nav2_config.yaml'
    )
    config = os.path.join(
        get_package_share_directory("custom_pkg"), 
        "config", "explore.yaml"
    )

    for robot in robots:
        namespaced_nav= ReplaceString(
            source_file=nav2_config_file, replacements={"namespace":(robot)} 
        ) 

        # Nav2
        nav2 = GroupAction(
                actions=[
                    PushROSNamespace(robot),
                    SetRemap(src='/tf', dst='/tf'),
                    SetRemap(src='/tf_static', dst='/tf_static'),
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(nav2_launch),
                        launch_arguments={
                            'autostart': 'true',
                            'namespace': robot,
                            'use_sim_time': use_sim_time,
                            'params_file': namespaced_nav, 
                        }.items()
                    )
                ]
        )

        # Explore lite
        explore_lite = Node(
            package="explore_lite",
            name="explore_node",
            namespace=robot,
            executable="explore",
            parameters=[config, 
                        {"use_sim_time": use_sim_time},
                        {"robot_base_frame": f"{robot}/base_link"}],
            output="screen",
        )

        goal_node = Node(
            package='custom_pkg',
            executable='goal_node',
            name='goal_node',
            namespace=robot,
            parameters=[
                    {'namespace':robot},
                    {"use_sim_time": use_sim_time}],
            output='screen',
        )

        ld.add_action(nav2) 
        ld.add_action(explore_lite) 
        ld.add_action(goal_node)

    # Nodi custom
    central_node = Node(
        package='custom_pkg',
        executable='central_node',
        name='central_node',
        parameters=[
                {"use_sim_time": use_sim_time}],
        output='screen',
    )
    ld.add_action(central_node)

    return ld