import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushROSNamespace
from nav2_common.launch import ReplaceString

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Add the commands to the launch description
    ld = LaunchDescription()

    # Positions for house 1 and 2
    #x_pose_1 = '3.0'
    #y_pose_1 = '-3.0'
    #x_pose_2 = '3.0'
    #y_pose_2 = '5.0'

    # Positions for house 3, 4 and 5 
    x_pose_1 = '2.0'
    y_pose_1 = '-5.0'
    x_pose_2 = '2.0'
    y_pose_2 = '5.5'


    robots = ['robot1','robot2']
    #robots = ['robot2'] # to use only one robot to uncomment and comment the other

    slam_launch = os.path.join(
        get_package_share_directory('slam_toolbox'),
        'launch', 'online_async_launch.py'  
    )

    slam_config = os.path.join(
        get_package_share_directory('custom_pkg'),
        'config', 'slam.yaml'  
    )

    rviz_config = os.path.join(
        get_package_share_directory('custom_pkg'),
        'rviz', 'rviz_config.rviz'  
    )

    world_launch = os.path.join(
        get_package_share_directory('custom_pkg'),
        'launch', 'gazebo_multirobot.launch.py'  
    )

    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(world_launch),
        launch_arguments={
                'x_pose_1' : x_pose_1,
                'y_pose_1' : y_pose_1,
                'x_pose_2' : x_pose_2,
                'y_pose_2' : y_pose_2,
        }.items()        
    )
    ld.add_action(world_launch)
    
    for robot in robots:
        if robot == 'robot1':
            x_pose = x_pose_1
            y_pose = y_pose_1
        elif robot == 'robot2':
            x_pose = x_pose_2
            y_pose = y_pose_2

        namespaced_slam= ReplaceString(
            source_file=slam_config, replacements={"namespace":(robot)} 
        )
        # Launch SLAM Toolbox
        slam = GroupAction(
            actions=[
                PushROSNamespace(robot),
                IncludeLaunchDescription(
                PythonLaunchDescriptionSource(slam_launch),
                launch_arguments={
                    'use_sim_time': use_sim_time,  
                    'slam_params_file': namespaced_slam,
                }.items()
                )
        ])
        # Static transformations to create the frame map
        map_broadcaster_cmd = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_broadcaster',
            namespace=robot,
            arguments=[
                "--x", x_pose, "--y", y_pose, "--z", "0.01",
                "--roll", "0.0", "--pitch", "0.0", "--yaw", "3.14159",
                "--frame-id", "map", "--child-frame-id", f"{robot}/map"
            ],
            output="screen" 
        )
        # YOLO
        yolo_node = Node(
            package='custom_pkg',
            executable='yolo_node',
            name='yolo_node',
            namespace=robot,
            parameters=[
                    {'namespace':robot},
                    {"use_sim_time": use_sim_time}],
            output='screen',
        )
        
        ld.add_action(slam)
        ld.add_action(map_broadcaster_cmd)
        ld.add_action(yolo_node)
    
    # Rviz2 with parameters
    rviz2 = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen')

    ld.add_action(rviz2)

    return ld