import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    '''Gazebo launch per 2 tobot burger_cam'''
    ros_gz_sim = get_package_share_directory('ros_gz_sim')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Posizioni iniziali 
    # posizioni per world2
    x_pose_1 = LaunchConfiguration('x_pose_1', default='3.0') 
    y_pose_1 = LaunchConfiguration('y_pose_1', default='-3.0')
    x_pose_2 = LaunchConfiguration('x_pose_2', default='3.0')
    y_pose_2 = LaunchConfiguration('y_pose_2', default='5.0')

    declare_x_pose_1 = DeclareLaunchArgument('x_pose_1', description='Initial x pose for robot1')
    declare_y_pose_1 = DeclareLaunchArgument('y_pose_1', description='Initial y pose for robot1')
    declare_x_pose_2 = DeclareLaunchArgument('x_pose_2', description='Initial x pose for robot2')
    declare_y_pose_2 = DeclareLaunchArgument('y_pose_2', description='Initial y pose for robot2')

    # world ufficiale
    #x_pose_1 = LaunchConfiguration('x_pose', default='2.5') 
    #y_pose_1 = LaunchConfiguration('y_pose', default='-5.5')
    #x_pose_2 = LaunchConfiguration('x_pose', default='4.5')
    #y_pose_2 = LaunchConfiguration('y_pose', default='5.5')

    world = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'models',
        'turtlebot3_house',
        #'house_model.sdf'
        'house2_model.sdf'
    )

    model_file1 = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'models',
        'turtlebot3_burger_cam',
        'model1.sdf'
    )

    model_file2 = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'models',
        'turtlebot3_burger_cam',
        'model2.sdf'
    )    

    urdf_path1 = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'urdf',
        'turtlebot3_burger_cam1.urdf'
        )

    with open(urdf_path1, 'r') as infp:
        robot_desc1 = infp.read()

    urdf_path2 = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'urdf',
        'turtlebot3_burger_cam2.urdf'
        )

    with open(urdf_path2, 'r') as infp:
        robot_desc2 = infp.read()

    bridge_params = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'params',
        'turtlebot3_burger_cam_bridge.yaml'
    )

    # Launch Gazebo simulation
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r -s -v2 ', world], 'on_exit_shutdown': 'true'}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-g -v2 ', 'on_exit_shutdown': 'true'}.items()
    )

    set_env_vars_resources = AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            os.path.join(
                get_package_share_directory('turtlebot3_gazebo'),
                'models'))

    # Spawn e configurazione dei robot burger_cam
    spawn_turtlebot_cmd1 = Node(
        package='ros_gz_sim',
        executable='create',
        namespace='robot1',
        arguments=[
            '-name', 'robot1',
            '-file', model_file1,
            '-x', x_pose_1,
            '-y', y_pose_1,
            '-z', '0.01',
            '-Y', '3.14159'
        ],
        output='screen',
    )

    spawn_turtlebot_cmd2 = Node(
        package='ros_gz_sim',
        executable='create',
        namespace='robot2',
        arguments=[
            '-name', 'robot2',
            '-file', model_file2,
            '-x', x_pose_2,
            '-y', y_pose_2,
            '-z', '0.01',
            '-Y', '3.14159'
        ],
        output='screen',
    )

    start_gazebo_ros_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ],
        output='screen',
    )

    start_gazebo_ros_image_bridge_cmd = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=[
            '/robot1/camera/image_raw',
            '/robot2/camera/image_raw',
        ],
        output='screen',
    )

    robot_state_publisher_cmd1 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace='robot1',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_desc1
        }]
    )

    robot_state_publisher_cmd2 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace='robot2',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_desc2
        }]
    )
    
    # Add the commands to the launch description
    ld = LaunchDescription()
    ld.add_action(declare_x_pose_1)
    ld.add_action(declare_y_pose_1)
    ld.add_action(declare_x_pose_2)
    ld.add_action(declare_y_pose_2)

    ld.add_action(gzserver_cmd) 
    #ld.add_action(gzclient_cmd) # se commentato non si apre gazebo
    ld.add_action(set_env_vars_resources)
    
    ld.add_action(spawn_turtlebot_cmd1)
    ld.add_action(spawn_turtlebot_cmd2)
    ld.add_action(start_gazebo_ros_bridge_cmd)
    ld.add_action(start_gazebo_ros_image_bridge_cmd)
    ld.add_action(robot_state_publisher_cmd1)
    ld.add_action(robot_state_publisher_cmd2)   

    return ld
