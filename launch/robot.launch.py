import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
# Import the IfCondition module
from launch.conditions import IfCondition 
from launch.substitutions import LaunchConfiguration, Command
# from launch.launch_description_sources import PythonLaunchDescriptionSource # This import wasn't used

def generate_launch_description():
    # 1. Define the launch argument for controlling Rviz start up
    declare_rviz_arg = DeclareLaunchArgument(
        'start_rviz', 
        default_value='False', 
        description='Whether to start Rviz2'
    )

    declare_gz_arg = DeclareLaunchArgument(
        'start_gz', 
        default_value='False', 
        description='Whether to start Rviz2'
    )

    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', 
        default_value='False', 
        description='Whether to start with use_sim_time flag'
    )
    
    # 2. Get the value of the argument during launch time
    # This value will be used in the IfCondition below
    start_rviz = LaunchConfiguration('start_rviz')
    start_gz = LaunchConfiguration('start_gz')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Get package paths
    pkg_robot = get_package_share_directory('turtle_description') # Your package name
    xacro_file = os.path.join(pkg_robot, 'urdf', 'turtle.urdf.xacro')
    
    # Generate robot description from XACRO
    robot_description = Command(['xacro ', xacro_file]) # Process xacro

    '''
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')
    gz_launch_dir = os.path.join(ros_gz_sim_share, 'launch')


    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gz_launch_dir, 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': 'empty.sdf -r'}.items(), # example: starts with empty world and runs immediately
    )

    gz_sim_node = Node(
        package='gz',
        executable='gz_sim',
        name='gz_sim',
        output='screen'    
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name', 'turtle',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1',
        ],
    )

    '''
    # Nodes to Launch
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_description}] # Use generated description
    )

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        # 3. Add the condition here: only launch if 'start_rviz' is 'True'
        condition=IfCondition(start_rviz) 
    )

    diff_drive_kinematic_node = Node(
        package='turtle_description',
        executable='diff_drive_node',
        name='diff_drive_node',
        output='screen'
    )

    cmd_vel_publisher = ExecuteProcess(
        cmd=[
            'ros2', 'topic', 'pub', '/cmd_vel', 'geometry_msgs/msg/Twist',
            '{"linear": {"x": 1.0}, "angular": {"z": 0.5}}', '-r', '1'
        ],
        output='screen'
    )

    return LaunchDescription([
        # Add the new launch argument declaration
        declare_rviz_arg,
        declare_gz_arg,
        declare_use_sim_time_arg,
        robot_state_publisher,
        joint_state_publisher_gui,
        diff_drive_kinematic_node,
        rviz2,
        cmd_vel_publisher
        #gz_sim,
        #spawn_entity
    ])