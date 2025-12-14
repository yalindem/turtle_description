import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Get package paths
    pkg_robot = get_package_share_directory('turtle_description') # Your package name
    xacro_file = os.path.join(pkg_robot, 'urdf', 'turtle.urdf.xacro')
    
    # Generate robot description from XACRO
    robot_description = Command(['xacro ', xacro_file]) # Process xacro

    # Nodes to Launch
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': False, 'robot_description': robot_description}] # Use generated description
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
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='False', description='Use simulation time'),
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz2
    ])
