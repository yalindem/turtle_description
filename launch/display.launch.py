import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
    # Paket yolu ve URDF/Xacro dosyasının yolu
    pkg_share = get_package_share_directory('turtle_description')
    xacro_file = os.path.join(pkg_share, 'urdf', 'turtle.urdf.xacro')

    # Xacro dosyasını okuyup URDF'ye dönüştürmek için komut
    # Bu, tüm sabit (fixed) dönüşümleri (örneğin base_link -> laser_link) içerir.
    robot_description_content = Command(['xacro ', xacro_file])
    
    # 1. Robot State Publisher (RSP) Düğümü
    # Bu düğüm URDF'yi alır ve STATİK TF'leri (/tf_static topic'inde) yayınlar.
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description_content}],
        output='screen'
    )

    # 2. Joint State Publisher (JSP) Düğümü (Sadece test için gerekli, Gazebo bunu üstlenir)
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=LaunchConfiguration('gui'),
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(name='gui', default_value='True',
                              description='GUI ile JSP başlatılıp başlatılmayacağı'),
        
        robot_state_publisher_node,
        joint_state_publisher_node
    ])