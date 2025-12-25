# --- START OF FILE mapping.launch.py ---
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取包路径
    pkg_me332 = get_package_share_directory('me332_robot_description')
    pkg_slam_toolbox = get_package_share_directory('slam_toolbox')
    
    # 参数定义
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # 1. 启动 SLAM Toolbox (异步在线建图模式)
    # 使用默认参数，但确保 use_sim_time 为 true
    start_async_slam_toolbox_node = Node(
        parameters=[
          {'use_sim_time': use_sim_time},
          os.path.join(pkg_slam_toolbox, 'config', 'mapper_params_online_async.yaml'),
          {'transform_timeout': 0.2}, 
          {'tf_buffer_duration': 30.0}, 
          {'minimum_time_interval': 0.1}, 
          {'stack_size_to_use': 40000000} 
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen')

    # 2. 启动 RViz2
    # 为了方便，这里我们不加载特定的 rviz 文件，而是启动后手动添加 Display
    # 或者如果您有保存好的配置，可以将 default_rviz_config 路径替换
    default_rviz_config = os.path.join(pkg_me332, 'rviz', 'mapping.rviz')
    
    # 检查 rviz 配置文件是否存在，不存在则启动空 RViz
    if not os.path.exists(default_rviz_config):
        print(f"[WARN] RViz config not found at {default_rviz_config}, launching empty.")
        rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', ''], # Empty config
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        )
    else:
        rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', default_rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation/Gazebo clock'),
        
        start_async_slam_toolbox_node,
        rviz_node
    ])
