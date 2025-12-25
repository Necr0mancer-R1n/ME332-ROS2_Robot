import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 1. 获取包路径
    pkg_me332 = get_package_share_directory('me332_robot_description')
    pkg_nav2 = get_package_share_directory('nav2_bringup')
    
    # 2. 默认地图路径
    default_map_path = os.path.join(pkg_me332, 'maps', 'map.yaml')
    
    # 3. 参数定义
    use_sim_time_arg = LaunchConfiguration('use_sim_time', default='True')
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file', default=os.path.join(pkg_me332, 'config', 'nav2_params.yaml'))

    # 4. Nav2 启动组
    nav2_group = GroupAction(
        actions=[
            # 确保命令速度话题映射正确 (对应你的 Gazebo 插件)
            SetRemap(src='/cmd_vel', dst='/base_controller/cmd_vel_unstamped'),
            
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(pkg_nav2, 'launch', 'bringup_launch.py')),
                launch_arguments={
                    'map': map_yaml_file,
                    'use_sim_time': use_sim_time_arg,
                    'params_file': params_file,
                    'autostart': 'true',
                    'use_composition': 'False',
                    'use_rviz': 'False',
                    'slam': 'False', 
                }.items(),
            ),
        ]
    )

    # ==========================================================
    # 5. RViz 启动逻辑
    # ==========================================================
    user_rviz_config = os.path.join(pkg_me332, 'scripts', 'nav2_rviz_config.rviz')
    
    if not os.path.exists(user_rviz_config):
        print(f"\n[WARN] 找不到自定义配置文件: {user_rviz_config}")
        print("[WARN] 正在回退到 Nav2 默认配置...\n")
        user_rviz_config = os.path.join(pkg_nav2, 'rviz', 'nav2_default_view.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', user_rviz_config],
        parameters=[{'use_sim_time': True}], 
        output='screen'
    )

    # 6. 延时启动包装器 (等待 Gazebo 完全就绪)
    delayed_start = TimerAction(
        period=5.0,
        actions=[nav2_group, rviz_node]
    )

    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=default_map_path, description='Full path to map yaml file'),
        DeclareLaunchArgument('use_sim_time', default_value='True', description='Use simulation/Gazebo clock'),
        DeclareLaunchArgument('params_file', default_value=os.path.join(pkg_me332, 'config', 'nav2_params.yaml'), description='params file'),
        
        delayed_start
    ])
