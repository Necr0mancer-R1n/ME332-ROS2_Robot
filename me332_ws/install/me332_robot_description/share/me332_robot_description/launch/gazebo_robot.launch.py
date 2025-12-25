import os
import xacro
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
    RegisterEventHandler,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node

def _launch_setup(context, *args, **kwargs):
    # ========================================================================
    # 1. 解析配置参数
    # ========================================================================
    model_path = LaunchConfiguration("model").perform(context)
    world_path = LaunchConfiguration("world").perform(context)
    ctrl_config_path = LaunchConfiguration("controllers_yaml").perform(context)

    # 打印调试信息，确保路径正确
    print(f"[DEBUG] Model Path: {model_path}")
    print(f"[DEBUG] Config Path: {ctrl_config_path}")

    # ========================================================================
    # 2. 处理 Xacro 文件
    # ========================================================================
    # 关键修复：通过 mappings 将 YAML 路径传给 Xacro 内部的 $(arg controllers_yaml)
    doc = xacro.process_file(
        model_path, 
        mappings={'controllers_yaml': ctrl_config_path} 
    )
    robot_description_config = doc.toxml()

    # ========================================================================
    # 3. 节点定义
    # ========================================================================
    
    # A. Robot State Publisher (发布 /robot_description)

    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": robot_description_config,
            "use_sim_time": True ,
            "publish_frequency": 50.0 ,
            "ignore_timestamp": False ,
        }],
    )

    # B. Gazebo (加载世界)
    # 注意：不要在这里传 robot_description 参数，避免命令行过长报错
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py")
        ),
        launch_arguments={"world": world_path}.items(),
    )

    # C. Spawn Entity (在 Gazebo 中生成机器人)
    # 关键修复：使用 "-topic robot_description" 从 RSP 获取模型，避免 truncated 错误
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic", "robot_description",
            "-entity", "me332_mobile_manipulator",
            "-z", "0.10", 
        ],
        output="screen",
    )

    # D. 加载控制器
    # 注意：这里的名字必须与 me332_mm_controllers.yaml 中的 key 完全一致！
    
    # 1. 关节状态广播器
    load_jsb = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "--set-state", "active", "joint_state_broadcaster"],
        output="screen"
    )

    # 2. 底盘控制器 (YAML里叫 base_controller)
    load_base_ctrl = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "--set-state", "active", "base_controller"],
        output="screen"
    )

    # 3. 机械臂控制器 (YAML里叫 arm_trajectory_controller)
    load_arm_ctrl = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "--set-state", "active", "arm_trajectory_controller"],
        output="screen"
    )

    # 4. 夹爪控制器 (YAML里叫 gripper_trajectory_controller)
    load_gripper_ctrl = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "--set-state", "active", "gripper_trajectory_controller"],
        output="screen"
    )

    # ========================================================================
    # 4. 设置启动顺序 (事件处理)
    # ========================================================================
    # 顺序：RSP & Gazebo -> Spawn -> JSB -> Base -> Arm -> Gripper
    
    return [
        node_robot_state_publisher,
        gazebo,
        spawn_entity,
        
        # Spawn 结束后 -> 加载 Joint State Broadcaster
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_jsb],
            )
        ),
        # JSB 加载完 -> 加载 Base Controller
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_jsb,
                on_exit=[load_base_ctrl],
            )
        ),
        # Base 加载完 -> 加载 Arm Controller
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_base_ctrl,
                on_exit=[load_arm_ctrl],
            )
        ),
        # Arm 加载完 -> 加载 Gripper Controller
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_arm_ctrl,
                on_exit=[load_gripper_ctrl],
            )
        ),
        
    ]

def generate_launch_description():
    # 定义包名
    pkg_name = "me332_robot_description" 
    try:
        pkg = get_package_share_directory(pkg_name)
    except:
        print(f"Error: Could not find package '{pkg_name}'. Please source your workspace.")
        return LaunchDescription([])

    # 定义默认路径
    default_model = os.path.join(pkg, "urdf", "robot.xacro")
    default_ctrl = os.path.join(pkg, "config", "me332_mm_controllers.yaml")
    default_world = os.path.join(get_package_share_directory("gazebo_ros"), "worlds", "empty.world")
    cleanup_script = os.path.join(pkg, "scripts", "cleanup_gazebo.sh")

    # 声明 Launch 参数
    model_arg = DeclareLaunchArgument("model", default_value=default_model)
    ctrl_arg = DeclareLaunchArgument("controllers_yaml", default_value=default_ctrl)
    world_arg = DeclareLaunchArgument("world", default_value=default_world)

    # 定义清理进程 (可选，如果你的脚本存在的话)
    cleanup = ExecuteProcess(
        cmd=["bash", cleanup_script],
        output="screen",
    )

    # cleanup 结束后再启动主逻辑
    start_after_cleanup = RegisterEventHandler(
        OnProcessExit(
            target_action=cleanup,
            on_exit=[OpaqueFunction(function=_launch_setup)],
        )
    )

    return LaunchDescription([
        model_arg, 
        ctrl_arg, 
        world_arg,
        cleanup,
        start_after_cleanup,
    ])
