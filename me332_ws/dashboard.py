#!/usr/bin/env python3
import os
import sys
import subprocess
import time

# ================= 配置区域 =================
ROBOT_PKG_NAME = "me332_robot_description" 
GAZEBO_LAUNCH_FILE = "gazebo_robot.launch.py"
MAPPING_LAUNCH_FILE = "mapping.launch.py"
NAV_LAUNCH_FILE = "navigation.launch.py"

TELEOP_PKG_NAME = "me332_teleop"
TELEOP_EXEC     = "teleop_keyboard" 
VOICE_EXEC      = "teleop_voice"
GESTURE_EXEC    = "teleop_gesture"

WORLDS_DIR_REL = f"src/{ROBOT_PKG_NAME}/worlds"
MAPS_DIR_REL   = f"src/{ROBOT_PKG_NAME}/maps"

# 新增：记录当前激活的世界名称（默认为 empty）
CURRENT_WORLD_NAME = "empty_world"

# ===========================================

class Colors:
    HEADER = '\033[95m'
    BLUE = '\033[94m'
    CYAN = '\033[96m'
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    RED = '\033[91m'
    END = '\033[0m'
    BOLD = '\033[1m'

WORKSPACE_ROOT = os.getcwd()
SETUP_BASH_PATH = os.path.join(WORKSPACE_ROOT, "install", "setup.bash")

def clear_screen():
    sys.stdout.write("\033[H\033[J")
    sys.stdout.flush()

def get_sourced_command(cmd):
    return f"source {SETUP_BASH_PATH} && {cmd}"

def install_dependencies():
    print(f"\n{Colors.YELLOW}正在准备安装所有依赖项...{Colors.END}")
    sys_packages = [
        "ros-humble-gazebo-ros-pkgs", "ros-humble-gazebo-ros2-control",
        "ros-humble-ros2-control", "ros-humble-ros2-controllers",
        "ros-humble-xacro", "ros-humble-rviz2", "ros-humble-slam-toolbox",
        "ros-humble-navigation2", "ros-humble-nav2-bringup", "ros-humble-nav2-map-server",
        "portaudio19-dev", "python3-pip"
    ]
    pkg_str = " ".join(sys_packages)
    cmd_apt = f"sudo apt update && sudo apt install -y {pkg_str}"
    cmd_pip = "pip3 install vosk pyaudio mediapipe opencv-python"
    try:
        subprocess.run(cmd_apt, shell=True, check=True)
        subprocess.run(cmd_pip, shell=True, check=True)
        print(f"\n{Colors.GREEN}✅ 依赖安装完成！{Colors.END}")
    except Exception as e:
        print(f"\n{Colors.RED}❌ 安装失败: {e}{Colors.END}")
    input("按回车继续...")

def launch_gazebo():
    global CURRENT_WORLD_NAME
    print(f"\n{Colors.CYAN}[场景选择]{Colors.END}")
    worlds_dir = os.path.join(WORKSPACE_ROOT, WORLDS_DIR_REL)
    available_worlds = []
    if os.path.exists(worlds_dir):
        available_worlds = sorted([f for f in os.listdir(worlds_dir) if f.endswith('.world')])
    
    print(f"  {Colors.GREEN}[0]{Colors.END} Empty World")
    for i, w in enumerate(available_worlds):
        print(f"  {Colors.GREEN}[{i+1}]{Colors.END} {w}")

    choice = input(f"\n{Colors.BOLD}请选择场景序号 > {Colors.END}").strip()
    
    world_arg = ""
    if choice == '0' or choice == '':
        CURRENT_WORLD_NAME = "empty_world"
    else:
        try:
            idx = int(choice) - 1
            if 0 <= idx < len(available_worlds):
                # 记录不带后缀的世界名称
                CURRENT_WORLD_NAME = available_worlds[idx].replace('.world', '')
                world_arg = f"world:={os.path.join(worlds_dir, available_worlds[idx])}"
            else:
                return
        except: return

    final_cmd = get_sourced_command(f"ros2 launch {ROBOT_PKG_NAME} {GAZEBO_LAUNCH_FILE} {world_arg}")
    subprocess.Popen(f"gnome-terminal --title='Gazebo' -- bash -c '{final_cmd}; exec bash'", shell=True)
    print(f"{Colors.GREEN}已记录当前环境为: {CURRENT_WORLD_NAME}{Colors.END}")
    time.sleep(1)

def launch_mapping():
    final_cmd = get_sourced_command(f"ros2 launch {ROBOT_PKG_NAME} {MAPPING_LAUNCH_FILE}")
    subprocess.Popen(f"gnome-terminal --title='SLAM' -- bash -c '{final_cmd}; exec bash'", shell=True)

def save_map():
    global CURRENT_WORLD_NAME
    maps_dir = os.path.join(WORKSPACE_ROOT, MAPS_DIR_REL)
    if not os.path.exists(maps_dir): os.makedirs(maps_dir)
    
    map_name = CURRENT_WORLD_NAME
    save_path = os.path.join(maps_dir, map_name)
    
    print(f"\n{Colors.CYAN}[自动保存地图]{Colors.END}")
    print(f"{Colors.YELLOW}正在保存地图到: {save_path}.yaml/pgm ...{Colors.END}")
    
    save_cmd = get_sourced_command(f"ros2 run nav2_map_server map_saver_cli -f {save_path} --ros-args -p save_map_timeout:=10.0")
    
    try:
        for ext in ['.yaml', '.pgm']:
            old_file = save_path + ext
            if os.path.exists(old_file): os.remove(old_file)

        result = subprocess.run(save_cmd, shell=True, executable='/bin/bash')
        
        if result.returncode == 0:
            print(f"\n{Colors.GREEN}✅ 地图成功保存为: {map_name}.yaml 和 {map_name}.pgm{Colors.END}")
            
            yaml_path = save_path + ".yaml"
            if os.path.exists(yaml_path):
                with open(yaml_path, 'r') as f:
                    lines = f.readlines()
                with open(yaml_path, 'w') as f:
                    for line in lines:
                        if line.startswith('image:'):
                            f.write(f"image: {map_name}.pgm\n")
                        else:
                            f.write(line)
        else:
            print(f"\n{Colors.RED}❌ 保存失败。提示：请检查 SLAM 节点是否开启。{Colors.END}")
    except Exception as e:
        print(f"\n{Colors.RED}发生错误: {e}{Colors.END}")
    input("\n按回车返回菜单...")

def launch_navigation():
    # 自动读取当前世界对应的地图
    global CURRENT_WORLD_NAME
    maps_dir = os.path.join(WORKSPACE_ROOT, MAPS_DIR_REL)
    map_file = os.path.join(maps_dir, f"{CURRENT_WORLD_NAME}.yaml")
    
    print(f"\n{Colors.CYAN}[自动加载导航]{Colors.END}")
    
    if os.path.exists(map_file):
        print(f"{Colors.GREEN}发现匹配地图: {CURRENT_WORLD_NAME}.yaml，正在启动...{Colors.END}")
        final_cmd = get_sourced_command(f"ros2 launch {ROBOT_PKG_NAME} {NAV_LAUNCH_FILE} map:='{map_file}'")
        subprocess.Popen(f"gnome-terminal --title='Navigation' -- bash -c '{final_cmd}; exec bash'", shell=True)
    else:
        print(f"\n{Colors.RED}❌ 错误: 未找到与当前世界 [{CURRENT_WORLD_NAME}] 匹配的地图文件。{Colors.END}")
        print(f"{Colors.YELLOW}提示: 请先使用选项 [4] 建图并使用选项 [5] 保存。{Colors.END}")
        input("\n按回车返回菜单...")
    
    time.sleep(1)

def launch_teleop():
    final_cmd = get_sourced_command(f"ros2 run {TELEOP_PKG_NAME} {TELEOP_EXEC}")
    subprocess.Popen(f"gnome-terminal --title='Keyboard' -- bash -c '{final_cmd}; exec bash'", shell=True)

def launch_voice():
    final_cmd = get_sourced_command(f"ros2 run {TELEOP_PKG_NAME} {VOICE_EXEC}")
    subprocess.Popen(f"gnome-terminal --title='Voice' -- bash -c '{final_cmd}; exec bash'", shell=True)

def launch_gesture():
    final_cmd = get_sourced_command(f"ros2 run {TELEOP_PKG_NAME} {GESTURE_EXEC}")
    subprocess.Popen(f"gnome-terminal --title='Gesture' -- bash -c '{final_cmd}; exec bash'", shell=True)

def kill_all():
    print(f"\n{Colors.RED}正在关闭所有进程...{Colors.END}")
    targets = ["gazebo", "rviz2", "slam_toolbox", "robot_state_publisher", "controller_manager", "bt_navigator", "nav2_planner"]
    for t in targets: os.system(f"pkill -f {t}")
    os.system(f"pkill -f {TELEOP_EXEC}"); os.system(f"pkill -f {VOICE_EXEC}"); os.system(f"pkill -f {GESTURE_EXEC}")
    time.sleep(1)

def main():
    if not os.path.exists(SETUP_BASH_PATH):
        print(f"{Colors.RED}错误: 请先 colcon build。{Colors.END}"); exit(1)

    while True:
        clear_screen()
        print(f"{Colors.HEADER}========================================{Colors.END}")
        print(f"{Colors.BOLD}       ME332 机器人中央控制台        {Colors.END}")
        print(f"{Colors.CYAN}  当前激活场景: {Colors.YELLOW}{CURRENT_WORLD_NAME}{Colors.END}")
        print(f"{Colors.HEADER}========================================{Colors.END}")
        print(f"  {Colors.CYAN}[1]{Colors.END} 启动 Gazebo 环境")
        print(f"  {Colors.CYAN}[2]{Colors.END} 启动 键盘控制 | {Colors.CYAN}[3]{Colors.END} 语音 | {Colors.CYAN}[4]{Colors.END} 手势")
        print("-" * 40)
        print(f"  {Colors.CYAN}[5]{Colors.END} 启动 SLAM 建图")
        print(f"  {Colors.CYAN}[6]{Colors.END} 保存地图 ")
        print(f"  {Colors.CYAN}[7]{Colors.END} 一键导航 (自动加载匹配地图)")
        print("-" * 40)
        print(f"  {Colors.CYAN}[0]{Colors.END} 安装依赖  {Colors.RED}[K]{Colors.END} 强杀进程  {Colors.RED}[Q]{Colors.END} 退出")
        
        choice = input(f"\n{Colors.BOLD}请输入指令 > {Colors.END}").lower()
        if choice == '1': launch_gazebo()
        elif choice == '2': launch_teleop()
        elif choice == '3': launch_voice()
        elif choice == '4': launch_gesture()
        elif choice == '5': launch_mapping()
        elif choice == '6': save_map()
        elif choice == '7': launch_navigation()
        elif choice == '0': install_dependencies()
        elif choice == 'k': kill_all()
        elif choice == 'q': break

if __name__ == "__main__":
    try: main()
    except KeyboardInterrupt: pass
