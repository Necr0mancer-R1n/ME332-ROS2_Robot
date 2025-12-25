# ME332 Mobile Manipulator Control System (ROS2 Humble)

本项目是一个基于 **ROS2 Humble** 开发的复合机器人系统，包含四轮移动底座与机械臂。项目不仅实现了经典的导航建图，还深度集成了 Gazebo 物理仿真插件与基于语音/手势的多模态交互控制。



## 🌟 项目亮点 (Highlights)

* **物理仿真优化**: 集成了 `gazebo_grasp_plugin`，解决了 Gazebo 仿真中机械臂抓取物体时容易滑落的物理难题。
* **多模态遥控 (AI Teleop)**: 在 `me332_teleop` 中集成了离线语音模型（包含声学模型 `am` 和 语言模型 `graph`），支持复杂的语音指令解析。
* **全栈导航能力**: 包含从机器人的 URDF 建模、传感器配置到 SLAM 建图及 Nav2 路径规划的全套流程。

## 📂 仓库结构说明 (Repository Structure)

项目主要由以下三个核心包组成：

* **`me332_robot_description`**: 
    * 包含机器人的 **URDF/Xacro** 物理模型。
    * 内置 `worlds` 仿真环境与 `maps` 已构建好的地图。
    * 提供完整的 `launch` 启动脚本，一键开启仿真环境。
* **`me332_teleop`**:
    * 核心控制逻辑所在。
    * 集成了语音识别模型资源（AM, Graph, Phones），实现非接触式语音交互。
    * 包含手势识别逻辑与移动/抓取控制脚本。
* **`gazebo-pkgs-humble`**:
    * 专为 ROS2 Humble 适配的 Gazebo 增强插件。
    * `gazebo_grasp_plugin`: 提供更真实的物体抓取判定。
    * `gazebo_version_helpers`: 解决不同 Gazebo 版本的兼容性问题。

## 🛠️ 技术栈 (Tech Stack)

* **环境**: Ubuntu 22.04 + ROS2 Humble
* **仿真器**: Gazebo Classic
* **核心组件**: Nav2, SLAM Toolbox, MoveIt2 (可选), OpenCV/MediaPipe (手势)
* **交互**: 离线语音识别 (Kaldi/Vosk based)

## 🚀 快速开始 (Quick Start)

1. **工作空间准备**:
   ```bash
   mkdir -p ~/me332_ws/src
   cd ~/me332_ws/src
   # 将本仓库克隆至此
   colcon build --symlink-install
   source install/setup.bash
   
2. **总控制台**:
   cd ~/me332_ws
   ./dashboard
