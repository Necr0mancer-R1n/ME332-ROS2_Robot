# 🚀 4WD Mobile Manipulator with SLAM & AI Interaction

这是一个集成了**四轮移动底座**与**多自由度机械臂**的复合机器人项目。该项目基于 ROS2 开发，旨在实现从自主导航到多模态交互（语音+手势）的全套功能。



---

## ✨ 核心功能 (Key Features)

### 🗺️ 自主导航与建图
* **SLAM 建图**: 支持多种算法（如 Gmapping, Cartographer 或 SLAM Toolbox），实现高精度环境建模。
* **Nav2 导航**: 基于 ROS2 Navigation2 堆栈，实现路径规划、动态避障及精准定点移动。

### 🦾 机械臂作业
* **协同控制**: 机械臂与底座坐标系解耦，支持 MoveIt2 运动规划，可完成抓取、放置等任务。

### 🧠 智能交互 (AI Interaction)
* **语音控制**: 集成语音识别模块（如 Whisper 或 PocketSphinx），支持通过语音指令控制机器人移动与抓取。
* **手势控制**: 基于 MediaPipe/OpenCV 的手势识别，实现通过手势实时引导机器人或切换模式。

---

## 🛠️ 技术栈 (Tech Stack)

* **OS**: Ubuntu 22.04 + ROS2 (Humble/Foxy)
* **Hardware**: 
    * 底座: 4轮 (Mecanum/Differential) + 编码器电机
    * 传感器: RPLIDAR, 深度相机 (RealSense/Kinect), 麦克风阵列
    * 控制器: Jetson Nano / Raspberry Pi 4 / PC
* **Languages**: Python, C++

---

## 📂 项目结构 (Repository Structure)

* `/robot_description`: 机器人的 URDF/Xacro 模型文件。
* `/robot_slam`: SLAM 配置文件及启动项。
* `/robot_navigation`: Nav2 参数及地图文件。
* `/robot_ai`: 语音识别与手势识别算法模块。
* `/robot_arm`: 机械臂控制与 MoveIt 配置文件。
