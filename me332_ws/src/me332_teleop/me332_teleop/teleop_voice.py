#!/usr/bin/env python3
import sys
import threading
import time
import json
import math
import os
from ctypes import *
from contextlib import contextmanager

import pyaudio
from vosk import Model, KaldiRecognizer

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState

# ================= 配置区域 =================
MOVE_SPEED = 0.4    # 语音模式下速度稍慢，确保安全
TURN_SPEED = 0.8    

STEP_ARM = 0.1      # 语音每次调整的弧度 (比键盘大，因为不能长按)
STEP_GRIP = 0.005   # 语音每次调整夹爪 5mm

TARGET_PITCH_DOWN = -1.57 

# 屏蔽 ALSA 错误信息
ERROR_HANDLER_FUNC = CFUNCTYPE(None, c_char_p, c_int, c_char_p, c_int, c_char_p)
def py_error_handler(filename, line, function, err, fmt):
    pass
c_error_handler = ERROR_HANDLER_FUNC(py_error_handler)

@contextmanager
def no_alsa_error():
    try:
        asound = cdll.LoadLibrary('libasound.so')
        asound.snd_lib_error_set_handler(c_error_handler)
        yield
        asound.snd_lib_error_set_handler(None)
    except:
        yield

script_dir = os.path.dirname(os.path.abspath(__file__))
MODEL_PATH = os.path.join(script_dir, "model")

# 语音指令映射表 (详细列表将在 UI 中打印)
CMD_MAP = {
    "停止": "STOP", "停": "STOP", "别动": "STOP",
    "前进": "BASE_FORWARD", "向前": "BASE_FORWARD",
    "后退": "BASE_BACKWARD", "向后": "BASE_BACKWARD",
    "左转": "BASE_LEFT", "向左": "BASE_LEFT",
    "右转": "BASE_RIGHT", "向右": "BASE_RIGHT",
    "底座左转": "ARM_J1_LEFT", "底座右转": "ARM_J1_RIGHT",
    "大臂抬起": "ARM_J2_UP", "大臂落下": "ARM_J2_DOWN",
    "小臂抬起": "ARM_J3_UP", "小臂落下": "ARM_J3_DOWN",
    "张开": "GRIP_OPEN", "松开": "GRIP_OPEN",
    "闭合": "GRIP_CLOSE", "抓紧": "GRIP_CLOSE"
}

class Colors:
    HEADER = '\033[95m'
    CYAN = '\033[96m'
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    RED = '\033[91m'
    BOLD = '\033[1m'
    END = '\033[0m'

class OfflineVoiceTeleop(Node):
    def __init__(self):
        super().__init__("me332_voice_industrial")

        if not os.path.exists(MODEL_PATH):
            print(f"{Colors.RED}错误：找不到模型文件夹！{Colors.END}")
            sys.exit(1)

        self.sub_joints = self.create_subscription(
            JointState, "/joint_states", self.joint_state_callback, 10)

        self.pub_base = self.create_publisher(Twist, "/base_controller/cmd_vel_unstamped", 10)
        self.pub_arm = self.create_publisher(JointTrajectory, "/arm_trajectory_controller/joint_trajectory", 10)
        self.pub_grip = self.create_publisher(JointTrajectory, "/gripper_trajectory_controller/joint_trajectory", 10)

        self.current_joints = {}
        # 机械臂参数 (同步键盘版)
        self.arm_names = ["joint1", "joint2", "joint3"]
        self.arm_target = [0.0, 0.0, 0.0]
        # [调整]: 将 Joint3 上限由 1.57 改为 0.0，与键盘版保持一致
        self.arm_limits = [(-3.14, 3.14), (-0.75, 1.57), (-1.57, 0.0)]
        
        # 夹爪参数 (同步键盘版：平移夹爪)
        self.grip_names = ["finger1_joint1", "finger2_joint1"]
        self.grip_active = 0.0  
        self.grip_limits = [0.0, 0.04]
        
        self.initial_state_received = False
        self.current_command = "WAITING..."
        self.last_text = ""

        print(f"{Colors.YELLOW}正在加载离线语音模型 (Vosk)...{Colors.END}")
        try:
            from vosk import SetLogLevel
            SetLogLevel(-1)
        except: pass

        try:
            self.vosk_model = Model(MODEL_PATH)
            self.recognizer = KaldiRecognizer(self.vosk_model, 16000)
        except Exception as e:
            print(f"{Colors.RED}模型加载失败: {e}{Colors.END}")
            sys.exit(1)

        self.voice_thread = threading.Thread(target=self.voice_listener_loop, daemon=True)
        self.voice_thread.start()
        
        self.create_timer(0.05, self.control_loop)
        self.print_ui()

    def joint_state_callback(self, msg):
        for name, pos in zip(msg.name, msg.position):
            self.current_joints[name] = pos
        if not self.initial_state_received:
            try:
                self.arm_target = [self.current_joints[n] for n in self.arm_names]
                self.grip_active = self.current_joints["finger1_joint1"]
                self.initial_state_received = True
            except KeyError:
                pass 

    def voice_listener_loop(self):
        with no_alsa_error():
            p = pyaudio.PyAudio()
        try:
            stream = p.open(format=pyaudio.paInt16, channels=1, rate=16000, input=True, frames_per_buffer=4000)
            stream.start_stream()
        except:
            self.last_text = "麦克风错误"
            return
        while rclpy.ok():
            try:
                data = stream.read(2000, exception_on_overflow=False)
                if self.recognizer.AcceptWaveform(data):
                    res = json.loads(self.recognizer.Result())
                    text = res.get("text", "").replace(" ", "")
                    if text: self.process_voice_command(text)
            except: pass

    def process_voice_command(self, text):
        self.last_text = text
        for keyword, action in CMD_MAP.items():
            if keyword in text:
                self.current_command = action
                break 
        self.print_ui()

    def calc_parallel_joint3(self, j2_val):
        target_j3 = TARGET_PITCH_DOWN - j2_val
        return max(self.arm_limits[2][0], min(self.arm_limits[2][1], target_j3))

    def control_loop(self):
        if not self.initial_state_received: return
        cmd = self.current_command
        
        # --- 1. 底盘控制 ---
        tv, tw = 0.0, 0.0
        if cmd == "BASE_FORWARD": tv = MOVE_SPEED
        elif cmd == "BASE_BACKWARD": tv = -MOVE_SPEED
        elif cmd == "BASE_LEFT": tw = TURN_SPEED
        elif cmd == "BASE_RIGHT": tw = -TURN_SPEED
        
        t = Twist()
        t.linear.x, t.angular.z = float(tv), float(tw)
        self.pub_base.publish(t)

        # --- 2. 机械臂控制 ---
        arm_changed = False
        hit_limit = False

        if cmd == "ARM_J1_LEFT": 
            self.arm_target[0] -= STEP_ARM; arm_changed = True
        elif cmd == "ARM_J1_RIGHT": 
            self.arm_target[0] += STEP_ARM; arm_changed = True
        elif cmd == "ARM_J2_UP": 
            # 联动逻辑：大臂升降时自动补偿小臂 (同键盘 R 键)
            self.arm_target[1] += STEP_ARM
            self.arm_target[2] = self.calc_parallel_joint3(self.arm_target[1])
            arm_changed = True
        elif cmd == "ARM_J2_DOWN": 
            # 联动逻辑 (同键盘 F 键)
            self.arm_target[1] -= STEP_ARM
            self.arm_target[2] = self.calc_parallel_joint3(self.arm_target[1])
            arm_changed = True
        elif cmd == "ARM_J3_UP": 
            self.arm_target[2] += STEP_ARM; arm_changed = True
        elif cmd == "ARM_J3_DOWN": 
            self.arm_target[2] -= STEP_ARM; arm_changed = True

        if arm_changed:
            for i in range(3):
                old_val = self.arm_target[i]
                self.arm_target[i] = max(self.arm_limits[i][0], min(self.arm_limits[i][1], self.arm_target[i]))
                if abs(old_val - self.arm_target[i]) > 0.001 and (self.arm_target[i] == self.arm_limits[i][0] or self.arm_target[i] == self.arm_limits[i][1]):
                    hit_limit = True
            
            msg = JointTrajectory()
            msg.joint_names = self.arm_names
            p = JointTrajectoryPoint()
            p.positions = [float(x) for x in self.arm_target]
            p.time_from_start.nanosec = int(0.1 * 1e9)
            msg.points = [p]
            self.pub_arm.publish(msg)

        # --- 3. 平移夹爪控制 ---
        grip_changed = False
        if cmd == "GRIP_OPEN": 
            self.grip_active += STEP_GRIP; grip_changed = True
        elif cmd == "GRIP_CLOSE": 
            self.grip_active -= STEP_GRIP; grip_changed = True

        if grip_changed:
            if self.grip_active > self.grip_limits[1]: self.grip_active = self.grip_limits[1]; hit_limit = True
            if self.grip_active < self.grip_limits[0]: self.grip_active = self.grip_limits[0]; hit_limit = True
            
            gmsg = JointTrajectory()
            gmsg.joint_names = self.grip_names
            gp = JointTrajectoryPoint()
            # 对称控制：左正右负
            gp.positions = [float(self.grip_active), float(-self.grip_active)]
            gp.time_from_start.nanosec = int(0.1 * 1e9)
            gmsg.points = [gp]
            self.pub_grip.publish(gmsg)

        if hit_limit:
            self.current_command = "STOP" # 到达限位自动停止，防止电机过载
            self.print_ui()

    def print_ui(self):
        sys.stdout.write("\033[H") # 光标回原点
        C, G, Y, R, E = Colors.CYAN, Colors.GREEN, Colors.YELLOW, Colors.RED, Colors.END
        
        status_c = R if self.current_command in ["STOP", "WAITING..."] else G
        curr_pitch = self.arm_target[1] + self.arm_target[2]
        pitch_status = f"{G}(平行){E}" if abs(curr_pitch - TARGET_PITCH_DOWN) < 0.15 else "      "

        lines = [
            f"{Colors.HEADER}=== ME332 智能语音控制台 (VOICE TELEOP) ==={E}",
            "",
            f"{Y}[ 可用语音指令列表 ]{E}",
            f"  {C}底盘移动{E}: 前进 / 后退 / 左转 / 右转 / 停止",
            f"  {C}机械臂  {E}: 大臂抬起 / 大臂落下 (带联动)",
            f"             小臂抬起 / 小臂落下 / 底座左转 / 底座右转",
            f"  {C}夹爪控制{E}: 张开 / 闭合",
            f"--------------------------------------------------",
            f"语音识别内容: {Y}{self.last_text:<20}{E}",
            f"当前执行指令: {status_c}{self.current_command:<15}{E} ",
            f"机械臂关节角: [{self.arm_target[0]:.2f}, {self.arm_target[1]:.2f}, {self.arm_target[2]:.2f}] {pitch_status}",
            f"工业夹爪开度: {G}{self.grip_active*2000:.1f} mm{E} (Max 80.0)",
            f"\033[J" # 清除剩余屏幕
        ]
        sys.stdout.write("\r\n".join(lines))
        sys.stdout.flush()

def main():
    rclpy.init()
    node = OfflineVoiceTeleop()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == "__main__":
    main()
