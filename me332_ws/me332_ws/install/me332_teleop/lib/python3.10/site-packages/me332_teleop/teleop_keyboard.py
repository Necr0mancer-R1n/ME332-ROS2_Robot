#!/usr/bin/env python3
import sys
import select
import termios
import tty
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool

# ================= 配置区域 =================

MOVE_SPEED = 0.5    # 前进/后退速度 (m/s)
TURN_SPEED = 2.5    # 旋转速度 (rad/s)

STEP_ARM = 0.05     # 机械臂步进 (rad)
STEP_GRIP = 0.002    # 夹爪步进 (m) - 因为是平移，单位是米

# 联动控制目标角度（手掌相对于地面的俯仰角）
TARGET_PITCH_DOWN = -1.57 

# 自动停止的超时时间
AUTO_STOP_DELAY = 0.1 

class Colors:
    HEADER = '\033[95m'
    CYAN = '\033[96m'
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    RED = '\033[91m'
    BOLD = '\033[1m'
    END = '\033[0m'

# ================= 辅助类 =================

class KeyboardInput:
    def __init__(self):
        self.fd = sys.stdin.fileno()
        self.old_settings = termios.tcgetattr(self.fd)
        tty.setraw(self.fd)
    def restore(self):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)
    def read(self):
        dr, _, _ = select.select([sys.stdin], [], [], 0.0)
        if dr: return sys.stdin.read(1)
        return None

def clamp(x, lo, hi):
    return max(lo, min(hi, x))

# ================= ROS节点 =================

class RobotTeleop(Node):
    def __init__(self):
        super().__init__("me332_teleop_industrial")

        self.sub_joints = self.create_subscription(
            JointState, "/joint_states", self.joint_state_callback, 10)

        self.pub_base = self.create_publisher(Twist, "/base_controller/cmd_vel_unstamped", 10)
        self.pub_arm = self.create_publisher(JointTrajectory, "/arm_trajectory_controller/joint_trajectory", 10)
        self.pub_grip = self.create_publisher(JointTrajectory, "/gripper_trajectory_controller/joint_trajectory", 10)

        self.current_joints = {}
        
        # 初始机械臂状态
        self.arm_names = ["joint1", "joint2", "joint3"]
        self.arm_target = [0.0, 0.0, 0.0]
        self.arm_limits = [(-3.14, 3.14), (-0.75, 1.57), (-1.57,0.0)]
        
        # [核心修改] 适配平移夹爪
        self.grip_names = ["finger1_joint1", "finger2_joint1"]
        self.grip_active = 0.0  # 这里的变量代表单侧手指的位移
        self.grip_limits = [0.0, 0.04] # 单侧位移范围 0 到 2.5cm
        
        self.initial_state_received = False
        
        # 运动状态记录
        self.last_key_time = time.time()
        self.cmd_v = 0.0
        self.cmd_w = 0.0

        self.kb = KeyboardInput()
        
        sys.stdout.write("\x1b[?25l\x1b[2J\x1b[H")
        sys.stdout.flush()
        
        self.print_ui()
        self.create_timer(0.02, self.control_loop) 

    def joint_state_callback(self, msg):
        for name, pos in zip(msg.name, msg.position):
            self.current_joints[name] = pos
        if not self.initial_state_received:
            try:
                self.arm_target = [self.current_joints[n] for n in self.arm_names]
                # 获取左侧手指的当前位置作为初始值
                self.grip_active = self.current_joints["finger1_joint1"]
                self.initial_state_received = True
            except KeyError:
                pass 

    def destroy_node(self):
        self.pub_twist(0.0, 0.0)
        self.kb.restore()
        sys.stdout.write("\x1b[?25h\n")
        sys.stdout.flush()
        super().destroy_node()

    def calc_parallel_joint3(self, j2_val):
        target_j3 = TARGET_PITCH_DOWN - j2_val
        return clamp(target_j3, self.arm_limits[2][0], self.arm_limits[2][1])

    def pub_twist(self, v, w):
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.pub_base.publish(msg)

    def pub_arm_cmd(self):
        if not self.initial_state_received: return
        msg = JointTrajectory()
        msg.joint_names = self.arm_names
        p = JointTrajectoryPoint()
        p.positions = [float(x) for x in self.arm_target]
        p.time_from_start.sec = 0
        p.time_from_start.nanosec = int(0.1 * 1e9) 
        msg.points = [p]
        self.pub_arm.publish(msg)

    def pub_grip_cmd(self):
        if not self.initial_state_received: return
        # 对称控制逻辑：左手(f1)位移为正，右手(f2)位移为负
        pos_left = self.grip_active
        pos_right = -self.grip_active
        
        msg = JointTrajectory()
        msg.joint_names = self.grip_names
        p = JointTrajectoryPoint()
        p.positions = [float(pos_left), float(pos_right)]
        p.time_from_start.sec = 0
        p.time_from_start.nanosec = int(0.1 * 1e9)
        msg.points = [p]
        self.pub_grip.publish(msg)
        
    def print_ui(self):
        sys.stdout.write("\033[H")
        C = Colors.CYAN
        G = Colors.GREEN
        Y = Colors.YELLOW
        E = Colors.END
        H = Colors.HEADER

        if abs(self.cmd_v) > 0.01: v_str = f"{self.cmd_v:+.2f}"
        else: v_str = " 0.00"
        
        if abs(self.cmd_w) > 0.01: w_str = f"{self.cmd_w:+.2f}"
        else: w_str = " 0.00"

        if self.initial_state_received:
            arm_str = f"[{self.arm_target[0]:.2f}, {self.arm_target[1]:.2f}, {self.arm_target[2]:.2f}]"
            curr_pitch = self.arm_target[1] + self.arm_target[2]
            pitch_err = abs(curr_pitch - TARGET_PITCH_DOWN)
            pitch_status = f"{G}(平行地面){E}" if pitch_err < 0.1 else ""
            grip_str = f"开合距: {self.grip_active*2000:.1f} mm" # 显示总开度
        else:
            arm_str = "[ 等待同步... ]            "
            grip_str = ""
            pitch_status = ""

        lines = [
            f"{H}=== INDUSTRIAL SLIDER CONTROL (HOLD TO MOVE) ==={E} ", 
            "",
            f"{Y}[ 底盘 BASE ]{E}              ",
            f"{C}W / S{E} : {G}前进/后退{E}               ",
            f"{C}A / D{E} : {G}左转/右转{E}               ",
            "",
            f"{Y}[ 机械臂 ARM ]{E} {pitch_status}           ",
            f"{C}1 / 2{E} : 底座旋转                     ",
            f"{C}3 / 4{E} : 大臂俯仰                     ",
            f"{C}5 / 6{E} : 腕部俯仰                     ",
            f"{G}G    {E} : {Y}一键朝下{E}                  ",
            f"{G}R / F{E} : {Y}联动升降 (Rise/Fall){E}      ",
            "",
            f"{Y}[ 工业夹爪 GRIPPER ]{E}                   ",
            f"{C}O / P{E} : {G}张开 / 闭合{E}    ",
            f"{C}[ / ]{E} : 快速全开 / 快速全闭            ",
            "",
            f"{H}------------------------------{E}       ",
            f"底盘速度: {G}{v_str}{E} m/s | {G}{w_str}{E} rad/s ", 
            f"关节角度: {G}{arm_str}{E}                    ", 
            f"夹爪状态: {G}{grip_str}{E}                    ",
        ]

        output = "\r\n".join(lines) + "\033[J"
        sys.stdout.write(output)
        sys.stdout.flush()

    def control_loop(self):
        update_ui = False
        key = self.kb.read()
        current_time = time.time()
        
        if key is not None:
            self.last_key_time = current_time

        # --- 底盘控制 ---
        if key == 'w':
            self.cmd_v = MOVE_SPEED; self.cmd_w = 0.0; update_ui = True
        elif key == 's':
            self.cmd_v = -MOVE_SPEED; self.cmd_w = 0.0; update_ui = True
        elif key == 'a':
            self.cmd_v = 0.0; self.cmd_w = TURN_SPEED; update_ui = True
        elif key == 'd':
            self.cmd_v = 0.0; self.cmd_w = -TURN_SPEED; update_ui = True
            
        if current_time - self.last_key_time > AUTO_STOP_DELAY:
            if self.cmd_v != 0.0 or self.cmd_w != 0.0:
                self.cmd_v = 0.0; self.cmd_w = 0.0; update_ui = True

        self.pub_twist(self.cmd_v, self.cmd_w)

        # --- 机械臂与夹爪控制 ---
        if self.initial_state_received and key is not None:
            arm_changed = False
            
            # 独立旋转关节控制
            if key == '1': self.arm_target[0] -= STEP_ARM; arm_changed=True
            elif key == '2': self.arm_target[0] += STEP_ARM; arm_changed=True
            elif key == '3': self.arm_target[1] -= STEP_ARM; arm_changed=True
            elif key == '4': self.arm_target[1] += STEP_ARM; arm_changed=True
            elif key == '5': self.arm_target[2] -= STEP_ARM; arm_changed=True
            elif key == '6': self.arm_target[2] += STEP_ARM; arm_changed=True
            
            # 联动逻辑保持不变
            elif key == 'g':
                self.arm_target[2] = self.calc_parallel_joint3(self.arm_target[1])
                arm_changed = True
            elif key == 'r':
                if self.arm_target[1] < self.arm_limits[1][1] - 0.001:
                    self.arm_target[1] += STEP_ARM
                    self.arm_target[2] = self.calc_parallel_joint3(self.arm_target[1])
                    arm_changed = True
            elif key == 'f':
                if self.arm_target[1] > self.arm_limits[1][0] + 0.001:
                    self.arm_target[1] -= STEP_ARM
                    self.arm_target[2] = self.calc_parallel_joint3(self.arm_target[1])
                    arm_changed = True

            if arm_changed:
                for i in range(3):
                    self.arm_target[i] = clamp(self.arm_target[i], self.arm_limits[i][0], self.arm_limits[i][1])
                self.pub_arm_cmd()
                update_ui = True

            # [平移夹爪控制逻辑]
            grip_changed = False
            if key == 'o': # 张开
                self.grip_active += STEP_GRIP
                grip_changed = True
            elif key == 'p': # 闭合
                self.grip_active -= STEP_GRIP
                grip_changed = True
            elif key == '[': # 快速全开
                self.grip_active = self.grip_limits[1]
                grip_changed = True 
            elif key == ']': # 快速全关
                self.grip_active = self.grip_limits[0]
                grip_changed = True 

            if grip_changed:
                self.grip_active = clamp(self.grip_active, self.grip_limits[0], self.grip_limits[1])
                self.pub_grip_cmd()
                update_ui = True
        
        if update_ui:
            self.print_ui()

def main():
    rclpy.init()
    node = RobotTeleop()
    try:
        while rclpy.ok(): rclpy.spin_once(node, timeout_sec=0.001)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == "__main__":
    main()
