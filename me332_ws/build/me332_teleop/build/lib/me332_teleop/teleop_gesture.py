#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import cv2
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
import numpy as np
import os
import sys

class GestureTeleop(Node):
    def __init__(self):
        super().__init__('gesture_teleop_node')
        
        # 1. 发布者配置
        self.pub_base = self.create_publisher(Twist, '/base_controller/cmd_vel_unstamped', 10)
        self.pub_grip = self.create_publisher(JointTrajectory, '/gripper_trajectory_controller/joint_trajectory', 10)
        
        # 2. 定位模型文件
        script_dir = os.path.dirname(os.path.realpath(__file__))
        model_path = os.path.join(script_dir, 'hand_landmarker.task')
        
        if not os.path.exists(model_path):
            self.get_logger().error(f"找不到模型文件: {model_path}")
            exit(1)

        # 3. 初始化 Mediapipe
        base_options = python.BaseOptions(model_asset_path=model_path)
        options = vision.HandLandmarkerOptions(
            base_options=base_options,
            num_hands=1,
            min_hand_detection_confidence=0.7,
            min_hand_presence_confidence=0.5,
            min_tracking_confidence=0.5
        )
        self.detector = vision.HandLandmarker.create_from_options(options)

        # 4. 控制变量
        self.cap = cv2.VideoCapture(0)
        self.max_linear_vel = 0.5   # 同步键盘速度
        self.max_angular_vel = 2.0  
        
        # 夹爪参数 (同步键盘版: 平移夹爪, 0-0.04m)
        self.grip_active = 0.0
        self.grip_limits = [0.0, 0.04]
        
        self.curr_v = 0.0
        self.curr_w = 0.0
        self.is_active = False

        self.print_console_help()
        self.create_timer(0.05, self.update_gesture)

    def print_console_help(self):
        """打印详细的终端控制说明"""
        os.system('clear')
        print("\033[95m=== AI 手势控制 (GESTURE TELEOP) ===\033[0m")
        print("\033[93m[控制范围]\033[0m")
        print("  1. 移动底盘 (Base) - 仅限 X/Y 平面")
        print("  2. 工业夹爪 (Gripper) - 平移开合")
        print("  * 注意：此模式暂不控制机械臂关节，以免误触 *")
        print("")
        print("\033[96m[手势指令说明]\033[0m")
        print("  \033[92m● 食指 + 拇指 (捏合)\033[0m : \033[1m控制底盘移动\033[0m")
        print("    - 手在该画面中心上方 -> 前进")
        print("    - 手在该画面中心下方 -> 后退")
        print("    - 手在该画面中心左侧 -> 左转")
        print("    - 手在该画面中心右侧 -> 右转")
        print("")
        print("  \033[92m● 中指 + 拇指 (捏合)\033[0m : \033[1m控制夹爪开合\033[0m")
        print("    - 距离越近 -> 夹爪闭合")
        print("    - 距离越远 -> 夹爪张开")
        print("")
        print("按 \033[91mCtrl+C\033[0m 退出程序")

    def pub_grip_cmd(self, pos):
        """发布夹爪位置指令 (对称平移)"""
        msg = JointTrajectory()
        msg.joint_names = ["finger1_joint1", "finger2_joint1"]
        p = JointTrajectoryPoint()
        # 对称控制：左正右负 (与键盘版一致)
        p.positions = [float(pos), float(-pos)]
        p.time_from_start.nanosec = int(0.1 * 1e9)
        msg.points = [p]
        self.pub_grip.publish(msg)

    def draw_ui(self, img, landmarks=None):
        h, w, _ = img.shape
        cx, cy = w // 2, h // 2
        font = cv2.FONT_HERSHEY_SIMPLEX

        # 绘制背景装饰
        cv2.rectangle(img, (10, 10), (320, 150), (40, 40, 40), -1)
        cv2.rectangle(img, (10, 10), (320, 150), (200, 200, 200), 2)
        
        cv2.putText(img, "GESTURE COMMANDS:", (20, 35), font, 0.6, (0, 255, 255), 2)
        
        # 状态指示灯
        color_move = (0, 255, 0) if self.is_active else (100, 100, 100)
        cv2.circle(img, (30, 65), 5, color_move, -1)
        cv2.putText(img, "Index+Thumb: BASE MOVE", (45, 70), font, 0.5, (255, 255, 255), 1)
        
        cv2.circle(img, (30, 95), 5, (255, 100, 0), -1)
        cv2.putText(img, "Middle+Thumb: GRIPPER", (45, 100), font, 0.5, (255, 255, 255), 1)
        
        # 夹爪数值显示
        grip_mm = self.grip_active * 2000 # 转换为 mm (0.04m -> 80mm total gap approx)
        cv2.putText(img, f"Gap: {grip_mm:.1f} mm", (45, 130), font, 0.5, (0, 200, 255), 1)

        # 绘制中心准星
        cv2.line(img, (cx - 20, cy), (cx + 20, cy), (255, 255, 255), 1)
        cv2.line(img, (cx, cy - 20), (cx, cy + 20), (255, 255, 255), 1)

        if landmarks:
            # 标记食指(底盘)和中指(夹爪)
            idx_p = (int(landmarks[8].x * w), int(landmarks[8].y * h))
            mid_p = (int(landmarks[12].x * w), int(landmarks[12].y * h))
            thb_p = (int(landmarks[4].x * w), int(landmarks[4].y * h))
            
            # 画出手部连线
            cv2.line(img, thb_p, idx_p, (0, 255, 0), 2)  # 食指连线
            cv2.line(img, thb_p, mid_p, (255, 100, 0), 2) # 中指连线
            cv2.circle(img, idx_p, 6, (0, 255, 0), -1)
            cv2.circle(img, mid_p, 6, (255, 100, 0), -1)

            if self.is_active:
                cv2.line(img, (cx, cy), idx_p, (0, 255, 0), 3)

    def update_gesture(self):
        success, img = self.cap.read()
        if not success: return

        img = cv2.flip(img, 1)
        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=img_rgb)
        detection_result = self.detector.detect(mp_image)

        base_msg = Twist()
        self.is_active = False
        hand_lms = None

        if detection_result.hand_landmarks:
            hand_lms = detection_result.hand_landmarks[0]
            
            t_tip = hand_lms[4]  # 大拇指
            i_tip = hand_lms[8]  # 食指
            m_tip = hand_lms[12] # 中指
            
            # --- A. 底盘控制逻辑 (食指 + 大拇指) ---
            # 只有当两个手指捏合时才激活
            dist_move = np.sqrt((i_tip.x - t_tip.x)**2 + (i_tip.y - t_tip.y)**2)
            if dist_move < 0.05:
                self.is_active = True
                # 以画面中心为原点计算速度
                offset_x = i_tip.x - 0.5
                offset_y = 0.5 - i_tip.y # Y轴反转
                
                # 增加死区，防止漂移
                if abs(offset_y) < 0.05: offset_y = 0
                if abs(offset_x) < 0.05: offset_x = 0
                
                self.curr_v = float(np.clip(offset_y * 2.5, -1.0, 1.0) * self.max_linear_vel)
                self.curr_w = float(np.clip(-offset_x * 3.5, -1.0, 1.0) * self.max_angular_vel)
                base_msg.linear.x = self.curr_v
                base_msg.angular.z = self.curr_w
            else:
                self.curr_v, self.curr_w = 0.0, 0.0

            # --- B. 夹爪控制逻辑 (中指 + 大拇指) ---
            dist_grip = np.sqrt((m_tip.x - t_tip.x)**2 + (m_tip.y - t_tip.y)**2)
            # 映射距离到 0~0.04m (修正了映射逻辑以匹配新夹爪)
            # 0.03 (闭合) <-> 0.18 (张开)
            target_grip = (dist_grip - 0.03) / (0.18 - 0.03) 
            target_grip = np.clip(target_grip, 0.0, 1.0) * self.grip_limits[1]
            
            if abs(target_grip - self.grip_active) > 0.001:
                self.grip_active = target_grip
                self.pub_grip_cmd(self.grip_active)

        self.pub_base.publish(base_msg)
        self.draw_ui(img, hand_lms)
        cv2.imshow("Gesture Control HUD", img)
        cv2.waitKey(1)

    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()

def main():
    rclpy.init()
    node = GestureTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
