#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import os
import math
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy.qos import qos_profile_sensor_data

from PyQt5.QtWidgets import (QApplication, QWidget, QVBoxLayout, QHBoxLayout, 
                             QLabel, QDial, QGridLayout, QGroupBox, QFrame,
                             QGraphicsDropShadowEffect)
from PyQt5.QtCore import Qt, pyqtSignal, QObject
from PyQt5.QtGui import QPixmap, QFont, QColor

JOINT_STATES_TOPIC = '/joint_states'

# 로봇의 실제 3개 컨트롤러에 맞춰 조인트 그룹화
HEAD_JOINTS = ['head_pan', 'head_tilt']
L_ARM_JOINTS = ['l_sho_pitch', 'l_sho_roll', 'l_el', 'l_wst', 'l_grp']
R_ARM_JOINTS = ['r_sho_pitch', 'r_sho_roll', 'r_el', 'r_wst', 'r_grp']
L_LEG_JOINTS = ['l_hip_yaw', 'l_hip_roll', 'l_hip_pitch', 'l_knee', 'l_ank_pitch', 'l_ank_roll']
R_LEG_JOINTS = ['r_hip_yaw', 'r_hip_roll', 'r_hip_pitch', 'r_knee', 'r_ank_pitch', 'r_ank_roll']

# 전체 조인트 리스트
ARM_JOINTS = L_ARM_JOINTS + R_ARM_JOINTS
LEG_JOINTS = L_LEG_JOINTS + R_LEG_JOINTS
JOINT_NAMES = HEAD_JOINTS + ARM_JOINTS + LEG_JOINTS

# ==============================================================================
# [조인트 각도 리미트 하드코딩] 도(Degree) 단위 변환 완료
# ==============================================================================
JOINT_LIMITS = {
    'head_pan': [-120.0, 120.0], 'head_tilt': [-120.0, 120.0],
    
    'l_sho_pitch': [-120.0, 120.0], 'l_sho_roll': [-103.1, 103.1],
    'l_el': [-120.0, 120.0], 'l_wst': [-90.0, 120.0], 'l_grp': [-28.6, 11.5],
    
    'r_sho_pitch': [-120.0, 120.0], 'r_sho_roll': [-103.1, 103.1],
    'r_el': [-120.0, 120.0], 'r_wst': [-90.0, 120.0], 'r_grp': [-28.6, 11.5],
    
    'l_hip_yaw': [-120.0, 120.0], 'l_hip_roll': [-114.6, 114.6],
    'l_hip_pitch': [-31.5, 120.0], 'l_knee': [-120.0, 68.8],
    'l_ank_pitch': [-120.0, 120.0], 'l_ank_roll': [-108.9, 108.9],
    
    'r_hip_yaw': [-120.0, 120.0], 'r_hip_roll': [-114.6, 114.6],
    'r_hip_pitch': [-28.6, 120.0], 'r_knee': [-120.0, 68.8],
    'r_ank_pitch': [-120.0, 120.0], 'r_ank_roll': [-108.9, 108.9]
}

class RosComm(QObject):
    initialize_state_signal = pyqtSignal(list, list)

class RosNode(Node):
    def __init__(self, comm):
        super().__init__('motor_gui_controller')
        self.comm = comm
        self.initial_state_received = False
        self.target_positions = {name: 0.0 for name in JOINT_NAMES}
        
        self.pub_head = self.create_publisher(JointTrajectory, '/head_controller/joint_trajectory', 10)
        self.pub_arm = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.pub_leg = self.create_publisher(JointTrajectory, '/leg_controller/joint_trajectory', 10)
        
        self.state_subscriber = self.create_subscription(
            JointState, JOINT_STATES_TOPIC, self.state_callback, qos_profile_sensor_data
        )
        self.get_logger().info("GUI 준비 완료. 로봇의 초기 각도 데이터를 기다립니다...")

    def state_callback(self, msg):
        if not self.initial_state_received:
            self.get_logger().info("✅ 초기 각도 세팅 완료!")
            self.initial_state_received = True
            
            for i, name in enumerate(msg.name):
                if name in self.target_positions:
                    self.target_positions[name] = msg.position[i]
            
            self.comm.initialize_state_signal.emit(list(msg.name), list(msg.position))
            self.destroy_subscription(self.state_subscriber)

    def send_command(self, joint_name, target_angle_deg):
        self.target_positions[joint_name] = math.radians(target_angle_deg)
        
        if joint_name in HEAD_JOINTS:
            target_joints, publisher = HEAD_JOINTS, self.pub_head
        elif joint_name in ARM_JOINTS:
            target_joints, publisher = ARM_JOINTS, self.pub_arm
        elif joint_name in LEG_JOINTS:
            target_joints, publisher = LEG_JOINTS, self.pub_leg
        else:
            return

        msg = JointTrajectory()
        msg.joint_names = target_joints 
        
        point = JointTrajectoryPoint()
        point.positions = [self.target_positions[name] for name in target_joints]
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 150000000 
        
        msg.points = [point]
        publisher.publish(msg)

class MotorDialWidget(QFrame):
    """개별 모터를 위한 다이얼 위젯 (단일 컴포넌트)"""
    def __init__(self, joint_name, ros_node):
        super().__init__()
        self.joint_name = joint_name
        self.ros_node = ros_node
        
        limit = JOINT_LIMITS.get(joint_name, [-180.0, 180.0])
        self.min_deg = limit[0]
        self.max_deg = limit[1]
        
        self.init_ui()

    def init_ui(self):
        self.setFrameShape(QFrame.StyledPanel)
        self.setProperty("class", "dial-frame")
        layout = QVBoxLayout()
        layout.setSpacing(2)
        layout.setContentsMargins(5, 5, 5, 5)
        
        # 조인트 이름 라벨
        self.name_label = QLabel(self.joint_name.upper())
        self.name_label.setAlignment(Qt.AlignCenter)
        self.name_label.setProperty("class", "joint-name")
        
        # 다이얼 위젯
        self.dial = QDial()
        self.dial.setRange(int(self.min_deg * 10), int(self.max_deg * 10))
        self.dial.setValue(0)
        self.dial.setNotchesVisible(True) # 눈금 표시
        self.dial.setWrapping(False)
        self.dial.valueChanged.connect(self.on_dial_changed)
        
        # 값 표시 라벨 (최소, 현재, 최대)
        val_layout = QHBoxLayout()
        min_lbl = QLabel(f"{self.min_deg:.0f}°")
        min_lbl.setProperty("class", "limit-label")
        
        self.val_label = QLabel("0.0°")
        self.val_label.setAlignment(Qt.AlignCenter)
        self.val_label.setProperty("class", "value-label")
        
        max_lbl = QLabel(f"{self.max_deg:.0f}°")
        max_lbl.setProperty("class", "limit-label")
        max_lbl.setAlignment(Qt.AlignRight)
        
        val_layout.addWidget(min_lbl)
        val_layout.addWidget(self.val_label)
        val_layout.addWidget(max_lbl)
        
        layout.addWidget(self.name_label)
        layout.addWidget(self.dial)
        layout.addLayout(val_layout)
        self.setLayout(layout)

    def on_dial_changed(self, value):
        target_deg = value / 10.0
        self.val_label.setText(f"{target_deg:.1f}°")
        self.ros_node.send_command(self.joint_name, target_deg)

    def set_initial_angle(self, rad_value):
        current_deg = math.degrees(rad_value)
        clamped_deg = max(self.min_deg, min(self.max_deg, current_deg))
        
        self.dial.blockSignals(True)
        self.dial.setValue(int(clamped_deg * 10))
        self.val_label.setText(f"{clamped_deg:.1f}°")
        self.dial.blockSignals(False)

class MotorGUI(QWidget):
    def __init__(self, ros_node, comm):
        super().__init__()
        self.ros_node = ros_node
        self.comm = comm
        self.motor_widgets = {}
        
        self.apply_stylesheet()
        self.init_ui()
        
        self.comm.initialize_state_signal.connect(self.initialize_states)

    def apply_stylesheet(self):
        # 다크 테마 및 모던 UI 스타일 (QSS)
        self.setStyleSheet("""
            QWidget {
                background-color: #1E1E2E;
                color: #CDD6F4;
                font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            }
            QGroupBox {
                border: 2px solid #45475A;
                border-radius: 8px;
                margin-top: 15px;
                font-weight: bold;
                font-size: 14px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 15px;
                padding: 0 5px;
                color: #89B4FA;
            }
            QFrame.dial-frame {
                background-color: #24273A;
                border-radius: 8px;
                border: 1px solid #313244;
            }
            QDial {
                background-color: #313244;
            }
            QFrame.hud-frame {
                background-color: #11111B;
                border-radius: 15px;
                border: 2px solid #89B4FA; /* 사이버틱한 파란색 테두리 */
            }
            QLabel.joint-name {
                font-size: 11px;
                font-weight: bold;
                color: #A6ADC8;
            }
            QLabel.value-label {
                font-size: 13px;
                font-weight: bold;
                color: #A6E3A1; /* 형광 연두색 포인트 */
            }
            QLabel.limit-label {
                font-size: 10px;
                color: #6C7086;
            }
        """)

    def create_dial_group(self, title, joints, columns=2):
        """다이얼들을 그리드 형태로 묶어주는 그룹박스 생성"""
        group = QGroupBox(title)
        layout = QGridLayout()
        layout.setSpacing(10)
        
        for idx, name in enumerate(joints):
            widget = MotorDialWidget(name, self.ros_node)
            self.motor_widgets[name] = widget
            row, col = divmod(idx, columns)
            layout.addWidget(widget, row, col)
            
        group.setLayout(layout)
        return group

    def find_image_in_directory(self):
        """스크립트가 실행되는 폴더에서 첫 번째 이미지 파일을 자동으로 찾습니다."""
        script_dir = os.path.dirname(os.path.abspath(__file__))
        valid_extensions = ['.png', '.jpg', '.jpeg', '.bmp', '.gif']
        
        # 현재 디렉토리의 파일들을 순회하며 이미지 파일 찾기
        for file in os.listdir(script_dir):
            if any(file.lower().endswith(ext) for ext in valid_extensions):
                return os.path.join(script_dir, file)
        return None

    def init_ui(self):
        self.setWindowTitle("RHP Humanoid Master Control")
        # 넓은 해상도 지원 (공간 배치를 위해 창 크기 확대)
        self.resize(1100, 850)
        
        # 메인 레이아웃: 3열 구성 (좌측 팔/다리 | 중앙 머리/이미지 | 우측 팔/다리)
        main_layout = QHBoxLayout(self)
        
        # --- 왼쪽 열 (Left Arm, Left Leg) ---
        left_layout = QVBoxLayout()
        left_layout.addWidget(self.create_dial_group("Left Arm", L_ARM_JOINTS))
        left_layout.addWidget(self.create_dial_group("Left Leg", L_LEG_JOINTS))
        left_layout.addStretch()
        
        # --- 중앙 열 (Head, 로봇 이미지) ---
        center_layout = QVBoxLayout()
        center_layout.addWidget(self.create_dial_group("Head", HEAD_JOINTS, columns=2))
        
        # [앱 화면 스타일] 로봇 이미지 HUD 컨테이너 생성
        hud_frame = QFrame()
        hud_frame.setProperty("class", "hud-frame")
        hud_layout = QVBoxLayout(hud_frame)
        hud_layout.setContentsMargins(15, 15, 15, 15)
        
        # 글로우(네온 빛) 효과 추가
        glow = QGraphicsDropShadowEffect(self)
        glow.setBlurRadius(25)
        glow.setColor(QColor(137, 180, 250, 100)) # 은은한 하늘색 네온
        glow.setOffset(0, 0)
        hud_frame.setGraphicsEffect(glow)

        self.img_label = QLabel()
        self.img_label.setAlignment(Qt.AlignCenter)
        self.img_label.setMinimumSize(320, 500)
        
        # 폴더 내의 이미지 자동 검색 및 렌더링
        image_path = self.find_image_in_directory()
        pixmap = QPixmap(image_path) if image_path else QPixmap()
        
        if not pixmap.isNull():
            # 이미지가 성공적으로 불러와지면 앱 화면 비율에 맞춰 부드럽게 크기 조정
            self.img_label.setPixmap(pixmap.scaled(320, 500, Qt.KeepAspectRatio, Qt.SmoothTransformation))
        else:
            # 이미지가 없을 경우 표시되는 홀로그램 느낌의 텍스트
            self.img_label.setText("🤖\n\n[ SYSTEM OFFLINE ]\n\n로봇 이미지를 적용하려면\n스크립트와 같은 폴더에\n이미지 파일(.png, .jpg 등)을\n넣어주세요.")
            self.img_label.setStyleSheet("color: #F38BA8; font-size: 15px; font-weight: bold; letter-spacing: 1px;")

        hud_layout.addWidget(self.img_label)
        center_layout.addWidget(hud_frame)
        
        center_layout.addStretch()
        
        # --- 오른쪽 열 (Right Arm, Right Leg) ---
        right_layout = QVBoxLayout()
        right_layout.addWidget(self.create_dial_group("Right Arm", R_ARM_JOINTS))
        right_layout.addWidget(self.create_dial_group("Right Leg", R_LEG_JOINTS))
        right_layout.addStretch()
        
        # 각 레이아웃을 메인에 부착
        main_layout.addLayout(left_layout, 3)
        main_layout.addLayout(center_layout, 4)
        main_layout.addLayout(right_layout, 3)

    def initialize_states(self, joint_names, positions):
        for i, name in enumerate(joint_names):
            if name in self.motor_widgets:
                self.motor_widgets[name].set_initial_angle(positions[i])

def main():
    rclpy.init(args=None)
    app = QApplication(sys.argv)
    
    comm = RosComm()
    ros_node = RosNode(comm)
    
    ros_thread = threading.Thread(target=rclpy.spin, args=(ros_node,), daemon=True)
    ros_thread.start()
    
    gui = MotorGUI(ros_node, comm)
    gui.show()
    
    app.exec_()
    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()