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
                             QGraphicsDropShadowEffect, QSpacerItem, QSizePolicy)
from PyQt5.QtCore import Qt, pyqtSignal, QObject, QRect
from PyQt5.QtGui import QPixmap, QFont, QColor, QPainter, QPen, QLinearGradient

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

    def state_callback(self, msg):
        if not self.initial_state_received:
            self.initial_state_received = True
            for i, name in enumerate(msg.name):
                if name in self.target_positions:
                    self.target_positions[name] = msg.position[i]
            self.comm.initialize_state_signal.emit(list(msg.name), list(msg.position))
            self.destroy_subscription(self.state_subscriber)

    def send_command(self, joint_name, target_angle_deg):
        self.target_positions[joint_name] = math.radians(target_angle_deg)
        if joint_name in HEAD_JOINTS:
            pub = self.pub_head
            joints = HEAD_JOINTS
        elif joint_name in ARM_JOINTS:
            pub = self.pub_arm
            joints = ARM_JOINTS
        elif joint_name in LEG_JOINTS:
            pub = self.pub_leg
            joints = LEG_JOINTS
        else: return

        msg = JointTrajectory()
        msg.joint_names = joints
        point = JointTrajectoryPoint()
        point.positions = [self.target_positions[name] for name in joints]
        point.time_from_start.nanosec = 150000000 
        msg.points = [point]
        pub.publish(msg)

class HUDBackgroundWidget(QWidget):
    """중앙 로봇 인터페이스 배경 위젯"""
    def __init__(self, parent=None):
        super().__init__(parent)
        self.image_path = self.find_image_in_directory()
        self.pixmap = QPixmap(self.image_path) if self.image_path else None

    def find_image_in_directory(self):
        script_dir = os.path.dirname(os.path.abspath(__file__))
        valid_ext = ['.png', '.jpg', '.jpeg', '.bmp']
        for file in os.listdir(script_dir):
            if any(file.lower().endswith(ext) for ext in valid_ext):
                return os.path.join(script_dir, file)
        return None

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        
        # 배경색
        painter.fillRect(self.rect(), QColor("#0B0E14"))
        
        # 그리드 패턴 효과 (사이버틱한 느낌)
        painter.setPen(QPen(QColor(137, 180, 250, 20), 1))
        for i in range(0, self.width(), 40):
            painter.drawLine(i, 0, i, self.height())
        for i in range(0, self.height(), 40):
            painter.drawLine(0, i, self.width(), i)

        if self.pixmap and not self.pixmap.isNull():
            # 로봇 이미지 렌더링
            scaled_w = int(self.width() * 0.4)
            scaled_pixmap = self.pixmap.scaled(scaled_w, self.height() - 100, Qt.KeepAspectRatio, Qt.SmoothTransformation)
            x = (self.width() - scaled_pixmap.width()) // 2
            y = (self.height() - scaled_pixmap.height()) // 2
            
            # 글로우 프레임 효과
            painter.setPen(QPen(QColor("#89B4FA"), 2))
            painter.drawRect(x - 5, y - 5, scaled_pixmap.width() + 10, scaled_pixmap.height() + 10)
            
            painter.setOpacity(0.9)
            painter.drawPixmap(x, y, scaled_pixmap)
            painter.setOpacity(1.0)
            
            # 모서리 타겟 마크
            size = 20
            painter.setPen(QPen(QColor("#A6E3A1"), 3))
            painter.drawLine(x-5, y-5, x-5+size, y-5) # Top-Left
            painter.drawLine(x-5, y-5, x-5, y-5+size)
            
            painter.drawLine(x+5+scaled_pixmap.width(), y-5, x+5+scaled_pixmap.width()-size, y-5) # Top-Right
            painter.drawLine(x+5+scaled_pixmap.width(), y-5, x+5+scaled_pixmap.width(), y-5+size)

class MotorDialWidget(QFrame):
    def __init__(self, joint_name, ros_node):
        super().__init__()
        self.joint_name = joint_name
        self.ros_node = ros_node
        limit = JOINT_LIMITS.get(joint_name, [-180.0, 180.0])
        self.min_deg, self.max_deg = limit
        self.init_ui()

    def init_ui(self):
        self.setFrameShape(QFrame.StyledPanel)
        self.setStyleSheet("""
            QFrame {
                background-color: rgba(30, 31, 46, 200);
                border: 1px solid rgba(137, 180, 250, 100);
                border-radius: 6px;
            }
            QLabel { border: none; background: transparent; }
        """)
        layout = QVBoxLayout()
        layout.setContentsMargins(4, 4, 4, 4)
        layout.setSpacing(0)
        
        name_lbl = QLabel(self.joint_name.split('_')[-1].upper())
        name_lbl.setAlignment(Qt.AlignCenter)
        name_lbl.setStyleSheet("font-size: 10px; color: #89B4FA; font-weight: bold;")
        
        self.dial = QDial()
        self.dial.setFixedSize(55, 55)
        self.dial.setRange(int(self.min_deg * 10), int(self.max_deg * 10))
        self.dial.setNotchesVisible(True)
        self.dial.valueChanged.connect(self.on_dial_changed)
        
        self.val_label = QLabel("0.0°")
        self.val_label.setAlignment(Qt.AlignCenter)
        self.val_label.setStyleSheet("font-size: 11px; color: #A6E3A1; font-weight: bold;")
        
        layout.addWidget(name_lbl)
        layout.addWidget(self.dial, alignment=Qt.AlignCenter)
        layout.addWidget(self.val_label)
        self.setLayout(layout)

    def on_dial_changed(self, value):
        deg = value / 10.0
        self.val_label.setText(f"{deg:.1f}°")
        self.ros_node.send_command(self.joint_name, deg)

    def set_initial_angle(self, rad):
        deg = math.degrees(rad)
        clamped = max(self.min_deg, min(self.max_deg, deg))
        self.dial.blockSignals(True)
        self.dial.setValue(int(clamped * 10))
        self.val_label.setText(f"{clamped:.1f}°")
        self.dial.blockSignals(False)

class MotorGUI(QWidget):
    def __init__(self, ros_node, comm):
        super().__init__()
        self.ros_node = ros_node
        self.comm = comm
        self.motor_widgets = {}
        self.init_ui()
        self.comm.initialize_state_signal.connect(self.initialize_states)

    def create_group(self, title, joints, cols=2):
        group = QGroupBox(title)
        group.setStyleSheet("""
            QGroupBox {
                color: #B4BEFE; font-weight: bold; font-size: 12px;
                border: 1px solid rgba(137, 180, 250, 50);
                border-radius: 10px; margin-top: 10px; padding-top: 10px;
                background-color: rgba(17, 17, 27, 150);
            }
            QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 5px; }
        """)
        layout = QGridLayout()
        layout.setSpacing(6)
        for i, name in enumerate(joints):
            w = MotorDialWidget(name, self.ros_node)
            self.motor_widgets[name] = w
            layout.addWidget(w, i // cols, i % cols)
        group.setLayout(layout)
        return group

    def init_ui(self):
        self.setWindowTitle("RHP MASTER CONTROL TERMINAL")
        self.resize(1280, 850)
        
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(0, 0, 0, 0)
        
        # 배경 HUD 위젯
        self.hud_bg = HUDBackgroundWidget(self)
        main_layout.addWidget(self.hud_bg)
        
        # 실제 컨트롤 레이아웃 (배경 위에 띄움)
        content_layout = QHBoxLayout(self.hud_bg)
        content_layout.setContentsMargins(25, 25, 25, 25)
        content_layout.setSpacing(20)
        
        # 왼쪽 패널 (L-ARM, L-LEG)
        left_side = QVBoxLayout()
        left_side.addWidget(self.create_group("LEFT UPPER (ARM)", L_ARM_JOINTS, cols=2))
        left_side.addStretch()
        left_side.addWidget(self.create_group("LEFT LOWER (LEG)", L_LEG_JOINTS, cols=2))
        
        # 중앙 패널 (HEAD)
        center_side = QVBoxLayout()
        head_group = self.create_group("CENTRAL UNIT (HEAD)", HEAD_JOINTS, cols=2)
        head_group.setFixedWidth(240)
        center_side.addWidget(head_group, alignment=Qt.AlignHCenter | Qt.AlignTop)
        
        # 시스템 상태 정보창 (앱 느낌 강화)
        status_box = QFrame()
        status_box.setFixedSize(240, 100)
        status_box.setStyleSheet("background: rgba(0,0,0,150); border: 1px solid #F38BA8; border-radius: 5px;")
        status_layout = QVBoxLayout(status_box)
        status_layout.addWidget(QLabel("● SYSTEM STATUS: ONLINE"), alignment=Qt.AlignLeft)
        status_layout.addWidget(QLabel("● MODE: MANUAL CALIBRATION"), alignment=Qt.AlignLeft)
        status_layout.addWidget(QLabel("● SIGNAL: 100% (LOCAL)"), alignment=Qt.AlignLeft)
        for i in range(status_layout.count()): status_layout.itemAt(i).widget().setStyleSheet("color: #F38BA8; font-size: 10px; font-weight: bold; border:none;")
        center_side.addStretch()
        center_side.addWidget(status_box, alignment=Qt.AlignHCenter | Qt.AlignBottom)
        
        # 오른쪽 패널 (R-ARM, R-LEG)
        right_side = QVBoxLayout()
        right_side.addWidget(self.create_group("RIGHT UPPER (ARM)", R_ARM_JOINTS, cols=2))
        right_side.addStretch()
        right_side.addWidget(self.create_group("RIGHT LOWER (LEG)", R_LEG_JOINTS, cols=2))
        
        content_layout.addLayout(left_side, 2)
        content_layout.addLayout(center_side, 3)
        content_layout.addLayout(right_side, 2)

    def initialize_states(self, names, positions):
        for n, p in zip(names, positions):
            if n in self.motor_widgets: self.motor_widgets[n].set_initial_angle(p)

def main():
    rclpy.init()
    app = QApplication(sys.argv)
    comm = RosComm()
    node = RosNode(comm)
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    gui = MotorGUI(node, comm)
    gui.show()
    app.exec_()
    rclpy.shutdown()

if __name__ == '__main__': main()