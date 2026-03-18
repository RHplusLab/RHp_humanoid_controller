#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import os
import math
import threading
from datetime import datetime

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy.qos import qos_profile_sensor_data

from PyQt5.QtWidgets import (QApplication, QWidget, QVBoxLayout, QHBoxLayout, 
                             QLabel, QDial, QFrame, QPushButton)
from PyQt5.QtCore import Qt, pyqtSignal, QObject, QRect
from PyQt5.QtGui import QPixmap, QColor, QPainter, QPen

JOINT_STATES_TOPIC = '/joint_states'
PREFERRED_IMAGE_FILES = [
    'robot_reference.png',
    'robot_reference.jpg',
    'robot_reference.jpeg',
    'motor_reference.png',
    'motor_reference.jpg',
]

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
    status_signal = pyqtSignal(str)

class RosNode(Node):
    def __init__(self, comm):
        super().__init__('motor_gui_controller')
        self.comm = comm
        self.target_positions = {name: 0.0 for name in JOINT_NAMES}
        self.last_joint_state = None
        self.state_refresh_pending = True
        self.received_state_count = 0
        
        self.pub_head = self.create_publisher(JointTrajectory, '/head_controller/joint_trajectory', 10)
        self.pub_arm = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.pub_leg = self.create_publisher(JointTrajectory, '/leg_controller/joint_trajectory', 10)
        
        self.state_subscriber = self.create_subscription(
            JointState, JOINT_STATES_TOPIC, self.state_callback, qos_profile_sensor_data
        )

    def state_callback(self, msg):
        self.last_joint_state = (list(msg.name), list(msg.position))
        self.received_state_count += 1
        if self.state_refresh_pending:
            self.apply_joint_state(msg.name, msg.position)
            self.state_refresh_pending = False
            self.comm.status_signal.emit(self.build_status_message("applied next /joint_states"))

    def apply_joint_state(self, names, positions):
        for i, name in enumerate(names):
            if name in self.target_positions:
                self.target_positions[name] = positions[i]
        self.comm.initialize_state_signal.emit(list(names), list(positions))

    def build_status_message(self, prefix):
        timestamp = datetime.now().strftime('%H:%M:%S')
        return f"STATE SYNC: {prefix} | rx={self.received_state_count} | {timestamp}"

    def request_state_refresh(self):
        self.state_refresh_pending = True
        self.comm.status_signal.emit(self.build_status_message("waiting for next /joint_states"))

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

class RobotOverlayWidget(QWidget):
    JOINT_LAYOUT = {
        'head_pan': (0.47, 0.06),
        'head_tilt': (0.53, 0.06),
        'r_sho_pitch': (0.39, 0.23),
        'r_sho_roll': (0.27, 0.25),
        'r_el': (0.18, 0.33),
        'r_wst': (0.17, 0.45),
        'r_grp': (0.15, 0.60),
        'l_sho_pitch': (0.61, 0.23),
        'l_sho_roll': (0.73, 0.25),
        'l_el': (0.82, 0.33),
        'l_wst': (0.83, 0.45),
        'l_grp': (0.85, 0.60),
        'r_hip_yaw': (0.43, 0.43),
        'r_hip_roll': (0.35, 0.50),
        'r_hip_pitch': (0.33, 0.58),
        'r_knee': (0.33, 0.69),
        'r_ank_pitch': (0.36, 0.82),
        'r_ank_roll': (0.36, 0.93),
        'l_hip_yaw': (0.57, 0.43),
        'l_hip_roll': (0.65, 0.50),
        'l_hip_pitch': (0.67, 0.58),
        'l_knee': (0.67, 0.69),
        'l_ank_pitch': (0.64, 0.82),
        'l_ank_roll': (0.64, 0.93),
    }

    def __init__(self, parent=None):
        super().__init__(parent)
        self.image_path = self.find_image_in_directory()
        self.pixmap = QPixmap(self.image_path) if self.image_path else None
        self.motor_widgets = {}
        self.ros_node = None
        self.setMinimumSize(900, 760)

    def set_ros_node(self, ros_node):
        self.ros_node = ros_node
        for name in JOINT_NAMES:
            widget = MotorDialWidget(name, ros_node)
            widget.setParent(self)
            self.motor_widgets[name] = widget
        self.update_widget_positions()

    def find_image_in_directory(self):
        script_dir = os.path.dirname(os.path.abspath(__file__))
        valid_ext = ['.png', '.jpg', '.jpeg', '.bmp']
        for preferred_name in PREFERRED_IMAGE_FILES:
            preferred_path = os.path.join(script_dir, preferred_name)
            if os.path.exists(preferred_path):
                return preferred_path
        for file in os.listdir(script_dir):
            if any(file.lower().endswith(ext) for ext in valid_ext):
                return os.path.join(script_dir, file)
        return None

    def get_image_rect(self):
        target_rect = self.rect().adjusted(30, 30, -30, -30)
        if self.pixmap and not self.pixmap.isNull():
            scaled_size = self.pixmap.size()
            scaled_size.scale(target_rect.size(), Qt.KeepAspectRatio)
            x = target_rect.x() + (target_rect.width() - scaled_size.width()) // 2
            y = target_rect.y() + (target_rect.height() - scaled_size.height()) // 2
            return QRect(x, y, scaled_size.width(), scaled_size.height())
        return target_rect

    def update_widget_positions(self):
        if not self.motor_widgets:
            return

        image_rect = self.get_image_rect()
        widget_w = max(74, min(94, image_rect.width() // 10))
        widget_h = max(96, min(118, image_rect.height() // 6))

        for joint_name, widget in self.motor_widgets.items():
            rel_x, rel_y = self.JOINT_LAYOUT.get(joint_name, (0.5, 0.5))
            center_x = image_rect.x() + int(image_rect.width() * rel_x)
            center_y = image_rect.y() + int(image_rect.height() * rel_y)
            widget.setGeometry(center_x - widget_w // 2, center_y - widget_h // 2, widget_w, widget_h)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self.update_widget_positions()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        
        painter.fillRect(self.rect(), QColor("#0B0E14"))
        
        painter.setPen(QPen(QColor(137, 180, 250, 20), 1))
        for i in range(0, self.width(), 40):
            painter.drawLine(i, 0, i, self.height())
        for i in range(0, self.height(), 40):
            painter.drawLine(0, i, self.width(), i)

        if self.pixmap and not self.pixmap.isNull():
            image_rect = self.get_image_rect()
            scaled_pixmap = self.pixmap.scaled(image_rect.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
            
            painter.setPen(QPen(QColor("#89B4FA"), 2))
            painter.drawRect(image_rect.adjusted(-5, -5, 5, 5))
            
            painter.setOpacity(0.9)
            painter.drawPixmap(image_rect.x(), image_rect.y(), scaled_pixmap)
            painter.setOpacity(1.0)
            
            size = 20
            painter.setPen(QPen(QColor("#A6E3A1"), 3))
            painter.drawLine(image_rect.x()-5, image_rect.y()-5, image_rect.x()-5+size, image_rect.y()-5)
            painter.drawLine(image_rect.x()-5, image_rect.y()-5, image_rect.x()-5, image_rect.y()-5+size)
            
            painter.drawLine(image_rect.x()+5+scaled_pixmap.width(), image_rect.y()-5, image_rect.x()+5+scaled_pixmap.width()-size, image_rect.y()-5)
            painter.drawLine(image_rect.x()+5+scaled_pixmap.width(), image_rect.y()-5, image_rect.x()+5+scaled_pixmap.width(), image_rect.y()-5+size)

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
                background-color: rgba(15, 18, 28, 215);
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
        name_lbl.setWordWrap(True)
        name_lbl.setStyleSheet("font-size: 10px; color: #89B4FA; font-weight: bold;")
        
        self.dial = QDial()
        self.dial.setFixedSize(52, 52)
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
        self.init_ui()
        self.comm.initialize_state_signal.connect(self.initialize_states)
        self.comm.status_signal.connect(self.status_label.setText)

    def init_ui(self):
        self.setWindowTitle("RHP MASTER CONTROL TERMINAL")
        self.resize(1440, 900)
        self.setStyleSheet("background-color: #0B0E14;")
        
        main_layout = QHBoxLayout(self)
        main_layout.setContentsMargins(18, 18, 18, 18)
        main_layout.setSpacing(16)

        self.robot_overlay = RobotOverlayWidget(self)
        self.robot_overlay.set_ros_node(self.ros_node)
        self.motor_widgets = self.robot_overlay.motor_widgets
        main_layout.addWidget(self.robot_overlay, 1)

        side_panel = QFrame()
        side_panel.setFixedWidth(280)
        side_panel.setStyleSheet("""
            QFrame {
                background-color: rgba(17, 17, 27, 220);
                border: 1px solid rgba(137, 180, 250, 80);
                border-radius: 10px;
            }
            QLabel {
                color: #CDD6F4;
                border: none;
                background: transparent;
            }
            QPushButton {
                background-color: #89B4FA;
                color: #11111B;
                border: none;
                border-radius: 8px;
                padding: 10px 12px;
                font-weight: bold;
            }
            QPushButton:pressed {
                background-color: #74C7EC;
            }
        """)
        side_layout = QVBoxLayout(side_panel)
        side_layout.setContentsMargins(16, 16, 16, 16)
        side_layout.setSpacing(12)

        title = QLabel("RHP MASTER CONTROL")
        title.setStyleSheet("color: #B4BEFE; font-size: 18px; font-weight: bold;")
        title.setAlignment(Qt.AlignLeft)
        side_layout.addWidget(title)

        desc = QLabel(
            "이미지 위 관절 위치에 다이얼을 배치했습니다.\n"
            "시작 시 1회 상태를 반영하고, 이후에는 Update 버튼을 누를 때 다음 joint_states 메시지 1개를 받아 다시 반영합니다."
        )
        desc.setWordWrap(True)
        desc.setStyleSheet("font-size: 11px; color: #BAC2DE;")
        side_layout.addWidget(desc)

        update_button = QPushButton("Update Joint State")
        update_button.clicked.connect(self.request_state_refresh)
        side_layout.addWidget(update_button)

        self.status_label = QLabel("STATE SYNC: waiting for first /joint_states")
        self.status_label.setWordWrap(True)
        self.status_label.setStyleSheet("color: #A6E3A1; font-size: 11px; font-weight: bold;")
        side_layout.addWidget(self.status_label)

        image_name = os.path.basename(self.robot_overlay.image_path) if self.robot_overlay.image_path else "no image found"
        image_info = QLabel(f"IMAGE: {image_name}")
        image_info.setWordWrap(True)
        image_info.setStyleSheet("font-size: 11px; color: #89DCEB;")
        side_layout.addWidget(image_info)

        mapping_note = QLabel(
            "배치 기준:\n"
            "- 화면 왼쪽 상단: r_arm\n"
            "- 화면 오른쪽 상단: l_arm\n"
            "- 화면 왼쪽 하단: r_leg\n"
            "- 화면 오른쪽 하단: l_leg"
        )
        mapping_note.setWordWrap(True)
        mapping_note.setStyleSheet("font-size: 11px; color: #F9E2AF;")
        side_layout.addWidget(mapping_note)
        side_layout.addStretch()

        main_layout.addWidget(side_panel)

    def request_state_refresh(self):
        self.status_label.setText("STATE SYNC: waiting for next /joint_states")
        self.ros_node.request_state_refresh()

    def initialize_states(self, names, positions):
        for n, p in zip(names, positions):
            if n in self.motor_widgets:
                self.motor_widgets[n].set_initial_angle(p)

def main():
    rclpy.init()
    app = QApplication(sys.argv)
    comm = RosComm()
    node = RosNode(comm)
    gui = MotorGUI(node, comm)
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    gui.show()
    app.exec_()
    rclpy.shutdown()

if __name__ == '__main__': main()
