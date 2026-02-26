#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import math
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy.qos import qos_profile_sensor_data

from PyQt5.QtWidgets import (QApplication, QWidget, QVBoxLayout, QHBoxLayout, 
                             QLabel, QSlider, QScrollArea, QGroupBox)
from PyQt5.QtCore import Qt, pyqtSignal, QObject

JOINT_STATES_TOPIC = '/joint_states'

# 로봇의 실제 3개 컨트롤러에 맞춰 조인트 그룹화
HEAD_JOINTS = ['head_pan', 'head_tilt']
ARM_JOINTS = [
    'l_sho_pitch', 'l_sho_roll', 'l_el', 'l_wst', 'l_grp',
    'r_sho_pitch', 'r_sho_roll', 'r_el', 'r_wst', 'r_grp'
]
LEG_JOINTS = [
    'l_hip_yaw', 'l_hip_roll', 'l_hip_pitch', 'l_knee', 'l_ank_pitch', 'l_ank_roll',
    'r_hip_yaw', 'r_hip_roll', 'r_hip_pitch', 'r_knee', 'r_ank_pitch', 'r_ank_roll'
]

JOINT_NAMES = HEAD_JOINTS + ARM_JOINTS + LEG_JOINTS

# ==============================================================================
# [조인트 각도 리미트 하드코딩]
# 보내주신 URDF 내부의 xacro 수식(pi, rad 등)을 모두 도(Degree) 단위로 변환한 값입니다.
# 소수점 1자리까지 정확하게 매핑되었습니다.
# ==============================================================================
JOINT_LIMITS = {
    # 머리
    'head_pan': [-120.0, 120.0],    # -pi*2/3 ~ pi*2/3
    'head_tilt': [-120.0, 120.0],   # -pi*2/3 ~ pi*2/3
    
    # 왼팔
    'l_sho_pitch': [-120.0, 120.0], # -pi*2/3 ~ pi*2/3
    'l_sho_roll': [-103.1, 103.1],  # -1.8 ~ 1.8
    'l_el': [-120.0, 120.0],        # -pi*2/3 ~ pi*2/3
    'l_wst': [-90.0, 120.0],        # -pi/2 ~ pi*2/3
    'l_grp': [-28.6, 11.5],         # -0.5 ~ 0.2
    
    # 오른팔
    'r_sho_pitch': [-120.0, 120.0], # -pi*2/3 ~ pi*2/3
    'r_sho_roll': [-103.1, 103.1],  # -1.8 ~ 1.8
    'r_el': [-120.0, 120.0],        # -pi*2/3 ~ pi*2/3
    'r_wst': [-90.0, 120.0],        # -pi/2 ~ pi*2/3
    'r_grp': [-28.6, 11.5],         # -0.5 ~ 0.2
    
    # 왼다리
    'l_hip_yaw': [-120.0, 120.0],   # -pi*2/3 ~ pi*2/3
    'l_hip_roll': [-114.6, 114.6],  # -2.0 ~ 2.0
    'l_hip_pitch': [-31.5, 120.0],  # -0.55 ~ pi*2/3
    'l_knee': [-120.0, 68.8],       # -pi*2/3 ~ 1.2
    'l_ank_pitch': [-120.0, 120.0], # -pi*2/3 ~ pi*2/3
    'l_ank_roll': [-108.9, 108.9],  # -1.9 ~ 1.9
    
    # 오른다리
    'r_hip_yaw': [-120.0, 120.0],   # -pi*2/3 ~ pi*2/3
    'r_hip_roll': [-114.6, 114.6],  # -2.0 ~ 2.0
    'r_hip_pitch': [-28.6, 120.0],  # -0.5 ~ pi*2/3
    'r_knee': [-120.0, 68.8],       # -pi*2/3 ~ 1.2
    'r_ank_pitch': [-120.0, 120.0], # -pi*2/3 ~ pi*2/3
    'r_ank_roll': [-108.9, 108.9]   # -1.9 ~ 1.9
}

class RosComm(QObject):
    # 시그널: 로봇 초기 각도 연동
    initialize_state_signal = pyqtSignal(list, list)

class RosNode(Node):
    def __init__(self, comm):
        super().__init__('motor_gui_controller')
        self.comm = comm
        self.initial_state_received = False
        
        self.target_positions = {name: 0.0 for name in JOINT_NAMES}
        
        # 퍼블리셔 (명령 전송)
        self.pub_head = self.create_publisher(JointTrajectory, '/head_controller/joint_trajectory', 10)
        self.pub_arm = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.pub_leg = self.create_publisher(JointTrajectory, '/leg_controller/joint_trajectory', 10)
        
        # 초기 각도 읽기용 서브스크라이버
        self.state_subscriber = self.create_subscription(
            JointState, JOINT_STATES_TOPIC, self.state_callback, qos_profile_sensor_data
        )
        self.get_logger().info("GUI 준비 완료. 로봇의 초기 각도 데이터를 기다립니다...")

    def state_callback(self, msg):
        if not self.initial_state_received:
            self.get_logger().info("✅ 초기 각도 세팅 완료! 통신망을 해지하고 명령 전송에만 집중합니다.")
            self.initial_state_received = True
            
            for i, name in enumerate(msg.name):
                if name in self.target_positions:
                    self.target_positions[name] = msg.position[i]
            
            self.comm.initialize_state_signal.emit(list(msg.name), list(msg.position))
            
            # 한 번만 초기화하고 구독 해지 (네트워크 리소스 완벽 절약)
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
        point.time_from_start.nanosec = 150000000 # 응답 속도 최적화(150ms)
        
        msg.points = [point]
        publisher.publish(msg)


class MotorWidget(QGroupBox):
    def __init__(self, joint_name, ros_node):
        super().__init__(joint_name)
        self.joint_name = joint_name
        self.ros_node = ros_node
        
        # URDF에 기반한 리미트 즉시 적용
        limit = JOINT_LIMITS.get(joint_name, [-180.0, 180.0])
        self.min_deg = limit[0]
        self.max_deg = limit[1]
        
        layout = QHBoxLayout()
        
        self.min_label = QLabel(f"{self.min_deg:.1f}°")
        self.min_label.setFixedWidth(50)
        self.min_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        
        self.slider = QSlider(Qt.Horizontal)
        self.slider.setRange(int(self.min_deg * 10), int(self.max_deg * 10))
        self.slider.setValue(0)
        self.slider.valueChanged.connect(self.on_slider_changed)
        
        self.max_label = QLabel(f"{self.max_deg:.1f}°")
        self.max_label.setFixedWidth(50)
        
        self.val_label = QLabel("0.0°")
        self.val_label.setFixedWidth(60)
        self.val_label.setStyleSheet("font-weight: bold; color: #0055ff;")
        self.val_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        
        layout.addWidget(self.min_label)
        layout.addWidget(self.slider)
        layout.addWidget(self.max_label)
        layout.addWidget(self.val_label)
        self.setLayout(layout)

    def on_slider_changed(self, value):
        target_deg = value / 10.0
        self.val_label.setText(f"{target_deg:.1f}°")
        self.ros_node.send_command(self.joint_name, target_deg)

    def set_initial_angle(self, rad_value):
        current_deg = math.degrees(rad_value)
        # 로봇 현재 각도가 URDF 리미트를 벗어났을 경우 리미트 선에 맞춤
        clamped_deg = max(self.min_deg, min(self.max_deg, current_deg))
        
        self.slider.blockSignals(True)
        self.slider.setValue(int(clamped_deg * 10))
        self.val_label.setText(f"{clamped_deg:.1f}°")
        self.slider.blockSignals(False)


class MotorGUI(QWidget):
    def __init__(self, ros_node, comm):
        super().__init__()
        self.ros_node = ros_node
        self.comm = comm
        self.motor_widgets = {}
        self.init_ui()
        
        # 초기화 시그널 연결
        self.comm.initialize_state_signal.connect(self.initialize_states)

    def init_ui(self):
        self.setWindowTitle("RHP 휴머노이드 모터 제어 패널")
        self.resize(550, 800)
        main_layout = QVBoxLayout(self)
        scroll_area = QScrollArea()
        scroll_area.setWidgetResizable(True)
        scroll_content = QWidget()
        scroll_layout = QVBoxLayout(scroll_content)
        
        for name in JOINT_NAMES:
            widget = MotorWidget(name, self.ros_node)
            self.motor_widgets[name] = widget
            scroll_layout.addWidget(widget)
            
        scroll_content.setLayout(scroll_layout)
        scroll_area.setWidget(scroll_content)
        main_layout.addWidget(scroll_area)

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