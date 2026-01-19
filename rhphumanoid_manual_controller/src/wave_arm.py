#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class WaveArmNode(Node):
    def __init__(self):
        super().__init__('wave_arm_node')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        self.publisher_ = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.timer = self.create_timer(2.0, self.send_trajectory)

        # [추가] 방향 전환을 위한 상태 변수
        self.direction = 1.0
        self.joints = ['l_sho_pitch', 'l_sho_roll', 'l_el', 'l_wst', 'l_grp',
                       'r_sho_pitch', 'r_sho_roll', 'r_el', 'r_wst', 'r_grp']

    def send_trajectory(self):
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = self.joints

        # [수정] 목표 각도를 0.8과 -0.8로 번갈아 설정
        target_pos = 0.8 * self.direction
        self.direction *= -1.0 # 다음 실행 때 반대 방향으로

        point = JointTrajectoryPoint()
        # 오른팔 pitch 위치에 target_pos 적용
        point.positions = [0.0, 0.0, 0.0, 0.0, 0.0,  target_pos, 0.0, 0.0, 0.0, 0.0]
        point.time_from_start = Duration(sec=1, nanosec=500000000)

        msg.points.append(point)
        self.publisher_.publish(msg)
        self.get_logger().info(f'목표 각도 {target_pos} 전송 완료!')

def main(args=None):
    rclpy.init(args=args)
    node = WaveArmNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 종료 시 중복 shutdown 방지를 위해 안전하게 처리
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
