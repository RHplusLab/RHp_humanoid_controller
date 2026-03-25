# rhphumanoid_walking

RHp 휴머노이드 로봇을 위한 ROS2 기반 보행 제어 패키지.
ROBOTIS OP3의 보행 알고리즘을 참고하여 RHp 로봇 사양에 맞게 재구현하였습니다.

---

## 개요

- **보행 방식**: 순수 기구학(open-loop) 보행. IMU/자이로 센서 피드백 없음
- **궤적 생성**: 사인파(wSin) 기반 ZMP 보행 궤적
- **역기구학**: OP3 closed-form IK 알고리즘 기반, RHp 링크 치수 적용
- **제어 주기**: 50Hz
- **통신 방식**: `/leg_controller/follow_joint_trajectory` Action (FollowJointTrajectory)

---

## 패키지 구조

```
rhphumanoid_walking/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/
│   └── rhphumanoid_walking_param.yaml     # 보행 파라미터
├── launch/
│   └── walking.launch.py                  # 노드 실행 launch 파일
├── include/rhphumanoid_walking/
│   ├── rhphumanoid_kinematics_dynamics_define.h   # 관절 ID 정의
│   ├── link_data.h                                # 링크 데이터 구조체
│   ├── rhphumanoid_kinematics_dynamics.h          # IK/FK 선언
│   ├── rhphumanoid_walking_parameter.h            # 보행 파라미터 구조체
│   └── rhphumanoid_walking_module.h               # 보행 모듈 선언
└── src/
    ├── link_data.cpp
    ├── rhphumanoid_kinematics_dynamics.cpp        # IK/FK 구현
    ├── rhphumanoid_walking_module.cpp             # 보행 로직 구현
    └── rhphumanoid_walking_node.cpp               # 노드 진입점
```

---

## 관절 구성

다리 관절 12개만 사용 (팔/머리 제외)

| 인덱스 | 관절 이름     |
|--------|--------------|
| 0      | r_hip_yaw    |
| 1      | r_hip_roll   |
| 2      | r_hip_pitch  |
| 3      | r_knee       |
| 4      | r_ank_pitch  |
| 5      | r_ank_roll   |
| 6      | l_hip_yaw    |
| 7      | l_hip_roll   |
| 8      | l_hip_pitch  |
| 9      | l_knee       |
| 10     | l_ank_pitch  |
| 11     | l_ank_roll   |

---

## RHp 링크 치수 (URDF 기준)

| 항목              | 값         |
|-------------------|------------|
| thigh_length      | 0.0721 m   |
| calf_length       | 0.0721 m   |
| ankle_length      | 0.0380 m   |
| leg_side_offset   | 0.06986 m  |
| hip_pitch_offset  | 0.0 m      |

---

## 토픽 / 액션 인터페이스

| 종류      | 이름                                        | 타입                                        | 방향  |
|-----------|---------------------------------------------|---------------------------------------------|-------|
| Subscribe | `/walking/command`                          | `std_msgs/msg/String`                       | 입력  |
| Action    | `/leg_controller/follow_joint_trajectory`   | `control_msgs/action/FollowJointTrajectory` | 출력  |

### 명령어

| 명령    | 동작                                          |
|---------|-----------------------------------------------|
| `start` | 차렷 자세(2초) → 보행 시작                   |
| `stop`  | 현재 스텝 완료 후 보행 종료                  |

---

## 보행 파라미터 (`rhphumanoid_walking_param.yaml`)

| 파라미터               | 기본값  | 설명                          |
|------------------------|---------|-------------------------------|
| `x_move_amplitude`     | 0.020 m | 전진 보폭 (Gazebo 튜닝 필요)  |
| `period_time`          | 600 ms  | 한 보행 사이클 시간           |
| `dsp_ratio`            | 0.2     | 양발 지지 구간 비율           |
| `foot_height`          | 0.020 m | 발 들어올리는 높이            |
| `swing_right_left`     | 0.018 m | 좌우 무게중심 이동량          |
| `swing_top_down`       | 0.005 m | 상하 무게중심 진동량          |
| `hip_pitch_offset`     | 10 deg  | 전방 경사 오프셋              |
| `pelvis_offset`        | 3 deg   | 외발 지지 시 골반 기울기      |

> 값 조정 후 노드 재시작 시 자동으로 yaml을 다시 읽습니다.

---

## 빌드

```bash
cd ~/your_ros2_ws
colcon build --packages-select rhphumanoid_walking
source install/setup.bash
```

---

## 가제보 테스트 방법

### 1. 가제보 + 로봇 실행

```bash
ros2 launch rhphumanoid_bringup rhphumanoid_bringup.launch.py use_sim:=true
```

`leg_controller`를 포함한 모든 컨트롤러가 자동으로 실행됩니다 (약 5초 소요).

### 2. 컨트롤러 확인

```bash
ros2 action list
# /leg_controller/follow_joint_trajectory 가 있어야 합니다
```

### 3. 보행 노드 실행

```bash
ros2 launch rhphumanoid_walking walking.launch.py
```

### 4. 보행 시작 / 정지

```bash
# 보행 시작
ros2 topic pub /walking/command std_msgs/msg/String "data: 'start'"

# 보행 정지
ros2 topic pub /walking/command std_msgs/msg/String "data: 'stop'"
```

---

## 파라미터 튜닝 방법 (Gazebo)

1. `config/rhphumanoid_walking_param.yaml` 에서 값 수정
2. 보행 노드 재시작

```bash
# Ctrl+C 로 노드 종료 후
ros2 launch rhphumanoid_walking walking.launch.py
```

`x_move_amplitude` 값부터 조정하면서 안정적인 전진 보폭을 찾는 것을 권장합니다.

---

## 참고

- OP3 보행 모듈: [ROBOTIS-GIT/ROBOTIS-OP3](https://github.com/ROBOTIS-GIT/ROBOTIS-OP3)
- RHp 로봇 리소스: `RHp_humanoid_resources-main/`
- 작동 확인된 통신 방식 참고: `rhphumanoid_walking_pattern/`
