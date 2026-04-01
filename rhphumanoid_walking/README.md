# rhphumanoid_walking

RHp 휴머노이드 로봇을 위한 ROS2 기반 보행 제어 패키지.
ROBOTIS OP3의 보행 알고리즘을 참고하여 RHp 로봇 사양에 맞게 재구현하였습니다.

---

## 개요

- **보행 방식**: 순수 기구학(open-loop) 보행. IMU/자이로 센서 피드백 없음
- **궤적 생성**: 사인파(wSin) 기반 open-loop 궤적 생성 (ZMP 계산/측정 없음)
- **역기구학**: OP3 closed-form IK 알고리즘 기반, RHp 링크 치수 적용
- **제어 주기**: 50Hz (20ms × 30포인트 = 600ms/사이클)
- **통신 방식**: `/leg_controller/follow_joint_trajectory` Action (FollowJointTrajectory)
  - 한 보행 사이클(600ms, 30포인트)을 계산하여 action goal 하나로 전송
  - goal 완료 시 result callback에서 다음 사이클 자동 전송

---

## 보행 원리

### 사인파 궤적 생성

센서 없이 보행하기 위해 발 끝점(foot endpoint)과 몸통(body)의 움직임을 모두 사인파로 미리 계산합니다.
실제 로봇 상태를 측정하지 않으므로 파라미터 튜닝으로 안정성을 확보합니다.

핵심 함수:
```
wSin(time, period, phase_shift, amplitude, amplitude_shift)
  = amplitude * sin(2π / period * time - phase_shift) + amplitude_shift
```

이 함수를 x(전후), y(좌우), z(상하), yaw(회전) 방향에 각각 적용하여 발과 몸통의 궤적을 생성합니다.

### 궤적 계산 → 관절각 변환 흐름

```
보행 파라미터
    ↓
wSin() 으로 발 끝점 좌표(x, y, z, roll, pitch, yaw) 계산  ← computeLegAngle()
    ↓
Closed-form IK 로 관절각 12개 계산                         ← calcInverseKinematicsFor{Right,Left}Leg()
    ↓
JointTrajectoryPoint 으로 패킹 (positions + time_from_start)
    ↓
FollowJointTrajectory action goal 전송 (30포인트 묶음)
    ↓
ros2_control leg_controller 가 실제 관절 구동
```

### 제어 주기와 포인트 전송 방식

- **50Hz** 기준으로 20ms마다 관절각 1포인트
- 한 보행 사이클(600ms) = **30포인트**를 한꺼번에 계산하여 action 1개로 전송
- action goal이 완료(result callback)되면 즉시 다음 사이클을 계산·전송 → 연속 보행
- 도중에 `stop` 명령을 받으면 `ctrl_running_ = false`로 설정하고, 현재 사이클이 끝난 뒤 자연스럽게 종료

```
사이클 1 (30포인트, 0~600ms)
    └─ 전송 완료 → 사이클 2 즉시 계산·전송
사이클 2 (30포인트, 600~1200ms)
    └─ 전송 완료 → 사이클 3 즉시 계산·전송
    ...
```

---

## 보행 단계 (Phase) 상세

한 사이클(period_time = 600ms)은 **DSP/SSP** 구조로 구성됩니다.

- **DSP (Double Support Phase)**: 양발이 모두 지면에 닿아 있는 구간. `dsp_ratio = 0.2` 이면 전체 주기의 20%.
- **SSP (Single Support Phase)**: 한쪽 발이 공중에 떠 있는 구간. 나머지 80%.

한 사이클 내 5개 구간:

```
시간 →
|──DSP──|────L-SSP────|──DSP──|────R-SSP────|──DSP──|
0      t1            t2      t3            t4    600ms

t1 = l_ssp_start_time  (왼발 스윙 시작)
t2 = l_ssp_end_time    (왼발 착지)
t3 = r_ssp_start_time  (오른발 스윙 시작)
t4 = r_ssp_end_time    (오른발 착지)
```

| 구간 | 시간 범위 | 상태 | 각 발 동작 |
|------|-----------|------|-----------|
| DSP 1 | 0 ~ t1 | 양발 지지 | 양발 고정, 몸통만 이동 |
| L-SSP | t1 ~ t2 | 왼발 스윙 | 왼발 위로 들어 앞으로 이동, 오른발 고정 |
| DSP 2 | t2 ~ t3 | 양발 지지 | 양발 착지, 몸통 중앙으로 이동 |
| R-SSP | t3 ~ t4 | 오른발 스윙 | 오른발 위로 들어 앞으로 이동, 왼발 고정 |
| DSP 3 | t4 ~ end | 양발 지지 | 양발 착지, 다음 사이클 준비 |

**SSP 구간 중 발 z축(높이) 궤적**: `wSin`으로 반원형 포물선을 그리며 들어올렸다가 내려옴. 최대 높이는 `foot_height`.

**SSP 구간 중 몸통 좌우 이동(y-swap)**: 지지발 쪽으로 무게중심을 이동시켜 균형 유지. `swing_right_left`로 진폭 조절.

**SSP 구간 중 pelvis_offset**: 외발 지지 시 hip_roll 관절에 추가 오프셋을 넣어 골반을 기울여 측면 안정성 보강.

### Phase 전환 시점 (processPhase)

4개의 전환 시점(phase1~3)에서 파라미터를 갱신합니다:

| 전환 | 시점 | 동작 |
|------|------|------|
| PHASE0 | time=0 | 타이밍 파라미터 갱신, stop 여부 확인 |
| PHASE1 | t1 직후 | 이동 파라미터 갱신 (보폭 등 실시간 반영) |
| PHASE2 | DSP 중간 | 타이밍 파라미터 갱신, stop 여부 확인 |
| PHASE3 | t3 직후 | 이동 파라미터 갱신 |

---

## 움직이는 관절

다리 관절 12개만 제어합니다 (팔/머리 관절 미사용).

| 인덱스 | 관절 이름 | 역할 |
|--------|-----------|------|
| 0 | r_hip_yaw | 오른쪽 고관절 수평 회전 (좌우 방향 전환) |
| 1 | r_hip_roll | 오른쪽 고관절 좌우 기울기 (측면 균형) |
| 2 | r_hip_pitch | 오른쪽 고관절 전후 기울기 (전진/후진) |
| 3 | r_knee | 오른쪽 무릎 굽힘 (발 들어올릴 때 굽힘) |
| 4 | r_ank_pitch | 오른쪽 발목 전후 (착지 자세) |
| 5 | r_ank_roll | 오른쪽 발목 좌우 (지면 수평 유지) |
| 6 | l_hip_yaw | 왼쪽 고관절 수평 회전 |
| 7 | l_hip_roll | 왼쪽 고관절 좌우 기울기 |
| 8 | l_hip_pitch | 왼쪽 고관절 전후 기울기 |
| 9 | l_knee | 왼쪽 무릎 굽힘 |
| 10 | l_ank_pitch | 왼쪽 발목 전후 |
| 11 | l_ank_roll | 왼쪽 발목 좌우 |

IK 풀고 난 뒤 추가 보정:
- `hip_roll` ← pelvis_offset 보정 (SSP 구간 중 골반 기울기)
- `hip_pitch` ← hip_pitch_offset 보정 (전방 경사 자세)

---

## 동작 흐름

```
start 명령 수신
    └─ sendTrajectory(stand_pose, 2s)   # 모든 관절 0도, 차렷 자세 (action)
    └─ 2.2초 대기 (wall_timer)
    └─ startGaitCycle()
           └─ for i in 0..29:
               └─ processPhase()         # phase 전환 처리
               └─ computeLegAngle()      # 발 끝점 계산 → IK → 관절각
               └─ JointTrajectoryPoint 추가
           └─ async_send_goal (action, 30포인트)
           └─ result_callback
                  ├─ ctrl_running_=true  → startGaitCycle() 반복
                  └─ ctrl_running_=false → 종료

stop 명령 수신
    └─ ctrl_running_ = false
    └─ 현재 사이클 완료 후 자동 종료 (급정지 없음)
```

---

## 보행 파라미터 (`rhphumanoid_walking_param.yaml`)

### 자세 오프셋

| 파라미터 | 기본값 | 설명 | 튜닝 팁 |
|----------|--------|------|---------|
| `x_offset` | -0.015 m | 발 기준 몸통 전후 오프셋 | 음수 = 몸통 뒤로. 앞으로 넘어지면 더 음수로 |
| `y_offset` | 0.005 m | 발 기준 몸통 좌우 오프셋 | 좌우 비대칭 시 조정 |
| `z_offset` | 0.010 m | 발 기준 몸통 높이 오프셋 | 높이면 무릎 더 굽힘 |
| `hip_pitch_offset` | 5.0 deg | 고관절 전방 경사각 | 클수록 앞으로 기울어짐. 앞으로 넘어지면 줄임 |
| `pelvis_offset` | 3.0 deg | SSP 중 골반 기울기 | 클수록 측면 안정성 향상, 너무 크면 뒤뚱거림 |

### 타이밍

| 파라미터 | 기본값 | 설명 | 튜닝 팁 |
|----------|--------|------|---------|
| `period_time` | 600 ms | 한 보행 사이클 시간 | 길수록 느리고 안정적, 짧으면 빠르지만 불안정 |
| `dsp_ratio` | 0.2 | 양발 지지 구간 비율 (0~1) | 클수록 안정적, 작을수록 빠름 |
| `step_forward_back_ratio` | 0.28 | 몸통 전후 swap 진폭 비율 | x_move_amplitude 대비 몸통 이동 비율 |

### 스텝 크기

| 파라미터 | 기본값 | 설명 | 튜닝 팁 |
|----------|--------|------|---------|
| `x_move_amplitude` | 0.020 m | 전진 보폭 (한 발 기준) | 키우면 넓은 보폭, 너무 크면 넘어짐 |
| `foot_height` | 0.020 m | 발 들어올리는 최대 높이 | 너무 낮으면 발이 지면에 걸림 |

### 스윙

| 파라미터 | 기본값 | 설명 | 튜닝 팁 |
|----------|--------|------|---------|
| `swing_right_left` | 0.018 m | 몸통 좌우 진동 진폭 | 보폭 키우면 같이 키워야 함. 너무 작으면 옆으로 넘어짐 |
| `swing_top_down` | 0.005 m | 몸통 상하 진동 진폭 | 자연스러운 걸음새 조절 |

### 앞으로 넘어질 때 조정 순서

1. `hip_pitch_offset` 줄이기 (전방 기울기 감소)
2. `x_offset` 음수로 더 키우기 (몸통 뒤로)
3. 위 둘로 부족하면 `x_move_amplitude` 줄이기 (보폭 감소)

### 옆으로 넘어질 때 조정 순서

1. `swing_right_left` 키우기 (좌우 무게중심 이동 증가)
2. `pelvis_offset` 키우기 (골반 기울기 증가)

---

## 토픽 / 액션 인터페이스

| 종류 | 이름 | 타입 | 방향 |
|------|------|------|------|
| Subscribe | `/walking/command` | `std_msgs/msg/String` | 입력 |
| Action | `/leg_controller/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` | 출력 |

### 명령어

| 명령 | 동작 |
|------|------|
| `start` | 차렷 자세(2초) → 보행 시작 |
| `stop` | 현재 스텝 완료 후 보행 종료 |

---

## RHp 링크 치수 (URDF 기준)

| 항목 | 값 |
|------|----|
| thigh_length | 0.0721 m |
| calf_length | 0.0721 m |
| ankle_length | 0.0380 m |
| leg_side_offset | 0.06986 m |
| hip_pitch_offset | 0.0 m |

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

## 빌드

```bash
cd ~/robot_ws
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
# 보행 시작 (--once 필수: 없으면 반복 전송됨)
ros2 topic pub --once /walking/command std_msgs/msg/String "data: 'start'"

# 보행 정지
ros2 topic pub --once /walking/command std_msgs/msg/String "data: 'stop'"
```

---

## 파라미터 튜닝 방법

1. `config/rhphumanoid_walking_param.yaml` 에서 값 수정
2. 보행 노드 재시작

```bash
# Ctrl+C 로 노드 종료 후
ros2 launch rhphumanoid_walking walking.launch.py
```

한 번에 하나의 파라미터만 수정하고 테스트하는 것을 권장합니다.

---

## 참고

- OP3 보행 모듈: [ROBOTIS-GIT/ROBOTIS-OP3](https://github.com/ROBOTIS-GIT/ROBOTIS-OP3)
- RHp 로봇 리소스: `RHp_humanoid_resources-main/`
- 작동 확인된 통신 방식 참고: `rhphumanoid_walking_pattern/`
