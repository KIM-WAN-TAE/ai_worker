# AI Worker F2 follower + A3 leader 제어 구조 가이드

작성 시각: 2026-07-30 21:20:01 KST (2026-07-30T12:20:01Z)  
검사 기준 소스: `/home/robotis/ai_worker` commit
`6fc164a66535e8ea1e2d3c894216ae6ea1d55723` (`feature-intern`)  
검사 범위: AI Worker 소스, 설정, launch, 현재 실행 중인 ROS 2 그래프  
안전 수준: **읽기 전용 검사 및 문서화**. 이 조사에서는 토픽을 발행하거나 로봇을
구동하지 않았다.

## 1. 먼저 보는 결론

F2 follower의 양팔을 A3 leader로 조작하는 핵심 경로는 다음과 같다.

```text
사람이 A3 팔을 움직임
  ↓
A3 DynamixelHardware
  - 엔코더/전송 상태를 관절 상태로 변환
  - J1~J4 × 1.0
  - J5~J7 × 1.5
  - gripper × 2.2
  ↓ ros2_control state interface
/leader/joint_trajectory_command_broadcaster
  - 좌우 관절을 이름으로 재정렬
  - gripper에 +0.1 rad offset
  - TELEOP 진입 시 2.5초 blend
  - F2 URDF 관절 한계로 clamp
  ↓
/leader/joint_trajectory_command_broadcaster_left/joint_trajectory
/leader/joint_trajectory_command_broadcaster_right/joint_trajectory
  ↓ launch remapping
/arm_l_controller/joint_trajectory
/arm_r_controller/joint_trajectory
  ↓
F2 JointTrajectoryController
  ↓ position command interface
F2 DynamixelHardware → 실제 F2 양팔
  ↓
/joint_states, 각 controller_state, Dynamixel 상태 토픽
```

가장 중요한 주의점은 다음 다섯 가지다.

1. `f2_ai.launch.py`는 F2와 **LG2 leader**를 묶는다. A3와 F2를 쓸 때는 현재
   운영 방식처럼 `f2_follower_ai.launch.py`와 `ffw_a3_ai.launch.py`를 각각
   실행해야 한다.
2. A3 J5~J7의 `1.5`는 follower 쪽에서 다시 나누지 않는다. 즉 A3 관절 상태에
   이미 적용된 값을 broadcaster가 F2 목표 위치로 사용한다.
3. A3 broadcaster의 `IDLE`은 명령 발행 중지가 아니라 **마지막 F2 목표를 계속
   발행하여 유지**하는 상태다.
4. `/leader/left_command`, `/leader/right_command`에는 각각 foot switch,
   joystick controller, gripper trigger라는 세 발행자가 있다. 메시지는 출처를
   표시하지 않으므로 여러 입력이 동시에 들어오지 않게 운영해야 한다.
5. 2026-07-30 라이브 검사에서는
   `/leader/joystick_controller/tact_trigger`의 구독자가 0이었다. tact 문자열을
   기록 시스템이 실제 소비하는지는 별도로 확인해야 한다.

## 2. 용어부터 정리

| 용어 | 이 문서에서의 뜻 |
|---|---|
| leader | 사람이 직접 움직이는 A3 장치. 팔 자세, gripper, joystick tact/축 값을 읽는다. |
| follower | leader의 목표를 받아 실제로 움직이는 F2 로봇. |
| hardware interface | Dynamixel 장치와 ros2_control의 state/command interface 사이를 연결하는 플러그인. |
| controller | `controller_manager` 안에서 실행되는 ros2_control 플러그인. ROS 노드 이름은 보이지만 반드시 별도 OS 프로세스인 것은 아니다. |
| broadcaster | A3 state interface를 읽어 F2가 받을 `JointTrajectory`와 leader-originated state를 발행하는 controller. |
| state | 측정값. 이 구성에서는 A3 측정 관절 상태와 F2 실제 feedback을 구분해야 한다. |
| target/reference | follower controller가 따라가도록 요청받은 목표. 실제 F2가 도달한 값과 같다고 가정하면 안 된다. |
| TELEOP | A3의 현재 관절 상태를 F2 목표로 계속 반영하는 모드. |
| IDLE | leader 반영을 멈추고 broadcaster 내부의 `last_target`을 유지하는 모드. |
| SAVE_POSE | 설정 파일의 저장 자세까지 cubic interpolation한 뒤 그 목표를 유지하는 모드. |

## 3. 어떤 launch를 실행하는가

### 3.1 F2 follower

기준 launch:

```text
ffw_bringup/launch/f2_follower_ai.launch.py
```

주요 역할은 다음과 같다.

- F2 URDF와 ros2_control hardware를 준비한다.
- F2 `controller_manager`와 각 controller를 시작한다.
- 양팔, head, lift, swerve 초기 위치 executor를 순서에 맞춰 시작한다.
- 양팔 controller 입력을 A3 broadcaster 토픽으로 remap한다.
- head/lift controller 입력을 leader joystick controller 토픽으로 remap한다.
- 기본값에 따라 카메라, LiDAR, 초기 위치 이동도 시작한다.

핵심 remap은 다음과 같다.

| F2 controller가 원래 구독하는 이름 | 실제 leader 입력 토픽 |
|---|---|
| `/arm_l_controller/joint_trajectory` | `/leader/joint_trajectory_command_broadcaster_left/joint_trajectory` |
| `/arm_r_controller/joint_trajectory` | `/leader/joint_trajectory_command_broadcaster_right/joint_trajectory` |
| `/head_controller/joint_trajectory` | `/leader/joystick_controller_left/joint_trajectory` |
| `/lift_controller/joint_trajectory` | `/leader/joystick_controller_right/joint_trajectory` |

ROS remap 때문에 토픽 그래프에서는 follower controller가 오른쪽 열의 이름을
직접 구독하는 것으로 나타난다.

### 3.2 A3 leader

기준 launch:

```text
ffw_bringup/launch/ffw_a3_ai.launch.py
```

이 launch는 `/leader` namespace 안에 A3 본체와 controller들을 구성한다.
예외적으로 `foot_switch_node`는 namespace 밖에 생성되지만, 모든 주요 출력
토픽을 `/leader/...` 절대 이름으로 발행한다.

기본 A3 serial port는 다음과 같다.

```text
/dev/serial/by-id/usb-ROBOTIS_STM32_Virtual_ComPort_3151388A3434-if00
```

### 3.3 혼동하기 쉬운 `f2_ai.launch.py`

`ffw_bringup/launch/f2_ai.launch.py`가 include하는 것은 아래 두 launch다.

```text
f2_follower_ai.launch.py
ffw_lg2_leader_ai.launch.py
```

따라서 파일 이름만 보고 `f2_ai.launch.py`를 실행하면 A3가 자동으로 붙는다고
생각하면 안 된다. 현재 소스에서 F2+A3 조합은 두 launch를 따로 실행하거나,
둘을 명시적으로 include하는 별도 composite launch가 필요하다.

현재 Docker s6 시작 스크립트도 SG2/BG2/SH5/BH5와 LG2 중심이며 F2+A3 조합을
하나의 기본 profile로 자동 구성하지 않는다. 2026-07-30 라이브 시스템도 F2와
A3 launch가 별도 프로세스로 시작돼 있었다.

## 4. 관련 패키지 전체 지도

### 4.1 직접 제어 경로에 들어가는 AI Worker 패키지

| 패키지 | 주요 역할 | F2+A3 경로에서의 위치 |
|---|---|---|
| `ffw` | AI Worker의 meta/package 집합 | 패키지 묶음과 의존성 진입점 |
| `ffw_bringup` | launch, controller YAML, foot switch, 초기 자세 executor | 전체 시스템 기동과 연결 |
| `ffw_description` | A3/F2 URDF, xacro, ros2_control hardware 선언, 관절 한계 | 하드웨어/관절/변환 계약 |
| `ffw_joint_trajectory_command_broadcaster` | A3 state를 F2 arm `JointTrajectory`로 변환 | 양팔 leader→follower 핵심 |
| `ffw_joystick_controller` | A3 joystick tact, head/lift jog, `/cmd_vel` 처리 | 팔 TELEOP toggle, tact event, 보조축 |
| `ffw_spring_actuator_controller` | A3 양쪽 gripper에 spring/damping/dither effort 적용 | leader gripper 촉감/복귀, follower 명령은 아님 |
| `ffw_swerve_drive_controller` | F2 base의 `/cmd_vel` 처리, wheel/steer 명령, odometry | base 경로. 현재 A3 profile에서는 non-zero 입력 비활성 |
| `ffw_robot_manager` | Dynamixel 오류 감시, LED, battery/상태 관리 | follower 상태/안전 보조 |

### 4.2 외부 ROS 패키지

| 패키지/플러그인 | 역할 |
|---|---|
| `controller_manager` | A3/F2 ros2_control controller lifecycle 및 100 Hz update 관리 |
| `joint_state_broadcaster` | A3의 `/leader/joint_states`, F2의 `/joint_states` 발행 |
| `joint_trajectory_controller` | F2 양팔/head/lift/swerve 초기 조향 trajectory 수신 |
| `robot_state_publisher` | 각 robot description과 joint state로 TF 발행 |
| `dynamixel_hardware_interface` | A3/F2 Dynamixel read/write, transmission matrix, 상태와 service 제공 |
| `dynamixel_interfaces` | Dynamixel 상태 및 service 메시지 정의 |

`ai_worker_ci.repos`는 `dynamixel_hardware_interface`의 `main` branch를
가리키므로 새 image를 만들 때 dependency 구현이 달라질 수 있다. 과거 감사에서
검증한 설치 소스 commit은
`3375b3d0c99c5bcdab11b2dc15c89a2923e2bca2`였지만, 재현성을 위해서는
image digest와 실제 dependency commit을 함께 기록해야 한다.

### 4.3 선택 사항이거나 기본 A3 arm 경로가 아닌 패키지

| 패키지 | 관계와 주의점 |
|---|---|
| `ffw_teleop` | 수동 keyboard/mobile teleop. 기본 A3 팔 경로는 아니다. |
| `ffw_teleop`의 `keyborad_control` | 철자가 이 이름으로 등록돼 있다. leader arm/head/lift trajectory 토픽에 직접 발행할 수 있어 A3 경로와 충돌하거나 broadcaster 제어를 우회할 수 있다. |
| `ffw_teleop`의 `mobile_teleop` | `/cmd_vel`에 직접 발행한다. |
| `ffw_moveit_config` | MoveIt 계획/설정. 일반 A3 leader→F2 follower arm 복제 경로에는 들어가지 않는다. |
| `ffw_navigation` | navigation stack. 현재 A3 profile의 navigation 비활성 결정과 별개이며 기본 arm 경로가 아니다. |

## 5. A3 leader 쪽 노드와 controller

### 5.1 라이브에서 확인한 A3 구성

| 노드/controller | 종류 | 입력 | 출력/역할 |
|---|---|---|---|
| `/leader/controller_manager` | ros2_control node | A3 hardware, controller YAML | controller lifecycle, 100 Hz update |
| `/leader/ffw_a3` | hardware component 이름 | serial Dynamixel | A3 state/command interface |
| `/leader/dynamixel_hardware_interface` | hardware 보조 node | A3 bus | DXL 상태와 service |
| `/leader/joint_state_broadcaster` | controller | A3 state interfaces | `/leader/joint_states`, `/leader/dynamic_joint_states` |
| `/leader/joint_trajectory_command_broadcaster` | controller | A3 arm/gripper position state, F2 초기 `/joint_states`, command topics | 좌우 F2 arm trajectory, leader-originated target state |
| `/leader/joystick_controller` | controller | SensorXEL joystick state, middle pedal, F2 `/joint_states` | arm mode command, tact trigger, zero `/cmd_vel`, optional head/lift trajectory |
| `/leader/spring_actuator_controller` | controller | A3 gripper position/velocity | A3 gripper effort |
| `/leader/robot_state_publisher` | node | `/leader/joint_states`, A3 description | A3 TF |
| `/leader/gripper_trigger` | node | `/leader/joint_states` | gripper threshold crossing 시 command ID 3 |
| `/foot_switch_node` | Python node | Linux input event device | 좌우 command ID 4, middle pedal Bool heartbeat |

### 5.2 A3 ros2_control interface

A3 hardware는 `dynamixel_hardware_interface/DynamixelHardware`를 사용하며,
16개 joint/transmission을 선언한다.

- arm J1~J7은 position **state interface**만 가진다.
- gripper joint는 effort command와 position/velocity state를 가진다.
- joystick SensorXEL은 X, Y, tact switch state interface를 가진다.
- arm의 물리 상태를 읽는 경로와 gripper spring effort를 쓰는 경로는 목적이
  다르다.

즉 `spring_actuator_controller`는 A3 gripper의 느낌과 복귀력을 만드는
controller이며, F2 gripper 목표를 직접 발행하는 controller가 아니다. F2
gripper 목표는 arm broadcaster의 여덟 번째 관절 값으로 함께 전달된다.

## 6. F2 follower 쪽 노드와 controller

### 6.1 라이브에서 확인한 F2 구성

| 노드/controller | 종류 | 입력 | 출력/역할 |
|---|---|---|---|
| `/controller_manager` | ros2_control node | F2 hardware와 YAML | controller lifecycle, 100 Hz update |
| `/ffw_f2_follower` | main hardware | `/dev/follower` | 양팔/head/lift 등 22개 joint |
| `/ffw_f2_base` | base hardware | `/dev/ttyUSB0` | swerve wheel/steer |
| `/ffw_f2_sensor` | sensor hardware | `/dev/ttyUSB1` | F2 sensor |
| `/joint_state_broadcaster` | controller | 전체 F2 state | `/joint_states`, `/dynamic_joint_states` |
| `/arm_l_controller` | JointTrajectoryController | leader left trajectory | 왼팔 7축 + gripper position command |
| `/arm_r_controller` | JointTrajectoryController | leader right trajectory | 오른팔 7축 + gripper position command |
| `/head_controller` | JointTrajectoryController | leader left joystick trajectory | head 2축 |
| `/lift_controller` | JointTrajectoryController | leader right joystick trajectory | lift 1축 |
| `/swerve_drive_controller` | custom controller | `/cmd_vel` | wheel/steer 명령, `/odom`, TF, 제한 속도 |
| `/ffw_robot_manager` | custom controller | DXL state/battery | 오류 감시, LED, battery 상태 |
| `/robot_state_publisher` | node | `/joint_states`, F2 description | F2 TF |
| `/arm_l_joint_trajectory_executor` | 초기화 node | `/joint_states`, left FollowJointTrajectory action | 초기 자세 명령 후 수렴 확인 |
| `/arm_r_joint_trajectory_executor` | 초기화 node | `/joint_states`, right FollowJointTrajectory action | 초기 자세 명령 후 수렴 확인 |

F2 설정에는 일시적인
`swerve_steering_initial_position_controller`도 있다. 초기 조향 자세 설정 뒤
정상 swerve controller로 전환하기 위한 controller이며, 계속 사용하는 A3 arm
경로는 아니다.

### 6.2 F2 arm controller 관절 순서

좌우 모두 메시지의 관절 순서는 다음과 같다.

```text
arm_[l|r]_joint1
arm_[l|r]_joint2
arm_[l|r]_joint3
arm_[l|r]_joint4
arm_[l|r]_joint5
arm_[l|r]_joint6
arm_[l|r]_joint7
gripper_[l|r]_joint1
```

두 arm controller는 다음 특성을 가진다.

- command interface: `position`
- state interface: `position`, `velocity`
- `allow_nonzero_velocity_at_trajectory_end: true`
- `allow_partial_joints_goal: true`

하지만 A3 broadcaster는 항상 해당 side의 8개 관절 이름과 위치를 함께
발행한다.

## 7. 팔 조작 데이터가 흐르는 순서

### 7.1 1단계: A3 hardware가 물리 위치를 읽는다

사람이 A3 팔을 움직이면 Dynamixel hardware interface가 transmission 위치를
읽고 `transmission_to_joint_matrix`를 적용한다.

행렬 선언 순서는 다음과 같다.

```text
gripper_l,
arm_l_joint1 ... arm_l_joint7,
gripper_r,
arm_r_joint1 ... arm_r_joint7
```

`/leader/joint_states`의 배열 순서가 항상 이 선언 순서와 같다고 가정하면 안
된다. downstream 코드는 joint 이름으로 값을 매칭해야 한다. 현재 broadcaster
설정은 각 side를 J1~J7, gripper 순으로 명시적으로 재정렬한다.

### 7.2 2단계: A3 transmission matrix가 scale을 적용한다

현재 양쪽 팔의 변환은 대각 행렬이며 다음과 같다.

| A3 joint | matrix gain | broadcaster offset | F2로 보낼 TELEOP 후보 |
|---|---:|---:|---|
| J1 | 1.0 | 0.0 | `q_A3` |
| J2 | 1.0 | 0.0 | `q_A3` |
| J3 | 1.0 | 0.0 | `q_A3` |
| J4 | 1.0 | 0.0 | `q_A3` |
| J5 | 1.5 | 0.0 | `1.5 × q_A3` |
| J6 | 1.5 | 0.0 | `1.5 × q_A3` |
| J7 | 1.5 | 0.0 | `1.5 × q_A3` |
| gripper | 2.2 | +0.1 | `2.2 × q_A3 + 0.1` |

이후 TELEOP blend가 필요하면 이전 목표와 섞고, 마지막에 F2 URDF limit로
clamp한다. 현재 A3 YAML에는 reverse joint 목록이 없으므로 broadcaster 단계의
부호 반전은 없다.

중요하게도 F2 hardware에는 이 gain을 되돌리는 inverse matrix가 없다. 따라서
아래와 같이 이해해야 한다.

```text
F2 target = clamp_F2_limit(
  optional_blend(
    optional_sign(A3_transformed_state) + offset
  )
)
```

`A3_transformed_state` 자체에 이미 J5~J7의 `1.5`와 gripper의 `2.2`가 반영돼
있다.

### 7.3 3단계: broadcaster가 모드에 따라 `last_target`을 갱신한다

| 모드 | `last_target` 처리 |
|---|---|
| `IDLE` | 갱신하지 않는다. 초기 F2 feedback 또는 마지막 목표를 유지한다. |
| `TELEOP` | 현재 A3 변환 상태 + offset을 읽고 F2 limit로 clamp한다. 진입 시 2.5초 cubic smoothstep blend한다. |
| `SAVE_POSE` | 현재 `last_target`에서 설정된 저장 자세까지 2.0초 cubic interpolation하고 완료 후 유지한다. |

초기값 `left_enabled_init`와 `right_enabled_init`은 기본 false이므로 두 side 모두
IDLE로 시작한다. broadcaster는 시작할 때 `/joint_states`에서 필요한 8개
follower 관절을 이름으로 찾아 각 side의 `last_target`을 한 번 초기화한다.
이후 그 follower subscription은 계속 존재하지만 초기화가 끝나면 새 feedback을
`last_target`에 반영하지 않는다.

### 7.4 4단계: trajectory를 계속 발행한다

유효한 `last_target`이 생긴 뒤에는 IDLE이어도 메시지를 계속 발행한다.

- message: `trajectory_msgs/msg/JointTrajectory`
- joint: side별 8개
- point: 1개
- `header.stamp`: 0
- `time_from_start`: 0
- controller update rate: 100 Hz

라이브에서 약 99.7 Hz 발행을 확인했다. 따라서 “toggle로 IDLE 전환”은 publisher
종료나 메시지 0 Hz가 아니라 목표 고정이다.

### 7.5 5단계: F2 controller와 hardware가 목표를 실행한다

launch remap으로 left/right trajectory는 각각 F2 arm controller에 바로
연결된다. JointTrajectoryController가 position command interface를 갱신하고,
F2 DynamixelHardware가 실제 actuator에 쓴다.

실제 follower 상태는 `/joint_states`에 나타난다. broadcaster가 내보낸 target과
실제 feedback은 반드시 별개의 의미로 기록해야 한다.

| 데이터 | 의미 |
|---|---|
| A3 raw/transmission 위치 | 물리 leader encoder 영역의 원자료 |
| `/leader/joint_states` | A3 matrix가 반영된 leader 관절 state |
| leader arm trajectory | offset/blend/clamp까지 적용된 desired F2 target |
| F2 controller reference | controller가 현재 추종하는 reference |
| `/joint_states` | F2에서 측정한 실제 follower state |

## 8. F2 관절 한계와 clamp

TELEOP에서는 F2 `/robot_description`을 transient-local/reliable QoS로 한 번
받아 관절 한계를 읽는다. 현재 URDF 기준 위치 한계는 다음과 같다.

| Joint | F2 left [rad] | F2 right [rad] |
|---|---:|---:|
| J1 | `[-3.14, 3.14]` | `[-3.14, 3.14]` |
| J2 | `[0, 3.14]` | `[-3.14, 0]` |
| J3 | `[-3.14, 3.14]` | `[-3.14, 3.14]` |
| J4 | `[-2.9361, 1.0786]` | `[-2.9361, 1.0786]` |
| J5 | `[-3.14, 3.14]` | `[-3.14, 3.14]` |
| J6 | `[-1.57, 1.57]` | `[-1.57, 1.57]` |
| J7 | `[-1.8201, 1.5804]` | `[-1.5804, 1.8201]` |
| gripper | `[0, 1.1]` | `[0, 1.1]` |

J5~J7에 1.5를 적용한 결과가 이 범위를 벗어나면 broadcaster가 target을 limit에
고정한다. 이 clamp는 안전에 도움이 되지만, raw leader 동작과 follower target의
기울기가 범위 끝에서 더 이상 1.5가 아니게 된다는 뜻이기도 하다.

## 9. 팔 모드를 바꾸는 command 토픽

### 9.1 `UInt8` 값의 의미

`/leader/left_command`와 `/leader/right_command`를 broadcaster가 같은 규칙으로
해석한다.

| 값 | 의미 | 결과 |
|---:|---|---|
| 0 | disable/stop | `IDLE`; 마지막 target 유지 |
| 1 | enable | `TELEOP`; 필요 시 2.5초 blend 후 leader 추종 |
| 2 | toggle | `TELEOP ↔ IDLE` |
| 3 이상 | save pose ID | 해당 side의 저장 자세 interpolation; 없는 ID는 경고 후 무시 |

현재 설정의 `save_pose_ids`는 `[3, 4]`, 보간 시간은 2.0초다.

### 9.2 같은 command 토픽의 발행자

| 발행자 | 언제 무엇을 발행하는가 |
|---|---|
| `/foot_switch_node` | 왼쪽 pedal press → left에 4, 오른쪽 pedal press → right에 4 |
| `/leader/joystick_controller` | middle pedal 없이 짧은 tact release → 해당 side에 2; 양쪽 tact → 양쪽에 2 |
| `/leader/gripper_trigger` | `/leader/joint_states`의 gripper가 `-0.5` 아래로 내려가는 crossing → 해당 side에 3 |

발행자가 세 개이므로 한 이벤트 직후 다른 이벤트가 들어오면 마지막 메시지가
모드를 다시 바꿀 수 있다. `UInt8` 메시지에는 발행 원인, sequence, timestamp,
현재 모드가 없다.

### 9.3 저장 자세

현재 A3 YAML의 save pose는 side별 8개 값이며 마지막 값은 gripper다.

| ID | 발생 경로 | 의도 |
|---:|---|---|
| 3 | gripper trigger 기본값 | 설정된 작업/초기 자세로 이동 |
| 4 | 좌/우 foot pedal | 설정된 별도 저장 자세로 이동 |

ID 4의 gripper target은 1.0이며, ID 3의 gripper target은 0.0이다. SAVE_POSE
값은 A3 현재 state를 다시 matrix 변환한 값이 아니라 YAML에 직접 저장된 F2
target 배열이다.

정확한 현재 배열 순서는 `J1, J2, J3, J4, J5, J6, J7, gripper`다.

| Side/ID | 저장된 F2 target [rad] |
|---|---|
| left 3 | `[0.234, 0.0, 0.898, -2.3, -0.577, 0.906, -1.693, 0.0]` |
| right 3 | `[0.234, 0.0, -0.898, -2.3, 0.577, 0.906, 1.693, 0.0]` |
| left 4 | `[0.4310126487, 0.3037162118, 0.1518760822, -2.4574492064, -0.2668886886, 0.6856534595, -0.9479904508, 1.0]` |
| right 4 | `[0.4310126487, -0.3037162118, -0.1518760822, -2.4574492064, 0.2668886886, 0.6856534595, 0.9479904508, 1.0]` |

## 10. joystick tact와 foot switch 입력

### 10.1 foot switch

Linux 장치:

```text
/dev/input/by-id/usb-PCsensor_FootSwitch-event-kbd
```

| pedal | Linux key code | 동작 |
|---|---:|---|
| left | 30 | press 시 `/leader/left_command = 4` |
| middle | 48 | press/release edge와 10 Hz Bool heartbeat |
| right | 46 | press 시 `/leader/right_command = 4` |

debounce는 press/release 값별로 0.5초다. middle 상태 heartbeat 주기는 0.1초다.
node가 정상 종료될 때 middle false를 한 번 발행하도록 구현돼 있다.

### 10.2 A3 tact 처리

| 입력 | middle 상태 | 출력 |
|---|---|---|
| 짧은 right tact 후 release | fresh + pressed | tact trigger `"right"` |
| 짧은 left tact 후 release | fresh + pressed | tact trigger `"left"` |
| 짧은 right tact 후 release | released 또는 fresh하지 않지만 마지막 상태가 false | `/leader/right_command = 2` |
| 짧은 left tact 후 release | released 또는 fresh하지 않지만 마지막 상태가 false | `/leader/left_command = 2` |
| 짧은 양쪽 tact 후 release | middle 여부와 무관 | 양쪽 command에 2 |
| left long press | stale-pressed가 아님 | tact trigger `"left_long_time"` |
| right long press | fresh middle pressed | tact trigger `"right_long_time_middle"` |
| right long press | middle released | tact trigger `"right_long_time"` |
| tact release | 마지막 middle이 pressed인데 heartbeat가 0.5초 초과 | action 억제 |

fresh middle + short left의 `"left"`는 recording save/start toggle, short
right의 `"right"`는 recording discard 용도로 계약돼 있다. 다만 2026-07-30 라이브
snapshot에서는 이 tact trigger 토픽의 구독자가 0이었으므로, 문자열 발행만으로
기록 동작이 완료된다고 판단하면 안 된다.

### 10.3 현재 joystick gate

라이브에서 읽은 A3 parameter는 다음과 같다.

```yaml
enable_joystick_update: true
enable_joint_jog_update: false
enable_cmd_vel_update: false
require_middle_pedal_for_cmd_vel: true
middle_pedal_timeout_sec: 0.5
```

해석:

- master는 true라 tact switch 처리는 살아 있다.
- head/lift joint jog는 비활성이다.
- mobile-base non-zero `/cmd_vel`은 비활성이다.
- controller는 비활성 상태에서도 매 update마다 zero Twist를 발행한다.
- middle pedal heartbeat가 stale하면 base gate는 닫히고 zero Twist를 낸다.

따라서 현재 구성에서는 A3 joystick 축으로 head/lift/base를 움직이는 것이
목표가 아니다. 하지만 다른 publisher가 동일한 head/lift trajectory나
`/cmd_vel`에 연결되면 별도 경로가 생길 수 있다.

## 11. 핵심 토픽 표

### 11.1 leader→follower command와 mode

| 토픽 | 타입 | 발행자 | 구독자 | 설명 |
|---|---|---|---|---|
| `/leader/joint_trajectory_command_broadcaster_left/joint_trajectory` | `trajectory_msgs/msg/JointTrajectory` | A3 arm broadcaster | F2 left arm controller | 왼팔 8축 target |
| `/leader/joint_trajectory_command_broadcaster_right/joint_trajectory` | `trajectory_msgs/msg/JointTrajectory` | A3 arm broadcaster | F2 right arm controller | 오른팔 8축 target |
| `/leader/left_command` | `std_msgs/msg/UInt8` | foot switch, joystick controller, gripper trigger | arm broadcaster | 왼팔 mode/save pose |
| `/leader/right_command` | `std_msgs/msg/UInt8` | foot switch, joystick controller, gripper trigger | arm broadcaster | 오른팔 mode/save pose |
| `/leader/foot_switch/middle_pedal` | `std_msgs/msg/Bool` | foot switch | joystick controller | edge + 10 Hz heartbeat |
| `/leader/joystick_controller/tact_trigger` | `std_msgs/msg/String` | joystick controller | 기록/상위 시스템 | recording/tact 문자열 |
| `/leader/joystick_controller_left/joint_trajectory` | `trajectory_msgs/msg/JointTrajectory` | joystick controller | F2 head controller | 현재 joint jog gate false |
| `/leader/joystick_controller_right/joint_trajectory` | `trajectory_msgs/msg/JointTrajectory` | joystick controller | F2 lift controller | 현재 joint jog gate false |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | joystick controller 및 가능한 외부 teleop | F2 swerve controller | 현재 A3 controller는 zero만 발행 |

### 11.2 상태와 관찰

| 토픽 | 타입 | 의미 |
|---|---|---|
| `/leader/joint_states` | `sensor_msgs/msg/JointState` | A3 joint state. matrix 적용 후 값이며 이름으로 해석해야 함 |
| `/leader/dynamic_joint_states` | `control_msgs/msg/DynamicJointState` | A3 interface별 상세 state |
| `/robot/arm_left_leader/joint_states` | `sensor_msgs/msg/JointState` | broadcaster가 내보내는 왼쪽 `last_target` 중심 state |
| `/robot/arm_right_leader/joint_states` | `sensor_msgs/msg/JointState` | broadcaster가 내보내는 오른쪽 `last_target` 중심 state |
| `/robot/head_leader/joint_states` | `sensor_msgs/msg/JointState` | joystick controller의 head 관련 state |
| `/robot/lift_leader/joint_states` | `sensor_msgs/msg/JointState` | joystick controller의 lift 관련 state |
| `/joint_states` | `sensor_msgs/msg/JointState` | F2 전체 실제 joint feedback 및 A3 broadcaster 초기 target source |
| `/dynamic_joint_states` | `control_msgs/msg/DynamicJointState` | F2 interface별 상세 state |
| `/arm_l_controller/controller_state` | `control_msgs/msg/JointTrajectoryControllerState` | left reference, feedback, error |
| `/arm_r_controller/controller_state` | `control_msgs/msg/JointTrajectoryControllerState` | right reference, feedback, error |
| `/head_controller/controller_state` | 같은 타입 | head controller 상태 |
| `/lift_controller/controller_state` | 같은 타입 | lift controller 상태 |

`/robot/arm_*_leader/joint_states`라는 이름 때문에 순수 A3 측정값으로 오해하기
쉽지만, 현재 구현의 position은 `last_target`을 사용한다. IDLE과 SAVE_POSE에서
원래 A3 물리 자세와 다를 수 있다.

### 11.3 Dynamixel, base, 관리 토픽

| 토픽/service | 역할 |
|---|---|
| `/leader/dynamixel_hardware_interface/dxl_state` | A3 DXL 상태 |
| `/ffw_follower/dxl_state` | F2 main follower DXL 상태 |
| `/ffw_base/dxl_state` | F2 base DXL 상태 |
| `/ffw_sensor/dxl_state` | F2 sensor DXL 상태 |
| `ffw_follower/get_dxl_data`, set/reboot 계열 service | F2 DXL 조회/설정/재부팅 |
| `dynamixel_hardware_interface/set_dxl_torque` | 구성에 따른 DXL torque service |
| `/odom` | swerve odometry |
| `/tf` | robot state publisher 및 swerve TF |
| `/limited_cmd_vel` | swerve limit가 적용된 velocity |
| `/joint_commanders` | swerve joint command 진단 |
| `/swerve_drive_controller/swerve_visualization_markers` | swerve 시각화 |
| battery 관련 토픽 | `ffw_robot_manager`의 battery 상태 |

정확한 service 절대 이름은 namespace와 hardware xacro 파라미터에 따라 달라질
수 있으므로 라이브에서는 `ros2 service list -t`로 다시 확인해야 한다.

### 11.4 초기화 action과 controller 관리 service

| endpoint | 타입/소유자 | 용도 |
|---|---|---|
| `/arm_l_controller/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` | F2 왼팔 초기 자세 executor |
| `/arm_r_controller/follow_joint_trajectory` | 같은 action | F2 오른팔 초기 자세 executor |
| `/head_controller/follow_joint_trajectory` | 같은 action | F2 head 초기 자세 executor |
| `/lift_controller/follow_joint_trajectory` | 같은 action | F2 lift 초기 자세 executor |
| `/swerve_steering_initial_position_controller/follow_joint_trajectory` | 같은 action | 초기 swerve steer 정렬 |
| `/controller_manager/*` | controller_manager service 집합 | F2 controller list/load/configure/switch 등 |
| `/leader/controller_manager/*` | controller_manager service 집합 | A3 controller list/load/configure/switch 등 |

`ros2 control list_controllers`는 읽기 전용 조회이지만 load/configure/switch/
unload service는 runtime 상태를 바꾸므로 동일하게 취급하면 안 된다.

## 12. 초기화와 기동 순서

권장되는 의존 순서는 개념적으로 다음과 같다.

1. F2 hardware와 `/robot_description`을 준비한다.
2. F2 `joint_state_broadcaster`를 활성화해 `/joint_states`가 나오게 한다.
3. F2 arm/head/lift controller를 활성화한다.
4. 초기 자세 executor가 action을 보내고 실제 position/velocity 수렴을 확인한다.
5. A3 hardware와 A3 `joint_state_broadcaster`를 준비한다.
6. A3 broadcaster가 F2 `/joint_states`에서 양쪽 초기 `last_target`을 얻는다.
7. A3 broadcaster가 F2 `/robot_description`에서 limit를 읽는다.
8. A3 broadcaster가 active가 되고 IDLE hold trajectory를 발행한다.
9. command 1 또는 toggle 2가 들어오면 해당 side TELEOP가 시작된다.

A3 broadcaster의 `on_configure()`는 다음 두 조건을 동기적으로 기다린다.

- 양쪽 follower 관절을 모두 포함한 `/joint_states`
- F2 `/robot_description`

따라서 F2가 아직 준비되지 않았거나 QoS/이름이 맞지 않으면 A3 controller
configure가 대기할 수 있다. 현재 구현에는 이 두 wait의 자체 timeout이 없다.

F2 초기 자세 executor는 단순히 action 응답만 받고 끝나는 것이 아니라
`/joint_states`의 position과 velocity tolerance가 만족될 때 self-exit한다.
2026-07-30 snapshot에서는 left/right executor가 아직 살아 있고 head/lift/
swerve executor는 종료돼 있었다. 양팔 executor가 남아 있는 이유는 로그와
오차를 추가로 확인해야 한다.

현재 F2 초기화 설정은 다음과 같다.

| 대상 | 단계/목표 | 설정 시간 | 수렴 tolerance |
|---|---|---:|---|
| left arm | step0 `[1.57, 0, 0, -2.8, 0, 0, 0, 0]` → step1 `[0.234, 0, 0.898, -2.3, -0.577, 0.906, -1.693, 0]` | 5.0초 | position 0.1 rad, velocity 0.01 rad/s |
| right arm | step0 `[1.57, 0, 0, -2.8, 0, 0, 0, 0]` → step1 `[0.234, 0, -0.898, -2.3, 0.577, 0.906, 1.693, 0]` | 5.0초 | 동일 |
| head | `[1.0, 0.05]` | 5.0초 | 동일 |
| lift | `[0.0]` → `[-0.15]` | 10.0초 | 동일 |
| swerve steer | `[0.0, 0.0, 0.0]` | 10.0초 | 동일 |

arm의 최종 초기 자세는 save-pose ID 3과 같다. 그러나 초기 executor는 F2
controller action을 사용하고, save-pose ID 3은 A3 broadcaster의 command
state machine을 사용하므로 호출 경로와 완료 판정은 다르다.

## 13. base, head, lift는 어떻게 되는가

### 13.1 mobile base

경로:

```text
A3 left joystick axes
  → /leader/joystick_controller
  → /cmd_vel
  → /swerve_drive_controller
  → F2 wheel/steer
```

F2 swerve controller의 `cmd_vel_timeout`은 1.0초다. 그러나 현재 A3 parameter
`enable_cmd_vel_update=false`이므로 joystick controller는 non-zero Twist를
만들지 않고 zero Twist를 계속 발행한다.

### 13.2 head와 lift

경로:

```text
A3 left joystick → head trajectory → F2 head_controller
A3 right joystick X → lift trajectory → F2 lift_controller
```

현재 `enable_joint_jog_update=false`라 두 trajectory 경로의 joystick-driven
업데이트가 비활성이다. 이 사실은 “head/lift controller가 없음”을 뜻하지 않는다.
controller와 연결 토픽은 존재하지만 A3 축 명령 생성 gate가 닫혀 있다는 뜻이다.

## 14. 현재 라이브 snapshot

검사 시간대: 2026-07-30 20:49~21:20 KST의 읽기 전용 점검  
ROS 배포판: Jazzy  
`ROS_DOMAIN_ID`: 30  
RMW: `rmw_zenoh_cpp`

| 항목 | 확인 결과 |
|---|---|
| AI Worker source | `6fc164a66535e8ea1e2d3c894216ae6ea1d55723`, clean worktree |
| AI Worker container | `robotis/ai-worker:1.3.0` |
| image ID | `sha256:a5be28b7dc2db1a90458ab7fb788f46cfa16a4eb728fb94c8910674f74f68a4d` |
| container start | `2026-07-30T11:24:01.131508344Z` |
| F2 bringup | `f2_follower_ai.launch.py` 실행 중 |
| A3 bringup | `ffw_a3_ai.launch.py` 실행 중 |
| F2 controllers | arm L/R, head, lift, swerve, joint state, robot manager 모두 active |
| A3 controllers | joint state, arm broadcaster, spring, joystick 모두 active |
| arm trajectory graph | side별 publisher 1, subscriber 1 |
| command graph | side별 publisher 3, broadcaster subscriber 1 |
| middle pedal | publisher 1, subscriber 1; sample false |
| tact trigger | publisher 1, subscriber 0 |
| arm trajectory rate | 약 99.7 Hz |
| `/joint_states` rate | 약 100 Hz |
| `/cmd_vel` sample | zero Twist |

샘플링 당시 left arm trajectory와 F2 reference/feedback은 거의 같았고, A3
`/leader/joint_states` 값은 outgoing trajectory와 달랐다. 이는 broadcaster가
IDLE에서 follower로 초기화된 target을 유지한 상황과 일치하지만, 현재
broadcaster는 모드를 토픽/parameter로 노출하지 않으므로 snapshot만으로
IDLE이라고 확정하지는 않는다.

ROS graph에는 이름이 같은 `dynamixel_hardware_interface` node가 여러 개 보여
`ros2 node list` 경고가 발생했다. 진단 시 node 이름만으로 A3/F2/main/base/
sensor를 구분하지 말고 namespace, topic endpoint, process를 함께 봐야 한다.

## 15. 확인된 한계와 위험

### 15.1 설정에 있지만 구현에서 쓰지 않는 parameter

parameter schema에는 아래 항목이 있지만 현재 broadcaster C++의 동작 판단에는
사용되지 않는다.

- `follower_joint_states_topic`
- `sync_threshold`
- `max_error`
- `min_error`
- `min_delay`
- `max_delay`

실제로 쓰이는 것은 side별
`left_follower_joint_states_topic`/`right_follower_joint_states_topic`이며,
그마저 시작 시 `last_target`을 한 번 초기화하는 용도다. 따라서 parameter
이름만 보고 “leader/follower sync error가 실시간 감시된다”고 판단하면 안 된다.

### 15.2 현재 broadcaster가 노출하지 않는 운영 정보

- left/right 현재 모드
- command 메시지의 출처
- 마지막 mode 전환 시각
- clamp가 발생한 joint와 raw 값
- follower tracking error에 따른 gate
- follower state stale 여부

이 정보가 필요하면 별도 상태/진단 인터페이스를 설계해야 한다.

### 15.3 메시지와 controller 동작

- broadcaster는 100 Hz로 `stamp=0`, `time_from_start=0`인 1-point trajectory를
  계속 발행한다.
- TELEOP target은 F2 URDF limit로 clamp하지만 한 joint의 clamp를 전체 chunk
  reject로 취급하지 않는다.
- follower tracking error가 커도 A3 broadcaster 자체는 TELEOP를 자동
  중단하지 않는다.
- IDLE은 command stream 정지가 아니라 target hold다.

### 15.4 안전/운영 관련

- 현재 F2 `ffw_robot_manager` 설정은
  `disable_torque_on_error: false`,
  `set_led_error_on_error: true`다. 즉 이 profile에서는 manager가 error를
  감지했다고 자동 torque-off한다고 가정하면 안 된다.
- `f2_ai.launch.py`의 LG2 결합은 A3 운영자에게 혼동을 줄 수 있다.
- command 토픽의 다중 publisher는 이벤트 경합 가능성이 있다.
- tact trigger는 검사 시 구독자가 없었다.
- A3 J5~J7 gain 1.5는 dataset/action 계약에도 직접 영향을 준다.
- 특히 J7은 과거 follower reference와 feedback 차이가 관찰된 적이 있어,
  gain 문제와 actuator tracking 문제를 시간 정렬된 자료로 분리해야 한다.
- `ai_worker_ci.repos`의 Dynamixel dependency가 floating `main`이라 image 재빌드
  재현성이 약하다.

## 16. 데이터 수집과 정책 학습에서의 의미

F2의 학습 observation state는 일반적으로 실제 F2 `/joint_states`이고, action은
A3에서 출발해 matrix, offset, blend/clamp를 거친 desired F2 target이다.

```text
observation.state = measured F2 state
action             = requested F2 target generated from A3
```

따라서 action과 state가 다르다고 해서 곧바로 라벨 오류는 아니다. tracking
delay, actuator 한계, clamp, contact, controller 상태 때문에 차이가 날 수 있다.
반대로 A3 raw 위치를 action이라고 기록하면 현재 실행 경로와 계약이 달라진다.

새 dataset/checkpoint를 만들 때 최소한 다음을 고정해야 한다.

- AI Worker source commit과 container image digest
- Dynamixel hardware interface commit
- A3 expanded `transmission_to_joint_matrix`
- broadcaster joint order, offset, reverse 목록
- F2 URDF joint limit과 hardware revision
- command/reference/feedback 중 무엇을 action으로 저장하는지
- control rate와 timestamp 의미
- J5~J7 1.5 및 gripper 2.2/+0.1 계약

## 17. 읽기 전용 확인 명령

아래 명령은 상태 확인용이다. 실행 전에 현재 shell/container의 ROS 환경이
올바른지 확인한다.

```bash
export ROS_DOMAIN_ID=30
export RMW_IMPLEMENTATION=rmw_zenoh_cpp

ros2 node list
ros2 control list_controllers -c /controller_manager
ros2 control list_controllers -c /leader/controller_manager

ros2 topic info -v \
  /leader/joint_trajectory_command_broadcaster_left/joint_trajectory
ros2 topic info -v \
  /leader/joint_trajectory_command_broadcaster_right/joint_trajectory
ros2 topic info -v /leader/left_command
ros2 topic info -v /leader/right_command
ros2 topic info -v /leader/joystick_controller/tact_trigger

ros2 topic echo --once /leader/joint_states
ros2 topic echo --once /joint_states
ros2 topic echo --once /arm_l_controller/controller_state
ros2 topic echo --once /arm_r_controller/controller_state
ros2 topic echo --once /cmd_vel

ros2 param get /leader/joystick_controller enable_joystick_update
ros2 param get /leader/joystick_controller enable_joint_jog_update
ros2 param get /leader/joystick_controller enable_cmd_vel_update
ros2 param get /leader/joystick_controller require_middle_pedal_for_cmd_vel
ros2 param get /leader/joystick_controller middle_pedal_timeout_sec
```

이 문서에는 `ros2 topic pub`, action goal, torque service 호출 같은 구동 명령을
싣지 않았다. powered 검증은 별도 사전 점검과 명시적 사용자 승인이 필요하다.

## 18. 소스 읽는 순서

처음 구조를 따라갈 때는 다음 순서가 가장 빠르다.

1. `ffw_bringup/launch/f2_follower_ai.launch.py`
2. `ffw_bringup/launch/ffw_a3_ai.launch.py`
3. `ffw_bringup/config/ffw_f2_follower/ffw_f2_follower_ai_hardware_controller.yaml`
4. `ffw_bringup/config/ffw_a3/ffw_a3_ai_hardware_controller.yaml`
5. `ffw_description/ros2_control/ffw_a3/ffw_a3.ros2_control.xacro`
6. `ffw_description/ros2_control/ffw_f2_follower/ffw_f2_follower.ros2_control.xacro`
7. `ffw_joint_trajectory_command_broadcaster/src/joint_trajectory_command_broadcaster.cpp`
8. `ffw_joint_trajectory_command_broadcaster/src/joint_trajectory_command_broadcaster_parameters.yaml`
9. `ffw_joystick_controller/src/joystick_controller.cpp`
10. `ffw_joystick_controller/src/joystick_controller_parameters.yaml`
11. `ffw_bringup/ffw_bringup/foot_switch.py`
12. `ffw_joint_trajectory_command_broadcaster/src/gripper_trigger.cpp`
13. `ffw_spring_actuator_controller/src/spring_actuator_controller.cpp`
14. `ffw_swerve_drive_controller/src/swerve_drive_controller.cpp`
15. `ffw_robot_manager/src/ffw_robot_manager.cpp`
16. `ffw_bringup/ffw_bringup/joint_trajectory_executor.py`

## 19. 최종 요약

F2+A3의 arm leader-follower 제어는 A3의 변환된 state를 custom broadcaster가
F2의 `JointTrajectoryController` 입력 토픽으로 직접 전달하는 구조다. 핵심
계약은 joint 이름/순서, A3 gain, gripper offset, F2 limit, TELEOP/IDLE/
SAVE_POSE 상태기계다.

현재 profile은 arm tact 처리는 유지하면서 A3 joystick 기반 head/lift/base
명령을 비활성화한다. foot switch는 좌우 save pose와 middle heartbeat를
담당한다. 시스템을 진단할 때는 A3 측정 state, broadcaster target, F2 controller
reference, F2 feedback을 서로 다른 데이터로 보고 같은 시간축에서 비교해야 한다.
