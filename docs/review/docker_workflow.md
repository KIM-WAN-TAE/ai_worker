# Docker Workflow: AI Worker 셸 별칭 유지

## 연결 커밋

```text
[REVIEW] Docker Workflow: preserve AI Worker shell shortcuts
```

## 해결하려던 문제

F1·F2 bring-up, Leader와 controller를 자주 실행하면서 긴 ROS 2 명령을 매번 입력해야 했다.
Dockerfile의 `RUN echo ... >> ~/.bashrc`에 별칭을 추가하면 새 이미지를 빌드한 뒤에는
사용할 수 있지만, 현재 source를 compose로 마운트해 검토하는 동안에는 이미지 재빌드
전까지 반영되지 않는다. 이미지와 개발 컨테이너의 별칭 목록이 서로 달라지는 문제도
생긴다.

이 커밋은 별칭을 `docker/bash_aliases` 한 파일에 두고 compose가
`/root/.bash_aliases`로 읽기 전용 마운트하도록 한다. 기본 Ubuntu `.bashrc`가
`~/.bash_aliases`를 source하므로 새 interactive shell부터 바로 같은 목록을 사용한다.

## 포함한 별칭

### 빌드

- `cb`: release 설정의 symlink-install colcon build

### Leader·Follower·AI bring-up

- `ffw_a3_leader`
- `ffw_lg2_leader_ai`
- `ffw_bg2_follower_ai`, `ffw_sg2_follower_ai`
- `ffw_bg2_ai`, `ffw_sg2_ai`
- `ffw_sh5_follower_ai`, `ffw_bh5_follower_ai`
- `ffw_f2_follower`, `ffw_f2_follower_ai`
- `ffw_f1_follower_ai`
- `ffw_f2_ai`, `ffw_f1_ai`
- `ffw_f2_bring_down`, `f2_bring_down`

F1·F2 호환 이름은 이 브랜치에 실제로 존재하는 접두어 없는 launch 파일
`f1_*.launch.py`와 `f2_*.launch.py`를 가리킨다.

### Controller와 Zenoh

- `motion_controller`
- `motion_controller_hand`
- `zenohd`

## Dockerfile과의 관계

이번 검토에서는 새 F1·F2 별칭을 ARM64 Dockerfile의 `.bashrc`에 다시 추가하지 않았다.
별칭 정의가 이미지와 mount 두 곳에 중복되면 어느 쪽이 최신인지 알기 어렵기 때문이다.
기존 Dockerfile 별칭은 그대로 두고, 이번에 정리한 운영 별칭은
`docker/bash_aliases`를 단일 검토 대상으로 사용한다.

compose mount는 다음 한 줄이다.

```yaml
- ./bash_aliases:/root/.bash_aliases:ro
```

읽기 전용이므로 컨테이너에서 별칭 파일을 수정해 host source를 우발적으로 덮어쓸 수
없다. source를 수정한 뒤에는 새 interactive shell을 열거나
`source ~/.bash_aliases`를 실행해야 현재 shell에 반영된다.

## 안전 범위

별칭은 기존 ROS 2 launch·run 명령의 짧은 이름일 뿐이며 안전 gate를 우회하지 않는다.
다만 `ffw_a3_leader`, Follower·AI bring-up과 controller 별칭은 실행 시 실제 hardware
프로세스로 이어질 수 있다. 별칭이 존재한다는 사실은 powered motion 승인이 아니며,
실행 전에는 해당 기능의 기존 현장 안전 절차와 사용자 승인이 필요하다.

`ffw_a3_leader`는 `launch_gripper_trigger:=true`를 명시한다. 첫 gripper 표본을
baseline으로만 저장하는 안전 수정은 별도 F2 Safety 커밋에 포함되어 있다.

## 검증 항목

- `bash -n docker/bash_aliases` 문법 검사
- 각 별칭이 참조하는 `ffw_bringup/launch/*.launch.py` 존재 확인
- interactive container shell에서 `.bashrc`의 `~/.bash_aliases` source 확인
- `ffw_f2_follower`와 `zenohd` 별칭 해석 확인
- `cyclo_motion_controller_ros`와 `rmw_zenoh_cpp` package 설치 경로 확인
- compose 설정에서 읽기 전용 mount 확인

## 검토자가 확인할 핵심

1. 운영자가 실제로 유지할 별칭 목록인지 확인한다.
2. F1 호환 별칭을 F2 robot-side 이미지에도 계속 둘지 결정한다.
3. source mount 방식으로 유지할지, 최종 main 반영 시 Dockerfile image에도 같은 파일을
   `COPY`할지 결정한다.
4. powered 명령 별칭을 별도 이름 규칙으로 더 명확히 표시할지 검토한다.
