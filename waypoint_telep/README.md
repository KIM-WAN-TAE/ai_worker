# Waypoint Telep

`팔별 실행 파일`의 `실행 파일 추가 (여러 개)`에서 JSON 파일을 여러 개 선택하면,
파일의 `arm` 값에 따라 왼팔·오른팔 보관 목록에 각각 등록됩니다. 같은 이름의 파일도
별도 항목으로 등록되며, 등록만으로 현재 waypoint를 바꾸거나 로봇을 이동하지 않습니다.
각 팔에서 원하는 파일의 `활성화`를 누르면 편집·미리보기·실행 대상이 해당 파일로
바뀝니다. 팔마다 하나씩 독립적으로 활성화할 수 있으며, 실제 이동은 기존 실행 버튼으로
시작합니다. 실행 또는 키보드 제어 중에는 파일을 전환할 수 없습니다.

파일 전환 시 현재 활성 파일의 편집 내용을 보관하고, 등록되지 않은 기존 작업은
`현재 작업 (자동 보관)`으로 남깁니다. 전환한 팔의 순차 실행 단계는 기존 파일 불러오기와
같이 해제되므로 다시 추가하세요. 비활성 파일의 `제거`는 보관 목록에서만 제거합니다.
보관 목록은 UI 서버의 현재 실행 세션 동안 유지되어 브라우저를 새로고침해도 남지만,
프로그램 종료 시 초기화됩니다. 필요한 편집 결과는 `이 팔 파일 저장`으로 내려받으세요.
저장 버튼을 누르면 기본 이름(`left_waypoints.json` / `right_waypoints.json`)을 표시하며,
원하는 이름으로 바꿔 저장할 수 있습니다. `.json` 확장자는 생략하면 자동으로 붙습니다.

`구간 삭제`를 켠 뒤 목록에서 시작점과 끝점을 클릭하거나 번호를 선택하면,
양 끝점을 포함한 연속 구간을 한 번에 삭제할 수 있습니다. 삭제 개수와 범위를
표시하며, 숨겨진 보간점도 포함합니다. 목록이 바뀌면 삭제 선택을 해제합니다.
삭제한 팔을 사용하는 순차 실행 단계는 기존 삭제 동작과 같이 해제됩니다.

목록 또는 양팔 순차 실행의 `보간점 제외 보기`는 원본 지점만 표시합니다.
시작·종료 선택은 실제 waypoint 번호를 유지하고, 실행 경로의 중간 보간점은
그대로 사용합니다. 새 파일은 지점별 `is_interpolated` 표시를 저장합니다.
이름 변경·재정렬·삭제·파일 저장/불러오기에서도 이 표시를 유지합니다. 이전 파일의
`구간 N 곡선/직선 중간점 N`, `왼팔/오른팔 지점 N-보간M (N→N)`,
`지점 N-직선보간M` 형식도 구분합니다. 이전 내보내기가 이 지점을 잘못
`is_interpolated: false`로 기록한 경우, 버전 없는 파일을 불러올 때 알려진 이름 형식에
한해 복구합니다. 새 저장 파일에는 `interpolated_provenance_version: 2`를 기록하며,
이 버전의 명시적 true/false는 이름보다 우선합니다.

보간점 제외는 목록, 순차 실행 구간, 실행 시작·종료 및 상대 이동 기준 선택에 적용됩니다.
실행 경로·원래 waypoint 번호는 유지하며 표시만 줄입니다. 이미 설정된 실행 끝점이나
참조 지점이 보간점이면 그 선택은 남겨 두며, 필터를 켜는 것만으로 변경하지 않습니다.

`waypoint_telep`는 AI Worker 양팔의 현재 자세와 사용자가 지정한 Cartesian waypoint를
팔별 KDL IK로 변환해 미리 보거나 `command_source_arbiter`의 `WAYPOINT` source로 실제
실행하는 선택형 ROS 2 패키지입니다. 실행하지 않을 때는 기존 동작에 영향을 주지 않습니다.

UI에서는 관절 목표, 전역 고정 frame 기준 상대 XYZ/Roll/Pitch/Yaw 목표, 그리퍼 로컬 frame
기준 상대 XYZ/Roll/Pitch/Yaw 목표를 함께 사용할 수 있습니다. F2의 실제 TF tree에는
`world`가 없으므로 기본 고정 frame은 `odom`입니다. 패널에는 현재 자세 또는 선택한
waypoint의 `odom` 절대 pose가 표시됩니다. 위치 델타는 `odom` XYZ축으로 더하고, 회전
델타는 고정축 회전 `ΔR × R기준`으로 합성합니다. 최신 TF의
`odom → arm_base_link` 변환으로 목표를 바꾼 뒤 IK를 풀기 때문에 이동 베이스의 위치·yaw와
lift 위치가 모두 반영됩니다.

다음 두 경로 방식을 선택할 수 있습니다.

- `LINEAR`: 엔드이펙터가 Cartesian 직선을 따르도록 모든 중간점에서 IK를 계산합니다.
- `JOINT`: waypoint의 IK 목표를 계산한 뒤 관절 공간에서 부드럽게 이동합니다. 엔드이펙터
  경로는 직선이 아닐 수 있습니다.

`JOINT 연속 통과`는 기본으로 켜져 있으며, 선택 구간이 모두 `JOINT`인 경우 PCHIP 기반 cubic Hermite
spline으로 전체 구간을 한 번에 계획합니다. 현재 자세에서 첫 waypoint까지는 정지 정렬하고,
첫·마지막 waypoint와 대기 시간이 설정된 지점에서만 속도를 0으로 만듭니다. 나머지 중간
waypoint는 입력 관절값을 정확히 통과하면서 속도를 이어 갑니다. 이 기능은 Cartesian
`LINEAR` 보간을 사용하지 않습니다. 선택 구간에 `LINEAR` waypoint가 하나라도 포함되면
오류를 발생시키지 않고 기존 구간별 보간 방식으로 자동 전환합니다. 실행 직전의 실제 관절 자세에서 첫 waypoint까지는
quintic joint 보간으로 자동 정렬하므로, 사용자가 첫 waypoint 자세를 미리 맞출 필요가
없습니다.

UI 실행기는 허용 범위 안에서 발생한 publish 지연만큼 이후 명령의 전송 시각을 늦춥니다.
긴 trajectory에서 밀린 명령이 한꺼번에 전송되거나 작은 지연이 누적되는 일을 줄이기
위한 처리입니다. 관절 명령을 건너뛰지 않으며, 부하가 있으면 실제 실행 시간은 Preview보다
길어질 수 있습니다. 단일 전송 지연이 `max_schedule_lag_s`를 넘거나 제어권이 해제된
경우에는 기존처럼 중단합니다. 사용자 정지도 계속 적용됩니다.

왼팔과 오른팔 planner는 `/waypoint_telep/left`, `/waypoint_telep/right` namespace에서
독립된 waypoint 목록, 보간 방식, trajectory를 유지합니다. 관절값을 직접 입력한 waypoint는
`JOINT`에서 입력값을 정확한 목표로 사용합니다.
`LINEAR`에서는 해당 관절값을 FK로 Cartesian pose로 변환한 뒤, 중간점마다 IK를 계산합니다.
각 waypoint에는 도착 후 다음 waypoint로 출발하기 전의 대기 시간을 0~30초 범위로 저장할
수 있습니다. 실제 실행 중에는 대기 자세를 100 Hz command로 계속 보내므로 arbiter 제어권과
Record action stream이 유지됩니다. 마지막 waypoint에도 설정된 대기 시간을 적용한 뒤
실행을 완료하거나 다음 실행 시퀀스로 넘어갑니다.
또한 각 waypoint까지의 동작 속도를 기준 속도의 0.01~90배로 지정할 수 있습니다. 속도 배율은
Cartesian 이동, 관절 보간, 그리퍼 이동에 함께 적용됩니다. 기본값 `1`은 기존 설정과 같고,
`0.5`는 절반 속도, `2`는 2배 속도(200%)로 해당 구간을 실행합니다. 높은 배율은 50 Hz로
촘촘하게 저장된 짧은 구간의 취득 시간을 재현하기 위한 상한입니다. 큰 관절 변위를 높은
배율로 지정하면 실제 관절 속도도 크게 증가할 수 있으므로 Preview와 관절 간격 검사를 먼저
통과해야 합니다.

Waypoint JSON에는 선택적으로 `joint_timing`을 저장할 수 있습니다. 예를 들어
`{"speed_mps": 0.0327590555675, "preserve_start_waypoint": 225, "preserve_end_waypoint": 231}`은
JOINT 연속 통과 곡선의 일반 이동을 약 32.8 mm/s 기준으로 시간 재배정합니다.
저장 지점과 원래 관절 곡선은 유지하며, 실행 샘플은 첫 자세 정렬 이후 전체 시간축의
100 Hz 간격으로 생성합니다. 마지막 샘플 간격은 남은 시간만큼 짧아질 수 있습니다.
관절 속도와 가속도 제약, 출발·정지에서는 감속하고 감속 전환을 완화합니다.
끝단 등속 시간 배정의 탐색 상한은 `max_uniform_tcp_speed_mps`로 지정합니다.
기본 설정은 기준 20 mm/s의 90배인 1.8 m/s이며, 실제 속도는 관절 속도·가속도와
샘플 간격 제한 안에서 결정됩니다. 이전의 고정 100 mm/s 상한은 사용하지 않습니다.
지정한 구간으로 들어가는 첫 연결 구간부터 종료 지점까지는 원래 시간을 유지하므로
위 예에서는 225→231번 삽입과 224→225번 연결 구간을 보존합니다. 그리퍼만 움직이는
구간과 각 지점의 대기 시간도 유지합니다. 설정은 UI 파일 내보내기에 포함되며,
목록 삭제·재정렬 시에는 보존 구간 번호가 달라지므로 초기화됩니다.
이 시간 배정은 기구학 계획이며 모터 응답이나 삽입 접촉을 시뮬레이션하지 않습니다.

## 오차를 검증하는 JOINT 경로 보간

`경로 확인 → 경로 부드럽게 만들기`에서 팔별로 설정합니다.
기본은 꺼져 있으며, 끈 상태에서는 기존 LINEAR·JOINT·JOINT 연속 통과 방식을 사용합니다.

### 선택 구간의 곡선 노이즈

같은 화면의 `곡선 · 시작점 노이즈 추가`를 필요할 때만 켭니다. 기본 흐름은 기존 경로의
보간·스무딩이며, 곡선을 켠 경우에만 선택한 구간 안의 중간 경로를 다시 만듭니다.

- 구간별 시작점·끝점과 `곡선`/`직선`을 선택하고 `+ 구간 추가`로 이어 붙입니다.
  세 지점 전용 기능이 아니며 한 구간부터 사용할 수 있습니다. 다음 구간의 시작점은
  앞 구간의 끝점으로 이어집니다. 끝까지 선택한 경우 구간 추가는 마지막 구간을 나눕니다.
- 곡선 정도는 구간별로 조절합니다. 0%는 두 끝점 사이의 직선 기준 경로이며,
  스무딩 강도와는 별개입니다. 기존 중간 waypoint를 지나는 경로와 두 끝점만 연결하는
  경로는 다를 수 있습니다. 곡선 편차 기본 상한은 3 mm이고, 생성 개수(1~30)·편차·시드는
  `곡선 고급 설정`에 있습니다. 여러 경로는 휘는 방향을 달리하며 첫 경로를 적용합니다.
- `계산 후 Waypoint에 적용`은 곡선 생성 → IK → 독립 C++ 계획기 Preview 검증을 거친 뒤
  목록에 반영하고 현재 설정으로 다시 Preview합니다. 로봇 실행 명령은 보내지 않습니다.
  생성된 중간점이 목록에 들어가며 선택 구간 밖의 waypoint는 유지됩니다.
  구간 끝점의 정확한 관절 해·그리퍼·대기시간은 유지합니다. 구간 안에 대기 또는
  그리퍼 변경 지점이 있으면 해당 지점에서 구간을 나누도록 안내합니다.
- 중간 Cartesian 기준점을 거리·자세 변화에 따라 자동 생성한 뒤 실제 계획기의
  Cartesian → JOINT 보간, FK 오차(기본 1 mm/1°), 관절 제한·명령 연속성을 검사합니다.
  곡선 옵션을 끄면 기존 보간 흐름을 사용하며 저장 지점을 다시 생성하지 않습니다.
- 구간 모양을 바꾸면 이전 `joint_timing`은 사용하지 않습니다. 필요한 목표시간은
  생성 후 다시 배정하세요. 선택 구간 밖의 지점을 보존해도 보간·시간 배정이 달라질 수
  있으므로 원래 실행 시각이나 구간 밖의 연속 곡선까지 동일하다는 의미는 아닙니다.

별도 `/curve-generator` 화면에서는 파일을 불러와 선택 구간만 생성하고 JSON/ZIP으로
저장할 수도 있습니다. ZIP에는 Waypoint 파일, 정확한 시간·8축 관절값이 있는
`*_trajectory.json`, 검증 요약이 포함됩니다. 검증은 Fast DDS의 독립 ROS domain
221~229에서 `publish_enabled=false`, `direct_action_enabled=false`로 수행합니다.
실행 중인 ROS domain의 waypoint 목록·제어권은 생성 작업에서 변경하지 않습니다.
충돌·실제 로봇 추종 오차는 검사하지 않습니다.

곡선·직선의 모양은 생성 단계에서 Cartesian 기준으로 만들고 IK로 관절값을 구합니다.
새로 생성한 지점은 실행·저장 시 `interpolation_mode: JOINT`이며,
`joint_smoothing_enabled: true`로 저장합니다. 선택 구간 밖을 유지하는 옵션에서는
그 바깥의 원래 보간 방식을 그대로 보존합니다. 저장된 JOINT 경로로 실제 native
계획을 만든 뒤 직선 위치·자세 오차와 관절 연속성을 다시 검증합니다.
`부드럽게 연결`의 오차 검증이 켜져 있으면 별도의 JOINT 검증 곡선을 사용하며,
이때 일반 연속 통과 체크박스는 비활성화됩니다. 기존 LINEAR 기준 지점을 사용하는
파일은 그 기준 모드를 유지하므로 새 곡선 파일이 필요하면 다시 생성하세요.

HTTP API는 `POST /api/generator/curve-path`이며 예시는 다음과 같습니다.

```json
{"waypoint_file": {"schema": "ai_worker_waypoints/v1", "arm": "right", "waypoints": []},
 "segments": [{"start_waypoint": 186, "end_waypoint": 241, "mode": "CURVE", "curve_percent": 100},
              {"start_waypoint": 241, "end_waypoint": 247, "mode": "LINEAR"}],
 "preserve_outside": true, "count": 1, "max_deviation_mm": 3, "seed": 42}
```

`waypoint_file`에는 실제 기준 파일 전체를 넣습니다. API 자체는 live 목록에 적용하지
않으며, 검증한 첫 Waypoint 파일·미리보기·전체 ZIP(base64)을 반환합니다.

### 보간 세부 동작

`선택 구간의 첫 시작점 바꾸기`를 켜면 시작 pose의 위치·방향 노이즈도 만들 수 있습니다.
기본 화면에서는 회색 손잡이를 드래그해 축별 최소·최대 범위를 정하고, 파란 손잡이로
현재값을 조절합니다. 파란 손잡이 위에서 휠을 굴리면 0.1 mm/0.1° 단위로 바뀝니다.
숫자 입력은 `숫자로 범위 지정 · 고급`에 있으며, 드래그 범위는 버튼으로 넓힐 수 있습니다.

- 위치는 팔 기준 좌표계의 XYZ(mm), 방향은 원래 시작 자세의 로컬 Roll·Pitch·Yaw(°)입니다.
  `위치만`/`방향만`/`위치 + 방향` 중 고릅니다. 회전은 원래 quaternion 오른쪽에 로컬
  `Rz(yaw) Ry(pitch) Rx(roll)`을 곱합니다. 화면의 시작 자세 축도 함께 바뀝니다.
- `이 범위에서 무작위 시작점 생성`을 켜면 지정한 구간 안에서 축별로 균등 샘플링합니다.
  같은 시드에서 샘플 순서는 재현 가능하며 생성 개수를 늘려도 첫 시작점은 같습니다.
  축별 범위이므로 세 축을 합친 거리·회전각 상한과는 다릅니다. 생성 개수는 1~30개입니다.
- 첫 시작점만 바꾸며 이후 구간 끝점의 정확한 관절 해·그리퍼·대기시간은 유지합니다.
  바뀐 시작점에서 다음 접근점까지 위치·자세를 보간한 뒤 기존 IK 및 native JOINT
  계획기로 검증합니다. IK가 불가능하면 범위를 줄이도록 안내하며 목록에 적용하지 않습니다.
- 여러 경로의 ZIP에는 각 파일의 노이즈 범위·실제 적용값·기준 시작 pose·시드가 기록됩니다.
  목록에 적용하면 바뀐 pose가 새 기준이 되고 시작점 노이즈 옵션은 자동으로 꺼집니다.
  같은 오프셋이 다음 적용에서 의도치 않게 누적되지 않게 하기 위한 동작입니다.

API의 선택 필드 `start_noise` 예시:

```json
{"enabled": true, "mode": "both", "sampling": "random",
 "translation_min_mm": [-5, -5, -5], "translation_max_mm": [5, 5, 5],
 "rotation_min_deg": [-5, -5, -5], "rotation_max_deg": [5, 5, 5]}
```

`sampling: "manual"`에서는 `translation_mm`, `rotation_rpy_deg`를 지정하며, 각각
최소·최대 범위 안이어야 합니다. 생략하거나 `enabled: false`이면 기존 생성 결과를 유지합니다.

1. `부드럽게 연결`을 켜고 `적용하고 미리보기`를 누릅니다. 기본 연결 강도는 100%이며,
   기본 화면에서 연결 방식을 따로 고를 필요가 없습니다.
   기본은 중간점 자동 계산입니다. 구간별 이동 거리, 회전량, 관절 변화량에 따라 분할 수가
   달라집니다. 초기 간격은 최대 10 mm 또는 5도이며 오차 허용 기준과는 별개입니다.
   Cartesian 구간은 검증 결과에 따라 추가 세분화합니다. 원본 지점과 대기시간은 유지됩니다.
2. 감속 정도를 바꿔야 할 때는 `고급 설정`에서 연결 강도를 0~100% 범위로 조정합니다.
   기존에 저장한 강도는 유지하며, 100%가 아니면 기본 화면에도 현재 설정을 표시합니다. `0%`는 원본 waypoint에서
   정지하고, 값이 커질수록 구간 사이의 속도를 이어 줍니다. 생성된 중간점에서는 불필요하게
   정지하지 않습니다. 원본 지점과 대기 시간은 유지하며 대기 지점에서는 항상 정지합니다.
   이 숫자는 구간 연결 접선의 적용 비율이며 위치 오차의 백분율을 뜻하지 않습니다.
3. 위치(mm), 자세(도), JOINT(rad) 허용 오차와 최대 추가 재시도 횟수는 `고급 설정`에 있습니다.
   중간점 자동 계산을 끄면 구간당 개수를 직접 정할 수 있습니다. 9개를 입력하면
   두 원본 지점 사이에 중간점 9개를 만듭니다. 기존 파일에 자동 계산 설정이 없으면
   이전처럼 저장된 수동 개수를 사용합니다. 결과 요약은 기본 화면에,
   분할 수·오차·재시도 기록과 JSON 저장은 `계산 결과 자세히 보기`에 표시합니다.
   LINEAR 구간은 위치 직선과 quaternion SLERP를 기준으로 IK 중간점을 만들고,
   JOINT 구간은 관절값을 같은 비율로 나눕니다. 두 경우 모두 관절 공간의 cubic Hermite
   곡선으로 계획하며 명령 샘플과 그보다 최소 4배 촘촘한 샘플에서 FK 오차를 검사합니다.
   JOINT에서는 출발·정지에 따른 진행 속도 변화와 경로 이탈을 구별하기 위해 관절 직선에
   투영한 진행률을 기준으로 비교합니다. 첫 선택 waypoint로 정렬하는 구간은 경로 오차
   비교에서 제외하지만 관절 제한과 샘플 변화량 검사는 적용합니다.
4. 허용 오차를 넘으면 Cartesian 분할 수를 두 배로 늘리고 원본 구간 사이의 연결 강도를
   절반으로 낮춰 재시도합니다. JOINT 중간점 수는 재시도로 바꾸지 않습니다. 추가 재시도 0회는
   최초 계산만 수행합니다. 실제 적용 강도와 중간점 수는 진단 화면과 JSON에 표시합니다.
   실패하면 이전 Preview를 비우고 실행을 차단합니다.
5. 보간 검증 후 `목표시간 자동 배정`을 적용할 수 있습니다. 실제 보간 planner에서 각 구간의
   기준 시간을 측정하고 속도 배율을 계산한 뒤 전체 경로를 다시 검증합니다. 구간당 동일
   시간, 전체 목표시간, 제외 구간과 대기 시간 보존을 지원합니다. 실패 시 기존 설정을
   유지합니다. 현재 자세에서 첫 waypoint까지의 정렬 시간은 전체 목표시간에서 제외합니다.

이 보간을 사용할 때 IK는 이전 자세에서 먼저 계산합니다. 해가 관절 제한 근처에 있거나 계산에 실패하면
제한 안쪽의 여러 초기 자세에서 다시 탐색하며, 이전 자세와 가까운 해를 우선합니다.
가능하면 제한에서 5도(관절 가동 범위의 최대 5%) 이상 여유를 둡니다. 이 범위에서 해를
찾지 못하면 여유를 단계적으로 줄이되 기계적 경계에서 최소 0.0001 rad를 남깁니다.
큰 관절 변화가 필요한 해만 나오면 여유를 줄여 이전 자세와 연속된 해를 먼저 찾습니다.
수정된 관절값이 원래 목표 pose를 만족하는지 FK로 다시 확인하며, 최종 관절값을 단순히
잘라 맞추지는 않습니다. 다른 해로 전환하면서 샘플 변화량 제한을 넘으면 경로를 거부합니다.
사용자가 직접 지정한 JOINT 목표는 그대로 유지하므로 이 IK 여유 정책으로 변경하지 않습니다.

양팔 설정은 독립적이며 역순·혼합 경로, 그리퍼와 대기를 지원합니다. 원본 waypoint 목록은
유지하고 JSON에는 `path_refinement`를 저장합니다. 이 필드가 없는 기존 파일은 보간을
끄고 불러옵니다. 기존 `JOINT 연속 통과` 대신 이 보간을 사용합니다. `joint_timing`과
함께 사용하면 검증된 보간 곡선의 관절값과 접선을 유지한 채 시간축만 재배정합니다.
내부 보간점은 실행용으로만 사용하며 원본 waypoint 수·이름·대기 시간은 그대로입니다.

중간점 입력은 0~255개이며 API의 `subdivisions`는 중간점 수에 1을 더한 1~256입니다.
추가 재시도는 0~8회, Cartesian 최대 분할 수는 4096, 전체 경로점 상한은 50000,
명령 샘플 상한은 250000입니다. 명령 전송 빈도는 기존 제어 주기를 따르며 원본 지점의
도착과 대기를 위해 추가 샘플이 들어갈 수 있습니다. 검사에는 실제 로봇의 추종 오차나
충돌 검사가 포함되지 않으며 연속 곡선 전체의 수학적 오차를 보증하지 않습니다.

ROS 설정 topic은 `~/path_refinement`이며 `Float64MultiArray`에
`[enabled, subdivisions, smoothing_0_to_1, position_m, orientation_rad, joint_rad, retries]`를
보냅니다. 현재 설정은 `~/path_refinement_state`, 시도 기록은
`~/path_refinement_diagnostics`(`String` JSON)에 발행됩니다. HTTP에서는
`POST /api/path-refinement`에 `{"arm":"left","path_refinement":{...}}`를 보내며,
`GET /api/state`와 `/api/runtime`에서 진단 기록을 확인할 수 있습니다.

## 브라우저 UI

기본 launch는 waypoint 노드와 로컬 UI를 함께 실행합니다. F2 bringup에서
`/robot_description`과 `/joint_states`가 제공되는 상태로 다음 명령을 실행합니다.

```bash
ros2 launch waypoint_telep waypoint_telep.launch.py
```

브라우저에서 <http://127.0.0.1:8765>를 엽니다. Docker가 host network를 사용하므로 로봇
PC의 브라우저에서도 같은 주소로 접속할 수 있습니다. 기본 bind 주소는 로봇 PC 외부에서
접속할 수 없는 `127.0.0.1`입니다.

### 드릴 waypoint 생성기

드릴을 기준 데이터와 조금 다른 높이 또는 앞·뒤 위치에서 잡은 경우에는
<http://127.0.0.1:8765/generator>에서 별도의 드릴 경로 생성기를 사용할 수 있습니다.
기존 UI 상단의 `드릴 경로 생성기` 링크로도 열 수 있습니다.

생성기는 절대 드릴 pose를 요구하지 않습니다. 기준 파지 시점의 엔드이펙터 pose를
`G_ref`, 새 파지 pose를 `G_new`, 기준 작업 경로의 각 엔드이펙터 pose를 `W_ref[i]`라고
하면 다음 관계를 사용합니다.

```text
C        = inverse(G_ref) * G_new
W_new[i] = W_ref[i] * C
```

현재 UI는 드릴을 같은 방향으로 잡고 위치만 달라지는 작업을 대상으로 하므로 `C`의 회전은
단위 회전으로 두고, 평행이동을 기준 파지 시점 엔드이펙터의 로컬 X/Y/Z(mm)로 입력합니다.
앞·뒤 또는 상·하가 어느 축인지는 로봇 설치 방향으로 가정하지 않습니다. 기존 Teleop의
3D 화면에서 기준 파지 자세의 빨강 X, 초록 Y, 파랑 Z축을 확인한 뒤 해당 입력란에 부호를
포함해 넣어야 합니다.

생성기에서는 Cyclo와 동일한 F2 URDF·STL mesh를 사용한 3D 화면을 표시합니다.
청록색은 기준 경로, 분홍색은 보정 경로이며, 파지 완료 pose의 큰 XYZ축을
보면서 로컬 X/Y/Z 보정값을 0.5 mm 단위로 조정할 수 있습니다.

사용 순서는 다음과 같습니다.

1. 기준 `ai_worker_waypoints/v1` JSON을 불러옵니다.
2. 영상과 gripper 값으로 `파지 완료` waypoint 하나만 직접 입력합니다.
   생성기는 파지 시점을 자동으로 추정하거나 선택하지 않습니다. 0.1에서 0.9로 닫히는
   데이터라면 사용자가 영상을 보고 0.9 파지가 완료된 waypoint를 지정합니다.
3. `±0.5 mm` 버튼이나 숫자 입력으로 새로 잡은 위치와 기준 파지 위치의 차이를
   로컬 X/Y/Z(mm)로 조정합니다. 3D의 큰 XYZ축이 입력값에 맞춰 바로 이동합니다. 합성 크기는
   기본 50 mm 이내로 제한됩니다.
4. `보정 경로 생성 · Teleop에 적용`을 누릅니다. 선택한 팔의 현재 waypoint 목록을 기준
   파일로 교체하고, 첫 waypoint부터 경로의 마지막 waypoint까지 같은 상대 보정을 적용한 뒤 모든
   변경 지점의 KDL IK를 순서대로 다시 풉니다. 한 지점이라도 IK 또는 관절 제한 검사가
   실패하면 보정 전 목록을 유지합니다.
5. 생성된 JSON에는 보정된 pose와 8축 관절값, 기존 gripper·대기 시간·속도·보간 방식이
   함께 저장됩니다. 따라서 `보정 JSON 저장`으로 내려받은 파일을 기존 Waypoint Teleop에서
   바로 불러올 수 있습니다. 생성 직후에는 같은 경로가 Teleop 목록에도 이미 적용되어
   있습니다.

생성 시 IK와 URDF 관절 제한은 검사하지만 self-collision, 드릴과 지그의 충돌, 실제 드릴
TCP 오차는 검사하지 않습니다. `Teleop 경로 계산` 또는 기존 UI의 Preview로 먼저 확인하고,
실제 데이터 취득에서는 작은 보정량부터 늘리는 것을 권장합니다.

다른 PC에서 접속할 때는 다음처럼 전체 인터페이스에 bind하고 로봇 PC 주소를 지정합니다.
실행 로그의 `UI 바로 열기:` 뒤에 브라우저에서 바로 열 수 있는 주소가 표시됩니다.

```bash
ros2 launch waypoint_telep waypoint_telep.launch.py \
  ui_host:=0.0.0.0 ui_port:=8766 ui_public_host:=192.168.6.2
```

`Address already in use`가 나오면 같은 포트의 기존 launch가 아직 실행 중입니다. 기존
터미널에서 `Ctrl+C`로 종료한 뒤 하나만 다시 실행하십시오. 중복 실행된 UI는 이제 종료
처리를 완료한 뒤 exit code 1로 끝나며, 같은 launch에서 시작한 planner도 함께 종료됩니다.
Python 종료 중 `SIGABRT`(exit code -6)가 발생하거나 planner만 남지 않습니다.

UI에서는 다음 순서로 사용할 수 있습니다.

`로봇 경로 시뮬레이션` 오른쪽의 6-DOF 편집기에서는 선택한 팔의 전역 고정 frame 목표를
직접 만들 수 있습니다. `XYZ 이동`에서는 빨강·초록·파랑 화살표를 드래그하고,
`Roll/Pitch/Yaw 회전`에서는 같은 색의 회전 링을 드래그합니다. 옆의 XYZ(mm)와
Roll/Pitch/Yaw(degree) 입력란에 숫자를 직접 입력해도 3D 조작기와 즉시 동기화됩니다.
`현재 자세 불러오기` 또는 `선택 waypoint 불러오기`로 기준 pose를 바꿀 수 있으며,
`이 자세를 waypoint로 추가`를 누르기 전까지는 편집 중인 pose를 ROS로 보내지 않습니다.

1. 상단의 `왼팔` 또는 `오른팔` 탭에서 편집할 팔을 선택합니다.
2. 관절값을 직접 지정하려면 `현재값 복사`를 누르고 arm 7축과 gripper를 degree 단위로
   수정한 뒤 `목표값을 waypoint로 추가`를 누릅니다.
   또는 `관절값 리스트 (rad)`에 Joint 1부터 Joint 7과 Gripper의 radian 값 8개를 JSON 배열
   형식으로 붙여 넣고 `radian 리스트로 waypoint 추가`를 누릅니다.
3. 전역 고정 frame(`odom`)을 기준으로 이동하려면 먼저 계산 기준에서 `현재 실제 자세`, 마지막
   waypoint 또는 특정 waypoint를 선택합니다. 화면에 표시된 기준의 절대 XYZ/RPY를 확인한
   뒤 `ΔX/Y/Z`(mm)와 `ΔRoll/Pitch/Yaw`(degree)를 입력합니다. 위치는 그리퍼 방향과 무관하게
   항상 화면의 `WORLD FIXED (odom)` 축으로 이동하고, 회전도 같은 고정축을 기준으로 적용됩니다.
4. 그리퍼 기준으로 이동하려면 선택한 팔의 RGB 축을 확인하고 `ΔX/Y/Z`(mm)와
   `ΔRoll/Pitch/Yaw`(degree)를 입력한 뒤 `상대 목표 계산 후 waypoint 추가`를 누릅니다.
   계산 기준의 기본값은 마지막 waypoint입니다. 목록이 비어 있으면 현재 실제 자세를
   사용하며, 필요하면 `현재 실제 자세` 또는 목록의 `Waypoint N`을 직접 선택할 수 있습니다.
5. 그리퍼 상대 목표는 `기준 자세 × 상대 변환`으로, 전역 고정 frame 상대 목표는
   `Δodom × 기준 자세`로 절대 pose를 계산하고 KDL IK를 즉시 수행합니다. 도달할 수 없거나
   URDF 관절 제한을 벗어나면 추가하지 않습니다.
6. 목표를 추가하기 전에 도착 후 대기 시간과 해당 waypoint까지의 동작 속도를 입력할 수
   있습니다. 다중 waypoint 목록에서는 각 지점의 속도와 대기 시간을 다시 저장하거나
   `이 지점 삭제`로 원하는
   waypoint만 삭제할 수 있습니다. 중간 지점을 삭제하면 남은 절대 목표 순서로 경로를 다시
   계산합니다.
7. 세 종류의 목표를 원하는 순서로 반복해 여러 waypoint를 추가합니다. 화면의
   다중 waypoint 목록에서 순서, 절대 XYZ/RPY, 속도, 대기 시간을 확인할 수 있습니다. Waypoint
   카드를 클릭하면 경로를 다시 계산하고 시뮬레이션을 해당 도착 시점으로 이동합니다.
   Viewer에는 8축 목표 radian과 현재 자세 또는 직전 waypoint 대비 변화량이 표시됩니다.
8. 각 카드의 이름을 업무에 맞게 바꿀 수 있고, 카드를 드래그해 도착 순서를 바꿀 수 있습니다.
   `이 팔 파일 저장`은 이름, 자세, 8축 관절값, 그리퍼, 보간 방식, 속도, 대기 시간과 실행
   구간을 JSON으로 저장합니다. `실행 파일 추가 (여러 개)`로 파일을 보관 목록에 등록한 뒤,
   원하는 항목의 `활성화`를 누르면 파일에 지정된 팔의 현재 목록에 적용합니다.
9. 다른 팔 탭으로 전환해 별도의 waypoint와 보간 방식을 설정합니다. `시작`과 `종료`를
   지정하면 해당 팔의 선택한 waypoint 구간만 계획·실행합니다.
10. 선택 구간의 모든 지점이 `JOINT`이면 기본으로 켜진 `JOINT 연속 통과`가 적용됩니다. Preview에서 경로를
    확인합니다. 설정은 팔마다 독립적이며 waypoint JSON을 저장할 때 함께 기록됩니다.
11. `양팔 순차 실행`에 `왼팔 2→4`, `오른팔 1→3`처럼 단계를 추가하면 목록 순서대로
    실행합니다. 시작 번호를 더 크게 지정한 `왼팔 4→1`은 `4, 3, 2, 1` 순서로 실행합니다.
    각 단계의 `이것만 실행` 버튼을 누르면 전체 목록 대신 해당 단계 하나만 실행합니다.
    각 단계는 실행 직전의 실제 자세에서 다시 계획되며, 선택되지 않은 팔은 직전 명령
    자세를 유지합니다.
12. `선택한 팔 계산 및 재생`은 현재 탭의 팔만 재생합니다. `양팔 경로 계산 및 동시 재생`은
   양쪽 planner의 경로 계산이 모두 성공했을 때 같은 0초 기준으로 두 팔을 함께 재생합니다.
13. `키보드 WAYPOINT`를 누르면 현재 양팔 자세로 정지한 뒤 `WAYPOINT` 제어권으로
   전환합니다. 선택한 팔은 `W/S`로 앞·뒤(`world` ±X), `A/D`로 왼쪽·오른쪽
   (`world` ±Y), `R/F`로 위·아래(`world` ±Z) 방향을 이동합니다. 기본 속도는 5 mm/s이며
   앞·뒤·왼쪽·오른쪽·위·아래를 각각 1~50 mm/s 범위에서 다르게 지정할 수
   있습니다. 키나 화면 방향 버튼을 누르는 동안만 작은
   Cartesian 이동을 반복하고, 키를 놓으면 마지막 자세를 유지합니다. 입력란에 커서가
   있을 때는 단축키가 동작하지 않습니다. 제어권 전환 시 팔 관절은 실제 state를 기준으로
   정지하지만, 양쪽 그리퍼는 직전의 유효한 action 목표를 유지합니다. UI 재시작 뒤 이전
   action을 받지 못했으면 fresh feedback이 각 팔의 선택된 시작 waypoint gripper 값과
   action을 받지 못했으면 fresh feedback의 현재 그리퍼 값을 초기 handoff 목표로 사용하고,
   실행 trajectory의 첫 정렬 구간에서 시작 waypoint의 그리퍼 값까지 부드럽게 이동합니다.
   feedback이 오래됐거나 없으면 전환을 차단합니다.
14. `텔레옵 미리 연결`을 누르면 UI가 intervention service를 시작하고 격리된 Mini
   leader 양팔 입력이 fresh해질 때까지 기다립니다. 준비가 끝나도 arbiter는 `NONE`을
   유지하므로 아직 로봇 명령은 전달되지 않습니다. 이후 `상대 리더 TELEOP`를 누르면
   이미 준비된 연결을 재사용해 바로 `TELEOP`으로 전환합니다. 미리 연결하지 않은 경우에는
   `상대 리더 TELEOP` 버튼이 같은 준비 절차를 자동으로 수행합니다.
15. 키보드 제어 중 `상대 리더 TELEOP`를 누르면 UI가 intervention service를 준비하고
   격리된 Mini leader 양팔 입력이 fresh한지 확인한 뒤 `TELEOP`으로 전환합니다.
   양팔을 자동 활성화하지는 않습니다. 이후 움직일 팔의 리더 물리 버튼을 눌러야 하고, 버튼을
   누른 시점의 leader/follower EE pose가 relative 기준 자세로 새로 저장됩니다.
   브라우저 탭이 숨겨지거나 키를 놓으면 추가 이동은 즉시 멈추고, UI heartbeat가
   0.5초 동안 끊기면 arbiter가 `NONE`으로 전환됩니다. `정지 요청`도 언제나
   `NONE`을 선택합니다.
16. 실제 실행이 명시적으로 활성화된 경우에만 확인란을 선택해 현재 탭의 팔 실행 버튼을
   활성화할 수 있습니다. 실행 중에는 양팔 최종 command topic이 계속 기록됩니다. 선택하지
   않은 팔은 실행 시작 자세를 유지하며, 양팔 동시 버튼은 미리보기만 수행합니다.
17. 실행 직전 source가 `TELEOP`이면 UI가 `WAYPOINT`로 제어권을 넘겨 경로를 실행한 뒤
    command source를 `TELEOP`으로 복귀시킵니다. 단, waypoint 전환 시 해제된 리더 팔은
    자동 활성화되지 않습니다. 움직일 팔의 리더 물리 버튼을 다시 누르면 follower의
    종료 자세와 그 시점의 leader 자세를 새 relative anchor로 삼습니다. recorder는 이 전환
    동안 계속 실행되며 시작·중지되지 않습니다. 오류나 사용자의 정지 요청이 있으면
    자동 복귀하지 않고 `NONE`으로 닫힙니다.

`로봇 경로 시뮬레이션`은 F2 bringup이 발행하는 전체 `/robot_description`과 Cyclo
Intelligence가 제공하는 동일한 `ffw_description`·`open_manipulator_description` STL
mesh를 로드합니다. 평상시에는 양팔 모두 live joint state를 적용하고, preview를 재생하거나
특정 waypoint를 선택한 동안에만 계산된 trajectory 관절값을 해당 팔에 덮어씁니다. 왼팔
경로는 청록색, 오른팔 경로는 분홍색으로
표시됩니다. 경로를 계산한 뒤 slider 또는 `재생` 버튼으로 전체 F2 모델의 움직임과 양쪽
end-effector 경로를 확인할 수 있습니다. 화면을 드래그하면 시점이
회전하고 휠로 확대·축소할 수 있습니다. 목표 조작기의 화살표나 회전 링을 드래그하는 동안에는
시점 회전이 잠깁니다. Mesh는 표시하지만 mesh collision, self-collision
및 주변 물체 충돌은 계산하지 않습니다.

UI 없이 기존 ROS interface만 사용하려면 다음과 같이 실행합니다.

```bash
ros2 launch waypoint_telep waypoint_telep.launch.py start_ui:=false
```

## 기본 안전 동작

- 기본값은 `publish_enabled=false`이며 preview만 허용합니다.
- `/robot_description`에서 현재 팔의 KDL chain과 관절 제한을 읽습니다.
- `/joint_states`가 없거나 오래됐으면 실행하지 않습니다.
- `POLICY`가 command를 소유하고 있으면 UI의 takeover를 거부합니다.
- `TELEOP`에서 넘어올 때 양팔을 현재 자세로 정지시킨 뒤 경로를 다시 계산합니다.
- 실제 경로는 `/intervention/waypoint/arm_left|right`의 fresh pair로 보내며 arbiter가
  최종 topic의 유일한 publisher를 유지합니다.
- 양팔 source가 끊기거나 오래되면 arbiter watchdog이 `NONE`으로 fail-close합니다.
- 키보드 Cartesian 이동은 매 요청마다 최신 실제 자세를 기준으로 FK/IK를 다시 계산하며,
  waypoint 목록에는 추가하지 않습니다. 1회 이동은 최대 2.5 mm이고 관절 변화량 제한도
  동일하게 적용됩니다.
- 키보드 heartbeat가 0.5초 끊기면 `WAYPOINT` command 유지도 중단하고 `NONE`을 선택합니다.
- IK 실패, 비정상 값, 관절 제한 위반 또는 관절값 불연속이 있으면 전체 경로를 거부합니다.
- 정상 완료 때만 기존 source가 `TELEOP`이었던 경우 텔레옵으로 복귀합니다. 정지·오류 때는
  `NONE`을 유지합니다.

이 검사는 self-collision이나 주변 물체 충돌을 계산하지 않습니다. Preview 성공은 실제
로봇 구동 승인이 아닙니다.

## 목표시간 자동 배정과 목록 접기

Waypoint 목록 위에서 현재 선택한 팔의 시간을 자동 배정할 수 있습니다.

기본 시간 기준은 **전체 목표시간 · 끝단 등속**입니다. 예를 들어 `180`초를 입력하면
첫 waypoint 도착부터 마지막 waypoint의 대기 종료까지 총 180초가 되도록 끝단의
기준 이동 속도(mm/s)를 계산합니다. 실제 FK 경로 길이에 따라 진행 속도를 조절하므로,
waypoint 간격이 다른 곳에도 같은 시간을 강제로 배정하지 않습니다. 출발·정지, 곡률과
관절 속도·가속도 제약이 있는 곳에서는 감속합니다. 관절 곡선과 지점별 속도 배율은
수정하지 않고 시간축만 변경합니다. 현재 자세에서 첫 지점으로 정렬하는 시간은 별도입니다.

이 모드는 JOINT 연속 통과 또는 `부드럽게 연결`로 검증한 JOINT 경로에서 사용합니다.
부드럽게 연결을 켠 상태에서도 그대로 시간 배정·저장·재불러오기를 할 수 있습니다.
목록 전체를 정순으로 계산합니다. 목표시간은 양수이고
계획 시간 오차가 **1 ms 이하**여야 적용합니다. 기준 끝단 속도는 최대 100 mm/s이며
관절 제약은 그대로 적용합니다. 목표가 불가능하거나 검증에 실패하면 기존 설정을
유지합니다. 선택 제외는 하나의 연속 구간만 지원하며, 보존 구간과 등속 구간의 경계
속도가 충돌하면 적용하지 않습니다. 이때 경계 조건을 검토하거나 전체 구간에 적용하세요.

결과는 `joint_timing`으로 파일에 저장됩니다. 보존 구간이 없는 전체 등속은
`preserve_start_waypoint: 0`, `preserve_end_waypoint: 0`으로 저장하며, 다시 불러와도
같은 시간축을 사용합니다. UI에는 기준 mm/s와 목표·계획 시간을 표시합니다.
속도 배율 자체를 변경하는 아래 구간당 동일 시간 방식과 구분됩니다.
`기존 구간별 속도 사용`을 누르면 끝단 등속 시간축만 해제하여 원래 속도 배율을
다시 사용합니다. 실제 실행 또는 키보드 제어 중에는 적용·해제할 수 없습니다.

시간 기준에서 **구간당 동일 시간**을 선택하면 기존 방식을 사용할 수 있습니다.

- **모든 구간 이동시간 동일**: 목록 정순의 `1→2`, `2→3`, … 이동시간을 동일하게 맞춥니다.
- **선택 구간 제외하고 적용**: 입력한 도착 waypoint 번호의 구간은 기존 속도 배율과 이동시간을 유지합니다. `225-231, 240`처럼 범위와 개별 번호를 함께 쓸 수 있습니다. `225`는 `224→225` 구간을 뜻합니다.
- 구간당 시간을 초 단위로 입력하거나 비워둡니다. 빈칸 또는 `0`은 기존 속도 배율로 산출한 대상 구간 시간의 평균을 사용합니다. 허용 속도 배율 `0.01..max_waypoint_speed_scale`의 공통 시간 범위 안으로 자동 조정합니다. 직접 입력한 시간이 범위를 벗어나면 적용하지 않고 가능한 시간 범위를 안내합니다.

자동 계산 중 기존 샘플 간 관절·그리퍼 변화량 한도에 걸리면, 대상 구간의 공통 시간을
1.5배씩 늘려 최대 12회 다시 계획합니다. 제외 구간의 시간과 변화량 한도는 유지합니다.
허용 시간 범위 안에서 검증을 통과하지 못하면 되돌립니다. 직접 입력한 시간은 자동으로
늘리지 않으며 검증에 실패하면 적용하지 않습니다.

시간 배정은 구간의 관절 이동량 또는 Cartesian 위치·회전량과 그리퍼 이동량을
기존 계획기와 같은 식으로 계산하고 `새 속도 배율 = 기본 이동시간 / 목표 이동시간`을
배정합니다. 제어기 파라미터, waypoint 위치·개수, 보간 모드와 JOINT 연속 통과 설정을
바꾸지 않습니다. 현재 자세에서 첫 waypoint로 정렬하는 구간과 도착 후 대기 시간도
유지합니다. 거리가 다른 구간에 같은 시간을 배정하면 구간별 속도는 달라집니다.

적용할 때 전체 정순 궤적을 메모리에서 생성하고, 각 waypoint의 도착 시각 차이에서
직전 waypoint의 대기를 뺀 값을 검증합니다. 대상 구간의 이동시간 오차가 **1 µs 이하**이고
제외 구간의 기존 이동시간도 일치해야 확정합니다. 검증에 실패하면 모든 변경을
되돌립니다. UI에는 `시간 배정 검증 완료`, 적용 구간 수, 목표 시간, 최대 오차가 표시되고,
Preview 궤적도 갱신됩니다. 이 과정은 로봇 실행 명령을 전송하지 않습니다.
경로·속도·보간 설정을 바꾸면 UI에서 재검증 필요 상태로 표시합니다.
관절 상태 입력이 없거나 오래됐으면 첫 waypoint의 관절값을 시작 자세로 삼아
검증하며, 결과에 `첫 waypoint 기준 시뮬레이션`을 표시합니다. 이 자세는 계획 계산에만
사용하고 계산 후 원래 측정 상태를 복구합니다. `/joint_states`나 로봇 명령으로
발행하지 않으며 실제 실행에 필요한 관절 상태 수신 조건도 유지합니다.

기존 `joint_timing` 끝단 등속 프로필은 구간당 동일 시간 방식을 적용하면 해제됩니다. 그 프로필이 켜져
있다면 제외 번호는 기존 삽입 보존 범위 안에서 지정해야 합니다. 그 외 구간을 제외하면
기존 실제 이동시간이 달라질 수 있으므로 적용을 거부합니다. 기존 보존 범위는 제외 입력에
기본 표시됩니다. 배정한 속도 배율은 기존 waypoint JSON 저장·불러오기로 유지합니다.
역순 실행이나 시작점 변경 시 새로 생기는 진입 구간에는 같은 시간을 보장하지 않습니다.
검증 결과는 계획된 이동시간에 대한 결과이며 실제 진동 측정 결과는 아닙니다.

API: `POST /api/waypoints/equal-time`

```json
{"arm": "right", "total_duration_s": 180, "distribution": "uniform", "excluded_waypoints": []}
```

`distribution: "uniform"`은 끝단 등속 배정이며 `~/uniform_total_time`을 사용합니다.
이 필드를 생략한 기존 API 요청은 이전의 구간당 동일 시간 배정을 유지합니다.
`total_duration_s`와 `duration_s`를 동시에 지정할 수 없습니다. 구간당 시간 API는 다음과 같습니다.

```json
{"arm": "right", "duration_s": 0, "excluded_waypoints": [225, 226, 227, 228, 229, 230, 231]}
```

네이티브 노드는 `~/equal_segment_time`에 `[duration_s, 제외할 도착 번호, ...]`를 받아
원자적으로 적용·검증합니다. 성공 상태는 `WAYPOINT_TIMES_VERIFIED:`이며, HTTP 응답의
`verified`, `duration_s`, `updated`, `excluded`, `max_error_s`로 결과를 확인할 수 있습니다.
전체 목표시간 명령은 `~/equal_total_time`에 `[total_duration_s, 제외할 도착 번호, ...]`로
전달합니다. HTTP 응답의 `total_s`, `target_total_s`, `total_error_s`, `alignment_s`,
`execution_total_s`, `fixed_s`, `mode`로 총시간과 보존 시간을 확인할 수 있습니다.
UI 서버와 양팔 waypoint 노드를 모두 새 버전으로 실행해야 사용할 수 있습니다.

Waypoint 목록 제목 옆의 `▾` 화살표를 누르면 점 목록을 접고, `▸`를 누르면 다시 펼칩니다.
접어도 현재 궤적과 설정은 유지되며 목표시간 입력과 파일 저장·불러오기 기능은 사용할 수
있습니다. 접기 상태는 현재 브라우저 화면에서 유지하고 새로고침하면 펼쳐집니다.

## 빌드

AI Worker 컨테이너 안에서 실행합니다.

```bash
cd /root/ros2_ws
colcon build --symlink-install --packages-select waypoint_telep
source install/setup.bash
```

## Preview-only 실행

F2 bringup으로 `/robot_description`과 `/joint_states`가 제공되는 상태에서 실행합니다.

```bash
ros2 launch waypoint_telep waypoint_telep.launch.py
```

시작할 때 경로 방식을 지정할 수도 있습니다.

```bash
ros2 launch waypoint_telep waypoint_telep.launch.py path_mode:=joint
```

RViz의 Fixed Frame을 `arm_base_link`로 지정하고 팔별 clicked-point topic을 사용하면 됩니다.
왼팔은 `/waypoint_telep/left/clicked_point`, 오른팔은
`/waypoint_telep/right/clicked_point`입니다. `PointStamped`에는 자세가 없으므로 점을 추가할
당시 해당 팔의 엔드이펙터 자세를 클릭 waypoint에 유지합니다.

전체 pose를 지정하려면 다음 topic을 사용합니다.

```bash
ros2 topic pub --once /waypoint_telep/left/waypoint_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: arm_base_link}, pose: {position: {x: 0.4, y: 0.2, z: 0.5}, orientation: {w: 1.0}}}"
```

같은 topic에 현재 전역 고정 frame인 `frame_id: odom`을 지정하면 절대 pose로 입력할 수
있습니다. 노드는 TF의 `odom → arm_base_link`와 `/joint_states`를 적용한 뒤 IK를 즉시
검증합니다. 다음 예시는 `odom` 기준 XYZ(m)와 quaternion을 사용합니다.

```bash
ros2 topic pub --once /waypoint_telep/left/waypoint_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: odom}, pose: {position: {x: 0.4, y: 0.2, z: 1.5}, orientation: {w: 1.0}}}"
```

XYZ만 변경하고 현재 실제 RPY를 유지하려면 `absolute_position`을 사용합니다.

```bash
ros2 topic pub --once /waypoint_telep/left/absolute_position geometry_msgs/msg/PointStamped \
  "{header: {frame_id: odom}, point: {x: 0.4, y: 0.2, z: 1.5}}"
```

RPY만 변경하고 현재 실제 XYZ를 유지하려면 `absolute_orientation`을 사용합니다.

```bash
ros2 topic pub --once /waypoint_telep/left/absolute_orientation geometry_msgs/msg/QuaternionStamped \
  "{header: {frame_id: odom}, quaternion: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}"
```

그리퍼 로컬 좌표계의 상대 pose는 다음 topic으로 추가합니다. `frame_id`는 선택한
`tip_link`여야 하며 position은 meter, orientation은 quaternion입니다. 첫 점은 현재 자세,
다음 점부터는 직전 waypoint가 기준입니다. 노드는 목표 절대 pose를 계산한 뒤 IK가 성공한
경우에만 목록에 추가합니다.

```bash
ros2 topic pub --once /waypoint_telep/left/relative_waypoint geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: end_effector_l_link}, pose: {position: {x: 0.01, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}"
```

전역 고정 frame 기준 상대 pose는 `frame_relative_waypoint`으로 추가합니다. 다음 예시는
기준 자세에서 `odom +X` 방향으로 10 mm 이동합니다. orientation은 기준 자세에 왼쪽에서
곱할 고정축 회전 델타이며, 단위 quaternion을 보내면 자세를 유지합니다.

```bash
ros2 topic pub --once /waypoint_telep/left/frame_relative_waypoint geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: odom}, pose: {position: {x: 0.01, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}"
```

관절 목표값을 직접 지정할 때는 현재 arm의 관절 7개와 gripper 이름을 순서대로 모두
포함합니다. 값의 단위는 radian입니다.

```bash
ros2 topic pub --once /waypoint_telep/left/joint_waypoint sensor_msgs/msg/JointState \
  "{name: [arm_l_joint1, arm_l_joint2, arm_l_joint3, arm_l_joint4, arm_l_joint5, arm_l_joint6, arm_l_joint7, gripper_l_joint1], position: [0.0, 0.1, 0.0, -0.2, 0.0, 0.1, 0.0, 0.3]}"
```

현재 엔드이펙터 pose를 waypoint로 저장할 수도 있습니다.

```bash
ros2 service call /waypoint_telep/left/capture_current std_srvs/srv/Trigger
```

경로 계산과 RViz `Path` 발행은 다음 명령으로 수행합니다.

```bash
ros2 service call /waypoint_telep/left/preview std_srvs/srv/Trigger
ros2 service call /waypoint_telep/right/preview std_srvs/srv/Trigger
```

경로 방식은 실행 중이 아닐 때 선택합니다. 이 서비스와 `path_mode` topic은 이후 추가할
waypoint의 기본 보간 방식만 바꾸며, 현재 목록의 waypoint 방식은 유지합니다.

```bash
ros2 service call /waypoint_telep/left/select_linear std_srvs/srv/Trigger
ros2 service call /waypoint_telep/right/select_joint std_srvs/srv/Trigger
```

topic으로도 `linear` 또는 `joint`를 선택할 수 있습니다.

```bash
ros2 topic pub --once /waypoint_telep/left/path_mode std_msgs/msg/String "{data: linear}"
```

개별 waypoint의 보간 방식은 `waypoint_mode`에서 `[waypoint 번호, mode code]`로
지정합니다. `0`은 해당 waypoint까지 Cartesian 직선 보간, `1`은 관절 보간입니다.

```bash
ros2 topic pub --once /waypoint_telep/left/waypoint_mode \
  std_msgs/msg/Int32MultiArray "{data: [3, 1]}"
```

중간 waypoint부터 시작하려면 `start_waypoint`을 지정합니다. 앞쪽 waypoint의 trajectory를
잘라 재사용하지 않고, 현재 실제 자세에서 선택한 waypoint까지 새로 계획한 뒤 나머지
목록을 이어서 실행합니다.

```bash
ros2 topic pub --once /waypoint_telep/left/start_waypoint \
  std_msgs/msg/UInt32 "{data: 3}"
```

상대 이동 기준은 `relative_waypoint` 또는 `frame_relative_waypoint`을 보내기 전에
`relative_reference`로 지정합니다.
`-1`은 마지막 waypoint(목록이 비었으면 현재 자세), `0`은 현재 실제 자세, 양수는
1부터 시작하는 waypoint 번호입니다.

```bash
ros2 topic pub --once /waypoint_telep/left/relative_reference std_msgs/msg/Int32 \
  "{data: 2}"
```

Preview 결과는 다음 topic에서 확인합니다.

- `/waypoint_telep/{left,right}/preview_path`: 팔별 Cartesian 경로
- `/waypoint_telep/{left,right}/planned_trajectory`: 팔별 전체 관절 trajectory
- `/waypoint_telep/{left,right}/waypoints`: 팔별 순서형 절대 pose 목록
- `/waypoint_telep/{left,right}/current_world_pose`: 현재 엔드이펙터의 전역 고정 frame 절대 pose
- `/waypoint_telep/{left,right}/world_root_pose`: 전역 고정 frame에서 본 URDF root pose
- `/waypoint_telep/{left,right}/world_waypoints`: waypoint 목록을 전역 고정 frame pose로 변환한 값
- `/waypoint_telep/{left,right}/waypoint_dwells`: 같은 순서의 도착 후 대기 시간 목록
- `/waypoint_telep/{left,right}/waypoint_speed_scales`: 같은 순서의 동작 속도 배율 목록
- `/waypoint_telep/{left,right}/waypoint_modes`: 같은 순서의 보간 방식 목록
  (`0=LINEAR`, `1=JOINT`)
- `/waypoint_telep/{left,right}/waypoint_joint_values`: 목록 전체의 원본 8축 radian 값
- `/waypoint_telep/{left,right}/selected_start_waypoint`: 현재 선택한 실행 시작 waypoint
- `/waypoint_telep/{left,right}/planned_waypoint_targets`: waypoint별 도착 시각과 8축
  radian 목표를 `[time, joint1, ..., joint7, gripper]` 순서로 이어 붙인 목록. 선택한 시작
  waypoint보다 앞쪽 지점은 포함하지 않습니다.
- `/waypoint_telep/{left,right}/status`: 팔별 상태와 거부 원인

waypoint 편집 명령은 다음과 같습니다.

```bash
ros2 service call /waypoint_telep/left/remove_last std_srvs/srv/Trigger
ros2 service call /waypoint_telep/right/clear std_srvs/srv/Trigger
```

1번 waypoint의 도착 후 대기 시간을 1.5초, 해당 지점까지의 속도를 0.5배로 바꾸거나,
2번 waypoint만 삭제하려면 다음
topic을 사용합니다. 번호는 1부터 시작합니다.

```bash
ros2 topic pub --once /waypoint_telep/left/waypoint_dwell \
  std_msgs/msg/Float64MultiArray "{data: [1.0, 1.5]}"
ros2 topic pub --once /waypoint_telep/left/waypoint_speed_scale \
  std_msgs/msg/Float64MultiArray "{data: [1.0, 0.5]}"
ros2 topic pub --once /waypoint_telep/left/remove_waypoint \
  std_msgs/msg/UInt32 "{data: 2}"
```

## 실제 trajectory 전송

실제 로봇에서는 현장 작업자, 물리 E-stop, 비어 있는 작업 영역, 낮은 속도 설정과 기존
명령 source 중지를 별도로 확인해야 합니다. 처음에는 mock hardware에서 검증하십시오.

Mini leader의 출력은 반드시 `/intervention/teleop/arm_left|right`로 들어가야 합니다.
최종 arm topic에 mini leader와 arbiter가 동시에 publisher로 표시되면 실제 실행은
`final-topic publisher conflict`로 차단됩니다. 수정된 mini-leader launch의 기본 출력은
격리된 teleop topic입니다.

```bash
ros2 launch waypoint_telep waypoint_telep.launch.py publish_enabled:=true
```

브라우저에서 waypoint를 Preview로 확인한 뒤 `실제 실행`을 사용합니다. 정상 경로는
`TELEOP → WAYPOINT → TELEOP` 또는 `NONE → WAYPOINT → NONE`으로 전환됩니다.
현재 자세와 선택한 시작 waypoint의 차이가 커서 일반 경로 계산이 관절 간격 제한으로
거부되면 `시작점 저속 정렬 후 실행`을 사용할 수 있습니다. 이 실행은 현재 자세에서 첫
waypoint까지의 속도 배율만 `slow_start_speed_scale` 이하로 제한합니다. 첫 waypoint에
도착한 뒤에는 각 waypoint에 저장된 원래 속도 배율을 사용합니다.
`/waypoint_telep/{arm}/execute`의 직접 action 실행은 기본적으로 비활성화되어 있습니다.
이는 recorder가 학습용 action으로 사용하는 최종 command topic에 실행 구간이 빠지는 것을
막기 위한 설정입니다.

이 Mini leader launch와 intervention service의 기본값은
`teleop_reference_mode:=relative`입니다. UI의 `상대 리더 TELEOP`는 intervention
service를 준비하고 fresh한 Mini leader 양팔 입력을 확인한 뒤 arbiter를 `TELEOP`으로
전환합니다. 각 팔은 리더의 물리 버튼으로 별도
활성화해야 하며, 그 시점에 leader/follower EE pose anchor를 잡습니다. waypoint로
전환하면 안전을 위해 리더 양팔이 해제되므로, 다시 텔레옵할 때 리더 물리 버튼을
다시 눌러야 합니다. `정지 요청`은 버튼을 마우스나 터치로 직접 누른
경우에만 처리됩니다. waypoint 실행 여부와 관계없이 arbiter를
`NONE`으로 전환하므로, 텔레옵과 waypoint에 공통으로 사용하는 원격 arm command 정지
경로입니다.
화면에는 현재 arbiter mode와 latch된 fault도 표시됩니다. 이 정지 기능은 물리 E-stop을
대신하지 않습니다.

긴급 취소 요청은 UI의 다음 버튼으로 보냅니다. 이 기능은 물리 E-stop을 대신하지 않습니다.

```bash
UI의 `정지 요청` 버튼
```

UI는 초기 로딩과 waypoint 변경 시에만 전체 계획을 읽습니다. 350ms 주기
갱신은 `/api/runtime`에서 관절 상태·실행 상태·arbiter만 읽어, 대용량 waypoint
직렬화가 100Hz trajectory publish를 지연시키지 않도록 합니다.

## 데이터 기록

이 패키지는 기존 recorder를 시작·중지·재시작하지 않습니다. F2 recorder가 원래 수집하는
다음 최종 action topic에 텔레옵과 UI waypoint 명령이 같은 형식으로 연속 기록됩니다.

- `/leader/joint_trajectory_command_broadcaster_left/joint_trajectory`
- `/leader/joint_trajectory_command_broadcaster_right/joint_trajectory`

진단이 필요하면 recorder의 extra topic에 다음 신호를 별도로 추가할 수 있습니다.

- `/waypoint_telep/{left,right}/planned_trajectory`
- `/waypoint_telep/{left,right}/status`
- `/joint_states`
- `/arm_l_controller/controller_state` 또는 `/arm_r_controller/controller_state`

학습용 action은 기존 F2 계약대로 위의 최종 좌·우 command topic을 사용합니다. sparse
waypoint와 `planned_trajectory`는 계획 및 진단 정보이며 action label을 대체하지 않습니다.
