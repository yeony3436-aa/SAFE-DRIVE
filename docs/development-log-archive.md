# SAFE-Drive Development Log

## Project Scope

SAFE-Drive는 ROS 2와 Gazebo를 이용해 자율주행 차량의 위험 시나리오를
생성하고, 제한된 시뮬레이션 예산 안에서 고위험 조건을 효율적으로 탐색하는
연구 플랫폼을 목표로 한다.

핵심 연구 질문은 다음과 같다.

> 동일한 시뮬레이션 예산에서 Risk-Guided Adaptive Grid Refinement가
> Random Sampling보다 고위험 시나리오를 더 효율적으로 발견할 수 있는가?

이 프로젝트는 실제 도로에서의 사고 확률을 추정하지 않는다. 정의된
시뮬레이션 시나리오 공간 안에서 위험 조건을 발견하는 효율을 비교하는 것이
연구 범위다.

전체 기능을 한 번에 구현하지 않고 다음 순서로 개발한다.

1. 차량 주행 베이스라인
2. Sudden Pedestrian 위험 시나리오
3. TTC 및 충돌 평가
4. 자동 반복 실험
5. Random Sampling
6. Risk-Guided Adaptive Grid Refinement
7. 두 탐색 방법의 공정한 비교

---

## 2026-09-25 — 초기 저장소 및 시뮬레이션 구성

### 목표

MVP-01의 시작점으로 최소한의 Gazebo 환경과 ego 차량을 구성한다.

```text
Gazebo 실행
  → 직선 도로 생성
  → ego 차량 생성
```

### 구현 내용

- ROS 2 워크스페이스 구조 구성
- `safe_drive_description` 패키지 생성
- `safe_drive_sim` 패키지 생성
- `safe_drive_bringup` 패키지 생성
- 60m 길이의 최소 직선 도로 월드 구성
- 4륜 ego 차량 URDF/Xacro 작성
- Gazebo 차량 스폰 launch 작성
- 실험 및 제어 설정 파일의 초기 구조 작성

초기 패키지 역할은 다음과 같이 구분했다.

| 패키지 | 역할 |
|---|---|
| `safe_drive_description` | 차량 링크, 조인트, 형상 및 관성 정의 |
| `safe_drive_sim` | Gazebo 월드, 차량 스폰 및 시뮬레이터 연결 |
| `safe_drive_bringup` | 사용자가 실행할 통합 launch 진입점 |

### 초기 실행 결과

- Gazebo Classic 11.10.2 실행 확인
- 최소 도로 월드 로딩 확인
- `safe_drive_ego` 차량 스폰 확인
- 기본 스폰 위치: `(-15.0, -1.75, 0.0)`

### 당시 미구현 항목

- ROS 2 차량 속도 제어
- odometry 발행
- waypoint 추종
- 목적지 정지
- 보행자 위험 시나리오
- 안전성 평가 및 자동 실험

---

## 2026-09-29 — MVP-01 차량 베이스라인 구현

### 1. 개발 목표 확정

위험 시나리오를 구현하기 전에 다음 흐름이 안정적으로 동작하는 것을
MVP-01의 완료 조건으로 정했다.

```text
Gazebo 실행
  → ego 차량 생성
  → ROS 2 속도 제어
  → odometry 발행
  → waypoint 추종
  → 최종 목적지 정지
```

이 단계에서는 YOLO, 차선 인식, 대시보드, TTC, 보행자 시나리오를 구현하지
않았다. 연구의 기반이 되는 차량 주행과 상태 피드백을 먼저 안정화했다.

### 2. 차량 모델과 Gazebo 설정 분리

차량의 기본 형상과 Gazebo 전용 구동 설정을 분리했다.

```text
safe_drive_description
  └── ego_vehicle_core.xacro
        └── 링크, 바퀴, 조인트, 질량, 관성, 충돌 형상

safe_drive_sim
  └── ego_vehicle.gazebo.urdf.xacro
        └── 바퀴 접촉 설정 및 Gazebo 구동 플러그인
```

관련 파일:

- `ros2_ws/src/safe_drive_description/urdf/ego_vehicle_core.xacro`
- `ros2_ws/src/safe_drive_description/urdf/ego_vehicle.urdf.xacro`
- `ros2_ws/src/safe_drive_sim/urdf/ego_vehicle.gazebo.urdf.xacro`

이 구조를 선택한 이유는 순수 차량 모델이 Gazebo 플러그인에 직접 종속되는
것을 줄이고, 시뮬레이터 전용 설정을 `safe_drive_sim`이 소유하도록 하기
위해서다.

### 3. ROS 2 차량 구동 및 odometry 연결

MVP 단계에서는 복잡한 Ackermann 조향 모델 대신 재현하기 쉬운 4륜
skid-steer 구동 방식을 사용했다.

사용한 Gazebo 플러그인:

```text
libgazebo_ros_diff_drive.so
```

ROS 2 인터페이스:

| 토픽 | 메시지 | 역할 |
|---|---|---|
| `/safe_drive/ego/cmd_vel` | `geometry_msgs/msg/Twist` | 직진 및 회전 속도 명령 |
| `/safe_drive/ego/odom` | `nav_msgs/msg/Odometry` | 차량 위치, 방향 및 속도 피드백 |

`cmd_vel`에서는 다음 값을 사용한다.

```text
linear.x  = 차량 직진 속도
angular.z = 차량 회전 속도
```

odometry는 약 50Hz로 발행되도록 설정했다.

### 4. 수동 차량 구동 검증

waypoint controller를 연결하기 전에 Gazebo 차량 자체가 ROS 2 명령으로
움직이는지 단독 검증했다.

검증 순서:

```text
simulation.launch.py 실행
  → cmd_vel 직진 명령 발행
  → odometry 위치 변화 확인
  → 정지 명령 발행
  → 최종 속도 확인
```

측정 결과:

```text
초기 위치
x ≈ -15.0 m
y ≈ -1.75 m

직진 후 위치
x ≈ -12.79 m
y ≈ -1.75 m
```

y 위치가 거의 변하지 않아 직진 명령에서 차선을 유지하는 것을 확인했다.
정지 후 선속도는 약 `0.00002 m/s`로 측정됐다.

회전 명령도 별도로 전달해 `angular.z` 명령에 따라 차량 방향이 변경되는
것을 확인했다.

### 5. `safe_drive_control` 패키지 구현

차량 구동과 odometry가 확인된 후 waypoint 추종을 별도 패키지로 구현했다.

```text
safe_drive_control/
├── launch/
│   └── waypoint_follower.launch.py
├── safe_drive_control/
│   ├── controller.py
│   └── waypoint_follower.py
├── scripts/
│   └── waypoint_follower
└── test/
    └── test_controller.py
```

`controller.py`의 책임:

- 각도 정규화
- 현재 waypoint 선택
- waypoint 도착 판정
- Pure Pursuit 기반 곡률 계산
- 속도 및 각속도 제한
- 최종 waypoint 감속
- 경로 완료 상태 관리

`waypoint_follower.py`의 책임:

- `/safe_drive/ego/odom` 구독
- ROS 2 파라미터 로드
- `/safe_drive/ego/cmd_vel` 발행
- odometry timeout 감시
- waypoint 진행 상태 기록
- 최종 목적지에서 지속적인 정지 명령 발행

ROS 2 입출력과 제어 계산을 분리해 제어 알고리즘을 Gazebo 없이 단위
테스트할 수 있도록 구성했다.

### 6. 제어 설정과 경로 구성

모든 제어값과 waypoint는 다음 파일에서 관리한다.

```text
config/controller.yaml
```

주요 설정:

```yaml
target_speed: 2.0
goal_tolerance: 0.5
lookahead_distance: 1.5
max_linear_speed: 3.0
max_angular_speed: 1.0
max_linear_deceleration: 1.0
control_rate_hz: 20.0
odom_timeout: 0.5
```

차량 스폰 위치와 같은 차선을 유지하도록 waypoint를 설정했다.

```yaml
waypoint_x: [-10.0, 0.0, 10.0]
waypoint_y: [-1.75, -1.75, -1.75]
```

초기 waypoint의 y 좌표는 `0.0`이었지만 차량의 스폰 위치가
`y=-1.75`였기 때문에 차량이 중앙선 쪽으로 이동할 가능성이 있었다. 모든
waypoint의 y 좌표를 `-1.75`로 맞춰 직선 차선을 유지하도록 수정했다.

### 7. 통합 launch 구성

Gazebo와 waypoint follower를 한 번에 실행하는 launch 파일을 추가했다.

```text
ros2_ws/src/safe_drive_bringup/launch/vehicle.launch.py
```

실행 명령:

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch safe_drive_bringup vehicle.launch.py
```

통합 실행 흐름:

```text
minimal_road.world
  → safe_drive_ego 생성
  → diff-drive 플러그인
  → odometry 발행
  → waypoint follower
  → cmd_vel 발행
  → waypoint 순차 통과
  → 최종 waypoint 감속 및 정지
```

실행 로그:

```text
[waypoint_follower] Following waypoint 1
[waypoint_follower] Following waypoint 2
[waypoint_follower] Following waypoint 3
[waypoint_follower] Final waypoint reached; vehicle stopped
```

### 8. 자동 추적 카메라 추가

Gazebo 기본 카메라가 고정되어 있어 이동하는 차량을 관찰하기 어려웠다.
`minimal_road.world`에 `track_visual` 카메라를 추가했다.

```xml
<gui>
  <camera name="user_camera">
    <track_visual>
      <name>safe_drive_ego</name>
      <static>true</static>
      <use_model_frame>true</use_model_frame>
      <xyz>-7 0 4</xyz>
    </track_visual>
  </camera>
</gui>
```

처음에는 차량 기준 뒤쪽 5m, 위쪽 3m로 설정했으나 차량과 너무 가까워
전체 주행 경로를 관찰하기 어려웠다. 뒤쪽 7m, 위쪽 4m로 오프셋을 조정해
조금 더 넓은 범위를 보면서 ego 차량을 자동 추적하도록 했다. 월드 SDF
파싱과 관련 패키지 빌드는 검증했으며, 실제 GUI 시점은 재실행 후 육안으로
확인한다.

---

## Troubleshooting

### T-01. 새 Xacro 공통 파일을 찾지 못함

#### 증상

공통 차량 모델 파일을 분리한 직후 다음 오류가 발생했다.

```text
No such file or directory:
install/safe_drive_description/.../ego_vehicle_core.xacro
```

#### 원인

Xacro의 `$(find safe_drive_description)`는 소스 디렉터리가 아니라 설치된
ROS 패키지 경로를 참조한다. 새 파일을 만들었지만 기존 `install` 경로에는
아직 해당 파일이 설치되지 않았다.

#### 해결

관련 패키지를 다시 빌드해 설치 경로를 갱신했다.

```bash
colcon build --symlink-install --packages-up-to safe_drive_bringup
```

이후 `xacro`와 `check_urdf`를 다시 실행해 정상 파싱을 확인했다.

#### 교훈

> ROS 패키지 경로를 통해 참조하는 Xacro 파일을 추가하거나 이동한 경우,
> 설치본을 다시 빌드한 뒤 검사해야 한다.

### T-02. 차량이 도로 밖으로 이탈한 뒤 계속 추락함

#### 증상

초기 수동 주행 중 차량이 도로 밖으로 이탈한 뒤 z 방향으로 계속
추락했다.

```text
odometry z ≈ -1433 m
```

#### 원인

- 도로 충돌 영역이 폭 8m의 유한한 영역이었음
- 도로 밖에는 충돌 가능한 지면이 없었음
- 마지막 `cmd_vel` 명령이 계속 유지됐음
- 바퀴 접촉 마찰과 조인트 감쇠 설정이 부족했음

#### 해결

- 도로 아래에 보조 `ground_plane` 추가
- 바퀴 조인트에 damping 추가
- 종방향과 횡방향 바퀴 마찰값 설정
- 수동 시험 후 명시적인 0 속도 명령 발행

정지 명령 예시:

```bash
ros2 topic pub --once \
  /safe_drive/ego/cmd_vel \
  geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

#### 결과

- 정지 상태에서 차량이 밀리지 않음
- 직진 시 y 위치를 안정적으로 유지함
- 도로 밖으로 나가더라도 무한 추락하지 않음

#### 교훈

> `cmd_vel` 기반 플러그인은 마지막 명령을 유지할 수 있으므로 수동 시험
> 후 반드시 정지 명령을 보내야 한다.

### T-03. ROS 2 launch가 waypoint 실행 파일을 찾지 못함

#### 증상

```text
executable 'waypoint_follower' not found on the libexec directory
```

설치 경로에는 파일이 존재했지만 launch에서 실행하지 못했다.

#### 원인

`waypoint_follower` 스크립트에 실행 권한이 없었다. `--symlink-install`에서는
설치 경로의 파일이 소스 스크립트를 가리키기 때문에 원본에도 실행 권한이
필요하다.

#### 해결

```bash
chmod +x ros2_ws/src/safe_drive_control/scripts/waypoint_follower
```

CMake에서는 실행 스크립트로 설치하도록 구성했다.

```cmake
install(
  PROGRAMS scripts/waypoint_follower
  DESTINATION lib/${PROJECT_NAME}
)
```

#### 결과

`vehicle.launch.py`에서 waypoint follower가 정상적으로 시작됐다.

### T-04. 차량이 최종 목적지를 지나쳐 정지함

#### 증상

첫 통합 주행에서 controller는 목표 도달을 판정했지만 차량이 관성 때문에
목표를 지나쳤다.

```text
목표 위치: x = 10.0 m
수정 전 최종 위치: x ≈ 10.74 m
오버슈트: 약 0.74 m
```

#### 원인

차량이 목표 허용 범위에 들어갈 때까지 약 2m/s를 유지한 후 바로 0 속도를
명령했다. Gazebo의 차량은 관성과 바퀴 가속도 제한을 가지므로 즉시 정지할
수 없었다.

#### 해결

최종 waypoint 접근 시 다음 관계를 이용해 정지 가능 속도를 계산했다.

```text
v_stop = sqrt(2 × a × d)
```

- `a`: 설정된 최대 감속도
- `d`: 목표 허용 범위까지 남은 거리

제어 명령은 목표 속도와 정지 가능 속도 중 작은 값을 사용하도록 수정했다.

```text
command_speed = min(target_speed, stopping_speed)
```

#### 결과

```text
목표 위치: x = 10.0 m
수정 후 최종 위치: x = 9.521 m
목표 오차: 약 0.479 m
허용 오차: 0.5 m
최종 속도: 약 0.00006 m/s
```

설정된 허용 범위 안에서 정지했다.

#### 교훈

> 물리 시뮬레이션에서도 관성을 고려해야 하므로, 위치 임계값에서 급정지하는
> 방식보다 남은 거리에 따른 감속 제어가 필요하다.

### T-05. Ctrl+C 종료 시 ROS context 오류 발생

#### 증상

통합 launch 종료 시 다음 오류가 발생했다.

```text
RCLError: Failed to publish: publisher's context is invalid
```

#### 원인

SIGINT 처리로 ROS context가 먼저 종료된 후 `finally` 블록에서 정지
메시지를 발행하려 했다. 이미 종료된 publisher를 사용한 것이 원인이었다.

#### 해결

정지 메시지 발행과 shutdown 전에 ROS context가 유효한지 확인했다.

```python
if rclpy.ok():
    node.stop()
```

```python
if rclpy.ok():
    rclpy.shutdown()
```

#### 결과

다음 프로세스가 오류 없이 정상 종료됐다.

```text
robot_state_publisher
waypoint_follower
gzserver
```

### T-06. XML lint가 패키지 스키마를 읽지 못함

#### 증상

샌드박스 환경에서 `ament_xmllint` 실행 시 ROS 패키지 스키마 URL을 읽지
못해 테스트가 실패했다.

```text
Failed to locate the main schema resource
```

#### 원인

`package.xml` 형식 오류가 아니라 테스트 환경의 외부 네트워크 접근 제한이
원인이었다.

#### 해결

네트워크 접근이 가능한 환경에서 동일 테스트를 다시 실행했다.

#### 결과

모든 `package.xml`이 유효한 것으로 확인됐고 전체 테스트가 통과했다.

#### 교훈

> 테스트 실패 시 코드 오류와 실행 환경 오류를 구분해야 하며, 실패 원인을
> 숨기지 말고 동일 검사를 적절한 환경에서 다시 수행해야 한다.

### T-07. 재실행 시 차량이 회전하며 도로를 이탈함

#### 증상

이전 실행에서는 차량이 직진해 목적지에 정상 정지했지만, 다시 실행했을 때
제자리에서 크게 회전하거나 도로를 이탈하는 현상이 간헐적으로 발생했다.

#### 확인 과정

1. 이전 실행에서 남은 ROS 2 또는 Gazebo 프로세스가 없는지 확인했다.
2. GUI를 끈 실행과 GUI를 켠 실행을 각각 다시 수행했다.
3. 두 재현 시험에서는 모두 최종 위치 약 `x=9.51`, `y=-1.75`에 정상
   정지해 매번 발생하는 Gazebo 물리 문제는 아닌 것으로 확인했다.
4. `/safe_drive/ego/cmd_vel`의 publisher는 `waypoint_follower` 하나뿐이어서
   중복 속도 명령 가능성을 제외했다.
5. waypoint controller에 첫 waypoint를 조금 지나친 pose를 직접 입력해
   제어 명령을 확인했다.

재현 입력과 결과:

```text
first waypoint: (-10.0, -1.75)
current pose:   (-9.4, -1.75)
goal tolerance: 0.5 m

command: linear=0.0, angular=1.0
waypoint_index: 0
```

#### 원인

현재 controller는 차량과 waypoint 사이의 직선거리가 `goal_tolerance` 이하일
때만 다음 waypoint로 넘어간다. 차량이 callback 지연이나 초기 물리 상태의
차이로 허용 반경을 한 번에 지나치면 이미 뒤에 있는 waypoint가 계속 목표로
남는다. 이때 controller가 뒤쪽 목표를 향해 회전하면서 차량이 도로를 이탈할
수 있다.

자동 추적 카메라는 GUI 관찰 위치만 바꾸며 차량 물리나 `/cmd_vel`을 변경하지
않는다. GUI와 headless 실행이 모두 정상 완료된 점과 controller 단독 재현
결과를 근거로, 카메라보다는 waypoint 통과 판정이 핵심 원인으로 판단했다.

#### 현재 상태와 후속 조치

원인은 확인했지만 controller 코드는 아직 수정하지 않았다. 다음 작업에서
거리 허용 조건뿐 아니라 경로 진행 방향을 기준으로 waypoint를 통과했는지
판정하도록 보강하고, 다음 회귀 검사를 추가한다.

- waypoint 허용 반경 안에 들어오면 다음 목표로 전환
- 허용 반경을 건너뛰었더라도 경로 진행 방향으로 통과했다면 다음 목표로 전환
- waypoint를 지나친 pose에서 회전 명령이 발생하지 않는 단위 테스트
- GUI와 headless 환경의 반복 통합 주행

#### 교훈

> 이산 주기로 동작하는 경로 추종기는 waypoint 반경 진입만 검사해서는 안
> 된다. 한 주기 사이에 목표 지점을 건너뛸 가능성까지 통과 판정에 포함해야
> 간헐적인 역회전과 경로 이탈을 방지할 수 있다.

---

## Verification Results

### 빌드

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

결과:

```text
Summary: 4 packages finished
```

검증된 패키지:

- `safe_drive_description`
- `safe_drive_sim`
- `safe_drive_control`
- `safe_drive_bringup`

### URDF/Xacro 검사

- 기본 차량 Xacro 파싱 성공
- Gazebo 차량 Xacro 파싱 성공
- `check_urdf` 성공
- 링크와 조인트 트리 확인

```text
base_footprint
  └── base_link
      ├── front_left_wheel
      ├── front_right_wheel
      ├── rear_left_wheel
      └── rear_right_wheel
```

### Gazebo 월드 검사

```bash
gz sdf --check ros2_ws/src/safe_drive_sim/worlds/minimal_road.world
```

결과:

```text
Check complete
```

### 단위 테스트

제어 알고리즘에 대해 다음 항목을 검증했다.

- 각도 정규화
- 전방 waypoint 직진
- waypoint 순차 변경
- 최종 waypoint 정지
- 뒤쪽 waypoint에 대한 회전
- 최종 목적지 감속
- 비어 있는 경로 거부

결과:

```text
6 passed
```

### 전체 패키지 테스트

```bash
colcon test
colcon test-result --verbose
```

최종 결과:

```text
28 tests
0 errors
0 failures
0 skipped
```

검증 항목:

- Python 단위 테스트
- Flake8
- PEP257
- CMake lint
- XML 검사

### 통합 주행

```bash
ros2 launch safe_drive_bringup vehicle.launch.py
```

확인된 동작:

1. Gazebo GUI 실행
2. 직선 도로 로딩
3. ego 차량 스폰
4. 자동 추적 카메라 설정 로드(실제 시점은 GUI에서 육안 확인 필요)
5. `/safe_drive/ego/odom` 발행
6. waypoint 1, 2, 3 순차 추종
7. 최종 목적지 감속
8. 목표 허용 범위 내 정지
9. Ctrl+C 종료 시 모든 프로세스 정상 종료

---

## Test Strategy

수동 주행, 토픽 확인, 통합 주행, 자동 테스트의 목적을 다음과 같이
구분한다.

| 검증 방법 | 목적 | 실행 시점 |
|---|---|---|
| 수동 `cmd_vel` 시험 | Gazebo 차량 구동 플러그인 단독 검증 | 차량 모델 또는 플러그인 변경 후 |
| ROS 토픽 확인 | 명령과 상태 데이터 연결 진단 | 차량이 움직이지 않거나 상태가 이상할 때 |
| 통합 주행 | Gazebo와 waypoint follower 전체 검증 | 주요 기능 변경 후 |
| `colcon test` | 알고리즘과 패키지 회귀 검사 | 코드 변경 후 |

수동 주행 시험에서는 controller가 없는 다음 launch를 사용한다.

```bash
ros2 launch safe_drive_bringup simulation.launch.py
```

자동 waypoint 주행에서는 다음 launch를 사용한다.

```bash
ros2 launch safe_drive_bringup vehicle.launch.py
```

`vehicle.launch.py` 실행 중 수동으로 같은 `/safe_drive/ego/cmd_vel` 토픽에
명령을 보내면 자동 controller와 수동 publisher가 충돌할 수 있으므로 함께
사용하지 않는다.

---

## Current Status

### MVP-01 — 기능 구현 완료, 안정성 보완 필요

- [x] Gazebo 실행
- [x] ego 차량 스폰
- [x] ROS 2 속도 제어
- [x] odometry 발행
- [x] waypoint 설정 분리
- [x] waypoint 순차 추종
- [x] 최종 목적지 감속
- [x] 허용 범위 내 정지
- [x] 통합 launch
- [x] 자동 추적 카메라 설정 및 SDF 검사
- [x] 제어 알고리즘 단위 테스트
- [x] 전체 패키지 빌드 및 테스트
- [ ] 중간 waypoint 통과 판정 보강 및 반복 주행 검증

현재 중간 waypoint의 허용 반경을 건너뛰면 이전 목표를 향해 회전할 수 있는
간헐적 문제가 확인됐다. MVP-02로 넘어가기 전에 이 판정을 수정하고 반복
주행으로 안정성을 확인한다.

### MVP-02 — 이후 작업

다음 개발 단계는 설정 가능한 Sudden Pedestrian 위험 시나리오다.

권장 구현 순서:

```text
보행자 모델 생성
  → 설정값으로 초기 위치 결정
  → ego 차량과 보행자의 좌표 확보
  → ego 거리 기반 트리거
  → 보행자 도로 진입
  → 충돌·통과·정지·시간 초과 종료 조건
  → 동일 설정과 seed로 재현성 검증
```

초기 설정 변수:

- `ego_speed`
- `pedestrian_speed`
- `pedestrian_spawn_distance`
- `trigger_distance`
- `seed`
- `timeout`

MVP-02가 안정적으로 재현된 이후 TTC, 최소 거리, 충돌 감지 및 위험 점수
계산으로 진행한다.

---

## Future Log Template

새로운 기능을 구현할 때 다음 형식으로 기록한다.

```markdown
## YYYY-MM-DD — 마일스톤 또는 기능 이름

### 목표

### 구현 내용

### 설계 결정과 이유

### 변경한 파일

### 트러블슈팅

#### 증상

#### 원인

#### 해결

#### 결과 및 교훈

### 검증 방법

### 검증 결과

### 남은 문제

### 다음 작업
```
