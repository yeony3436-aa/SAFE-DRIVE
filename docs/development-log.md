# SAFE-Drive Development Log

The original detailed baseline log is preserved in [development-log-archive.md](development-log-archive.md).

## Current Project Scope

2026-10-13부터 SAFE-Drive의 주 목표는 **ROS 2/Gazebo 자율주행 시뮬레이션
환경 구축**이다. 차량 외형, 도로, 횡단보도, 신호등, 보행자가 함께 동작하는
장면을 만들고 카메라·LiDAR를 ROS 2에서 사용 가능하게 한다. 레이더는
지원 방식과 구현 비용을 확인한 뒤 진행할 선택적 확장이다.

연구·실험은 환경 구축을 뒷받침하는 검증으로 둔다. 동일 설정에서 경로
완료, 적색 신호 정지, 녹색 진행, 보행자 상호작용, 센서 출력 등을 반복
실행해 실제 측정값과 한계를 기록한다. 실제 도로 사고 확률이나 실제 센서
성능을 추정하지 않는다.

기존 Random Sampling vs. Risk-Guided Adaptive Grid Refinement 계획은
선택적 후속 연구로 보존한다. 아래 과거 기록의 MVP 명칭과 당시 목표는
해당 날짜의 이력이며, 현재 우선순위는 이 절과 `AGENTS.md`를 따른다.

---

## 2026-10-13 — 프로젝트 방향 전환

### 결정

- 최종 산출물의 중심을 위험 시나리오 탐색 알고리즘에서 **동작하는
  ROS 2/Gazebo 자율주행 시뮬레이션 환경**으로 변경
- 차량·보행자·도로의 시각적 완성도와 신호등/횡단보도 동작을 핵심 범위에 포함
- 카메라와 LiDAR의 ROS 2 출력 검증을 핵심 범위에 포함
- 레이더는 물리적 정확도와 지원 여부를 구분해 검토할 확장으로 설정
- 기존 TTC·접촉 측정과 보행자 시나리오는 환경 검증에 재사용
- Random/Adaptive 비교와 100회 자동 실험 요구를 졸업 필수 조건에서 제외

### 현재 확인된 상태와 다음 단계

기존 차량 주행, 보행자 횡단, 안전 지표의 구현 기록은 아래에 보존한다.
현재 신호등 제어, 신호에 따른 차량/보행자 행동, 카메라, LiDAR, 레이더,
센서 기반 회피, 결과 파일 저장은 구현된 것으로 간주하지 않는다.
현재 YAML의 기본 `trigger_distance`는 `8.0 m`이다. 아래 10월 7일의
`3.5 m` 기본값과 충돌 결과는 당시 설정과 실행 이력이며 현재 기본
설정의 검증 결과로 재사용하지 않는다.

다음 통합 마일스톤은 작은 도로 장면에서 **신호 상태 발행 → 적색 정지선
정지 → 녹색 진행 → 보행자 횡단과 차량 대응**을 검증하는 것이다. 모델
개선과 센서 추가는 이 동작을 깨지 않는 단계로 진행한다.

### 검증 범위

이번 변경은 문서의 목표와 우선순위 수정이다. ROS 패키지 빌드나 Gazebo
동작 검증을 새로 수행한 결과를 뜻하지 않는다.

---

## 2026-09-25 — 초기 시뮬레이션 구성

### 완료 내용

- ROS 2 워크스페이스와 기본 패키지 구성
  - `safe_drive_description`: ego 차량 모델
  - `safe_drive_sim`: Gazebo 월드와 시뮬레이터 연결
  - `safe_drive_bringup`: 통합 launch 진입점
- 60 m 직선 도로와 보조 지면 구성
- 4륜 ego 차량 URDF/Xacro 작성
- Gazebo Classic에서 ego 차량 스폰 구현
- 차량 기본 스폰 위치 설정: `(-15.0, -1.75, 0.0)`

### 검증 결과

- Gazebo Classic 11.10.2 실행
- 최소 도로 월드 로딩
- `safe_drive_ego` 차량 스폰

---

## 2026-09-29 — MVP-01 차량 주행 베이스라인

### 완료 내용

- Gazebo 전용 차량 설정을 기본 차량 모델과 분리
- `libgazebo_ros_diff_drive.so`를 이용한 4륜 skid-steer 구동 구성
- ROS 2 차량 인터페이스 연결

| 토픽 | 메시지 | 역할 |
|---|---|---|
| `/safe_drive/ego/cmd_vel` | `geometry_msgs/msg/Twist` | 차량 속도 명령 |
| `/safe_drive/ego/odom` | `nav_msgs/msg/Odometry` | 차량 pose와 속도 |

- `safe_drive_control` 패키지와 waypoint follower 구현
- 제어 계산과 ROS 2 입출력 분리
- 직선 경로 waypoint를 `config/controller.yaml`로 분리
- Pure Pursuit 기반 조향과 속도 제한 구현
- 최종 waypoint 접근 시 정지 가능 속도를 계산해 감속
- odometry timeout 시 정지하는 fail-safe 구현
- 최종 목적지 도달 후 지속적인 정지 명령 발행
- Gazebo와 controller를 함께 실행하는 `vehicle.launch.py` 구성
- ego 차량 자동 추적 카메라 설정 추가

### 기본 경로와 제어값

```yaml
target_speed: 2.0
goal_tolerance: 0.5
lookahead_distance: 1.5
max_linear_deceleration: 1.0

waypoint_x: [-10.0, 0.0, 10.0]
waypoint_y: [-1.75, -1.75, -1.75]
```

### 검증 결과

- 수동 `cmd_vel` 직진·회전·정지 동작 확인
- `/safe_drive/ego/odom` 약 50 Hz 발행 확인
- waypoint 1, 2, 3 순차 추종 확인
- 최종 위치 약 `x=9.52 m`, `y=-1.75 m`에서 정지
- 목표 오차 약 `0.48 m`로 `0.5 m` 허용 범위 충족
- ROS 2 종료 시 관련 프로세스 정상 종료
- URDF/Xacro와 Gazebo SDF 파싱 성공
- 전체 4개 패키지 빌드 성공
- 당시 전체 테스트 28개 통과

---

## 2026-09-30 — 중간 waypoint 통과 판정 보강

### 문제

제어 주기 사이에 차량이 중간 waypoint의 허용 반경을 건너뛰면 이미 뒤에
있는 waypoint를 향해 회전할 수 있었다.

### 완료 내용

- 중간 waypoint에 경로 진행 방향 기준 통과 판정 추가
- 허용 반경에 들어오지 않았더라도 waypoint의 진행 평면을 통과하면 다음
  waypoint로 전환
- 첫 waypoint는 첫 번째 구간 방향, 이후 waypoint는 진입 구간 방향 사용
- 최종 waypoint는 기존 거리 허용 조건만 사용해 조기 완료 방지
- 첫 waypoint와 중간 waypoint를 지나친 pose에 대한 회귀 테스트 추가
- 최종 waypoint를 지나친 pose가 완료로 오판되지 않는 테스트 추가

### 검증 결과

- controller 단위 테스트: `9 passed`
- `safe_drive_control` 빌드 성공
- `colcon test --packages-select safe_drive_control` 통과
- 누적 ROS 테스트 결과: `31 tests, 0 errors, 0 failures, 0 skipped`
- headless Gazebo 통합 주행 성공
  - waypoint 1 → 2 → 3 순차 진행
  - 최종 위치 `x=9.51 m`, `y=-1.75 m`
  - waypoint follower, Gazebo, robot state publisher 정상 종료

---

## 2026-09-30 — MVP-02 Sudden Pedestrian

### 완료 내용

- `safe_drive_scenario` 패키지와 보행자 Gazebo 모델 추가
- YAML 시나리오 정의와 실행 로직 분리
- 최초 ego 위치 기준 횡단 위치 계산
- ego 거리 기반 보행자 횡단 트리거
- 완료 및 timeout 상태 처리
- 현재 시나리오와 실행 상태 ROS 2 토픽 발행
- 시나리오의 `ego_speed`를 waypoint controller에 전달
- `scenario.launch.py` 통합 실행 진입점 추가

### 검증 결과

- 시나리오 상태 기계 단위 테스트 6개 통과
- 기존 controller 회귀 테스트 9개 통과
- 전체 5개 패키지 빌드 성공
- 전체 테스트: `39 tests, 0 errors, 0 failures, 0 skipped`
- headless Gazebo에서 보행자 스폰과 다음 상태 전이 확인

```text
WAITING_FOR_EGO → ARMED → ACTIVE → COMPLETED
```

- 횡단 위치: `x=-5.00 m`
- ego 최종 위치: `x=9.51 m`, `y=-1.75 m`
- 종료 시 모든 ROS 2 및 Gazebo 프로세스 정상 종료

---

## Historical Implementation Status (recorded before the direction change)

### MVP-01 — 완료

- [x] Gazebo 실행
- [x] ego 차량 스폰
- [x] ROS 2 속도 제어
- [x] odometry 발행
- [x] 설정 기반 waypoint 추종
- [x] 중간 waypoint 통과 판정
- [x] 최종 목적지 감속 및 정지
- [x] 통합 launch
- [x] 제어 알고리즘 단위 테스트
- [x] headless 통합 주행 검증

### MVP-02 — Sudden Pedestrian 실행 베이스라인 완료

- [x] 설정 기반 보행자 스폰 및 횡단
- [x] ego 거리 기반 트리거
- [x] 완료 및 timeout 상태 처리
- [x] 통합 launch와 headless 실행 검증

### MVP-03 — 실시간 안전 평가 구현

- [x] Gazebo 모델 상태로 ego·보행자 중심 간 거리와 상대속도 측정
- [x] 접근 중인 경우의 line-of-sight TTC 및 최소값 계산
- [x] Gazebo 접촉 센서로 보행자 충돌 감지
- [x] 속도 명령 감소율 기준 emergency braking 기록
- [x] 시나리오 완료/timeout을 포함한 JSON 결과 토픽 발행
- [ ] 결과 파일 저장 및 자동 반복 실행

## 2026-10-07 — MVP-03 Ground-Truth Safety Evaluation

`safe_drive_safety` 패키지를 추가하고 계산 로직을 ROS 노드에서 분리했다.
Gazebo 모델 상태는 20 Hz로 사용한다. 중심 간 거리와 현재 두 물체를 잇는
방향의 접근 속도로 TTC를 계산하며, 비접근·동일 위치에서는 TTC를 `null`로
남긴다. 이 TTC는 궤적 교차 예측이나 차체 외곽 간 거리가 아니다.

실제 충돌 여부는 ego 차체의 Gazebo contact sensor로 판정한다. URDF가
SDF로 변환되며 차체 collision 이름이 바뀌는 것을 확인하고 센서 참조를
변환 후 이름에 맞췄다. `emergency_braking`은 현재 차량 실감속도가 아니라
설정한 임계값 이상의 속도 명령 감소율이다.

검증 결과:

- 전체 관련 패키지 6개 빌드 성공
- 안전 계산 단위 테스트 5개 통과
- 기본 설정 headless 실행: `min_distance=2.5001 m`, `min_ttc=2.5002 s`,
  `collision=false`, `scenario_completed=true`
- 임시 충돌 유도 설정 headless 실행: `min_distance=0.7269 m`,
  `min_ttc=0.3668 s`, `collision=true`, `state=TIMED_OUT`
- 모델 상태와 접촉 토픽 발행 및 결과 JSON 수신 확인

당시 계획: 결과를 기계 판독 가능 파일에 저장하고 자동 반복 실험을 구현한다.
통합 실행 종료 시 중복 SIGINT로 일부 Python 노드가 `KeyboardInterrupt`
traceback을 출력했다. 실행 중 안전 결과는 정상 발행됐으나 종료 로그는
추후 정리해야 한다.

차량 pose와 controller 상태를 함께 초기화하는 전용 reset 기능은 반복
검증이 필요할 때 구현한다.

---

## 2026-10-07 — 피드백 반영 및 보행자 이동 오류 수정

앞 절의 기본 시나리오 결과(`collision=false`, 최소 거리 약 2.50 m)는
보행자가 실제 Gazebo 월드에서 움직이지 않은 상태에서 얻은 값이므로
**유효한 위험 시나리오 결과로 사용하면 안 된다**. 상태 기계는 완료됐지만
Gazebo `/model_states`의 보행자 y 좌표는 시작값 `-4.25 m`에 머물렀다.

원인은 시나리오 매니저가 존재하지 않는 `/set_entity_state`를 기다린 반면,
월드 상태 플러그인은 `/safe_drive/gazebo/set_entity_state`를 제공한 것이다.
서비스 주소를 수정하고 서비스가 준비되지 않으면 시나리오 진행을 멈추도록
했다. 보행자 모델의 `static` 설정은 그대로 유지했다.

그 밖에 시나리오·안전·제어 기본 YAML을 각 ROS 패키지 안으로 옮겨 패키지
빌드의 저장소 루트 의존성을 없앴다. 통합 launch는 공통 시나리오 로더로
`ego_speed`를 검증하고, 차량 odometry는 `WORLD=1`을 명시했다.
`.pytest_cache/`도 Git 무시 목록에 추가했다.
최종 waypoint를 주행 방향으로 지나쳤더라도 경로의 허용 폭 안에 있으면
정지하도록 제어기를 보강하고, 경로 밖 통과는 완료로 처리하지 않는 테스트를
추가했다.

수정 후 headless 실행에서 Gazebo 보행자 y 좌표가 최종값 `4.25 m`에
도달하고, 기본 설정의 결과는 `collision=true`, `min_distance≈0.73 m`,
`min_ttc≈0.47 s`로 바뀌었다. 현재 기본 `trigger_distance=3.5 m`는
충돌 유도 데모 설정으로 둔다. 이는 정의된 시뮬레이션 시나리오 공간의
결과이며 실제 도로 사고 확률 추정이 아니다.

최종 확인에서 관련 6개 패키지 빌드, control 12개·scenario 7개·safety
5개·bringup 10개 테스트가 각각 통과했다. 기본 시나리오를 다시 실행해
`collision=true`, `min_distance=0.7300 m`, `min_ttc=0.4730 s`,
`scenario_completed=true`와 차량 목적지 정지를 확인했다.

---

## Standard Verification Commands

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
colcon test
colcon test-result --verbose
ros2 launch safe_drive_bringup vehicle.launch.py
ros2 launch safe_drive_bringup scenario.launch.py
```

---

## 2026-10-10 — 도로 시각 구성과 차량·보행자 비율 조정

- 8 m 양방향 도로에 보도, 연석, 녹지, 횡단보도, 정지선, 횡단 표지와 나무를
  시각 요소로 추가했다. 기본 횡단 위치는 `x=-5 m`다.
- 길이 0.9 m의 시험 차량을 길이 4.2 m·폭 1.8 m 승용차 크기로 바꾸고,
  바퀴 간격·지름·구동 토크·차체 관성을 함께 조정했다.
- 기본 보행자 `trigger_distance`를 3.5 m에서 8.0 m로 바꿨다. 현재 제어기는
  보행자를 인지하거나 회피하지 않는다. 기본 설정의 무충돌은 횡단 시작 시점
  조정에 따른 것이다. 충돌 유도 설정은 별도 YAML로 재현할 수 있다.

검증 결과:

- Gazebo SDF 및 차량 Xacro→SDF 파싱 성공, 관련 6개 패키지 빌드 성공
- 격리된 Gazebo Classic headless 차량 단독 주행: 최종 정지 위치
  `x=9.51 m`, `y=-1.75 m`
- 기본 보행자 시나리오: `COMPLETED`, `collision=false`,
  `min_distance=2.80 m`, `min_ttc=1.72 s`, 최종 정지 위치
  `x=9.50 m`, `y=-1.75 m`
- 초기 전체 질량 설정에서는 차량이 도로에 내려앉아 시나리오가 timeout됐다.
  관성·바퀴 구동 설정을 조정한 뒤 위의 결과로 재검증했다.

제약: 횡단보도 도색은 기본 위치에 고정되어 있다. 실험에서
`pedestrian_spawn_distance`를 변경할 때 도색 위치는 아직 따라 움직이지
않는다. 충돌 후 차량 경로가 크게 벗어날 수 있으며, 물리 정확도와 능동
보행자 회피는 현재 검증 범위 밖이다.

---

## 2026-10-15 — 차량 외관과 도로 주변 시각 요소 개선

- 차량에 지붕, 앞뒤 필러와 옆창 구분, 그릴, 범퍼, 좌우 전조등·후미등,
  사이드미러, 측면 하단 장식, 휠 허브를 URDF 기본 형상으로 추가했다.
- 보도 띠, 화단, 가로등 두 개, 도로에서 떨어진 작은 건물 두 개를 월드에
  추가했다. 모두 시각 요소이며 외부 에셋은 사용하지 않았다.
- 차체·바퀴·도로의 충돌 형상과 경로, 횡단보도, 보행자 설정은 유지했다.

검증: 차량 Xacro와 URDF 파싱, 월드 XML 파싱, 관련 두 패키지 빌드 성공.
`safe_drive_description` 및 `safe_drive_sim` 테스트는 46개 통과했다.
처음 샌드박스 실행에서는 ROS 패키지 XML 검사에 필요한 외부 스키마에
접근하지 못해 실패했고, 네트워크 접근이 가능한 실행에서 재검증했다.
Gazebo Classic headless 통합 시나리오에서 차량과 보행자가 생성되고
보행자 상태가 `COMPLETED`까지 진행했으며, 차량이 최종 위치
`x=9.50 m, y=-1.75 m`에서 정지했다. 이번 실행의 안전 지표는 파일로
저장하지 않았으므로 이전 기록의 수치를 새 결과로 인용하지 않는다.
GUI 화면에서의 최종 외관 평가는 아직 수행하지 않았다.

---

## 2026-10-15 — Prius 참고 장면, 횡단보도와 보행자 외관 수정

- `osrf/car_demo`의 Prius 차체·휠 메시와 MCity 아스팔트·콘크리트·잔디
  텍스처를 선별해 사용했다. 출처, 버전, 라이선스 선언은
  `docs/asset-inventory.md`에 기록했다. 원본의 ROS 1 플러그인과 대형
  MCity 월드는 가져오지 않았다.
- 차체 메시는 차량 진행축 +x에 맞춰 회전·축소했고, 기존 단순 충돌 형상과
  ROS 2 차동 구동을 유지했다. 이전 기본 형상 외관은 Xacro 옵션으로 남겼다.
- 횡단보도를 보행 진행 방향의 넓은 흰색 막대 5개와 보도 진입부로 다시
  그렸다. 중심은 시나리오와 같은 `x=-5 m`다.
- 보행자 외관을 재킷, 머리·얼굴, 팔·손, 바지, 보행 자세의 다리·신발로
  교체했다. 원래 원통 충돌 형상과 위치 이동 방식은 유지했다. 관절 보행
  애니메이션은 아직 없다.
- Gazebo 리소스 경로에 로컬 재질을 등록하고, 기본 모델·셰이더 경로도
  launch에서 보존하도록 했다. 초기 시도에서 기본 경로가 빠져 Gazebo가
  원격 모델 데이터베이스를 기다리며 멈춘 문제를 이 설정으로 해결했다.

검증: 차량 Xacro/URDF와 월드·보행자 SDF XML 파싱, 관련 3개 패키지 빌드,
46개 테스트 통과. Gazebo Classic headless 시나리오에서 차량·보행자 생성,
보행자 `COMPLETED`, 차량 최종 정지 `x=9.50 m, y=-1.75 m`를 확인했다.
GUI 렌더러 로그에서 Prius 차체/휠 텍스처와 세 가지 도로 텍스처 로딩을
확인했다. UI 화면 캡처를 통한 미적 평가는 아직 수행하지 않았다.
이번 실행의 안전 지표는 결과 파일로 저장하지 않았다.

---

## 2026-10-15 — 작은 사거리와 차량 신호 대응

- 기존 직선 월드와 주행·보행자 런치를 유지하고 별도의
  `signal_intersection.world` 및 `signal_intersection.launch.py`를 추가했다.
  교차 도로, 정지선, 신호등 외관은 기존 횡단보도 `x=-5 m`와 맞췄다.
- 신호 노드는 시뮬레이션 시간에 따라 RED 12초, GREEN 16초, YELLOW 3초를
  반복한다. 고정 RED/GREEN 모드도 제공하며, ROS 상태 발행과 Gazebo 램프
  위치 갱신에 동일한 상태를 사용한다.
- 신호를 사용하는 직진 경로 제어기는 빨강/노랑/상태 누락 시 차량 전면이
  `x=-8.2 m` 정지선보다 0.6 m 앞에서 멈추도록 감속한다. 녹색에는 기존
  경로를 계속 따른다. 기존 런치는 이 제한을 활성화하지 않는다.

검증: 관련 네 패키지 빌드 성공. 헤드리스 Gazebo 고정 RED 실행에서 차량
중심 `x=-10.91 m`, 전면 약 `x=-8.81 m`로 정지했으며 빨간 램프는
`z=2.85 m`, 초록 램프는 숨김 위치 `z=-2 m`임을 Gazebo 서비스로 확인했다.
고정 GREEN 실행에서 차량이 `x=9.50 m, y=-1.75 m`에 도착했고 초록
램프가 `z=2.05 m` 표시 위치에 있음을 확인했다. 보행자 신호 연동과
센서 기반 판단, 결과 파일 저장은 다음 작업이다. GUI 화면에서 신호등과
사거리의 최종 외관은 아직 확인하지 않았다.
기본 순환 실행에서도 RED 12초 뒤 GREEN 전환과 차량 출발, 최종
`x=9.51 m, y=-1.75 m` 정지, 28초 YELLOW 및 31초 RED 전환을 로그로
확인했다. 관련 테스트 51개가 통과했다.

---

## 2026-10-15 — 신호기 분리 및 도로 표시 방향 수정

- 앞선 장면의 세로형 신호기가 보행자용처럼 보인다는 문제를 확인했다.
  차량 진입로 우측에 가로형 3등 차량 신호기를, 남쪽 보도에는 세로형 2등 보행자
  신호기를 별도로 설치했다. 신호 노드가 두 램프와 두 ROS 상태 토픽을
  제어하지만, 차량 제어기는 `/safe_drive/vehicle_signal/state`만 읽는다.
- 보행자 신호는 차량 RED의 첫 7초간 WALK, 다음 5초간 DONT_WALK,
  차량 GREEN/YELLOW에는 DONT_WALK로 설정했다. 기존 보행자 actor는
  이 신호를 아직 읽지 않는다.
- 이전 5개 횡단보도 막대가 보행 방향으로 길게 놓인 오류를 수정했다.
  두 월드에서 차량 진행 방향으로 긴 막대 8개를 보행 진행 방향으로
  반복 배치했다. 사거리 동쪽 진입 정지선은 교차 영역 밖으로 옮기고,
  차량 경로의 정지선을 `x=-9.25 m`로 옮겼다. 도로 표시 근거와 좌표는
  `docs/road-layout.md`에 기록했다.

검증: 관련 네 패키지 빌드 및 테스트 51개 통과. 고정 RED 헤드리스 실행에서
차량 중심 `x=-11.95 m`, 앞부분 `x≈-9.85 m`로 정지해 정지선보다 약
`0.60 m` 앞에 있었다. ROS 차량 상태 RED/보행자 상태 WALK가 각각
발행됐고, Gazebo에서 차량 빨간 램프와 보행 WALK 램프가 표시 위치에
있었다. 고정 GREEN 실행에서 차량이 `x=9.50 m, y=-1.75 m`에 도착했고
차량 초록 램프 및 보행 DONT_WALK 램프가 표시 위치로 바뀌었다.
GUI 화면에서 횡단보도 막대 방향과 차량용 3등 신호기의 외형을 확인했다.
초기 전구가 어두운 렌즈 안에 가려지는 문제를 발견해 전구를 렌즈 앞면으로
옮겼다. 기본 추적 카메라도 뒤·위로 이동해 신호기를 화면에 포함했다.
차량용 신호기를 우측 보도 가장자리로 낮춰 3등 외형과 켜진 빨간 등도
확인했다. 최종 고정 RED 실행에서 제어 노드의 구독 토픽은 오도메트리와
`/safe_drive/vehicle_signal/state`이며 보행자 신호 토픽은 포함되지
않았다. 차량은 동일하게 중심 `x=-11.95 m`에 정지했다. 기존
`scenario.launch.py`도 다시 실행해 보행자 `COMPLETED`와 차량 최종
`x=9.50 m, y=-1.75 m` 정지를 확인했다.
