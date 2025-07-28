# Doosan Robotics Boot Camp(2025.01.06 ~ 2025.07.06)
## 3. ROKEY B-3조 협동-3 Project (디지털 트윈 기반 서비스 로봇 운영 시스템 구성) DigitalTwin_Real
&nbsp;
## 🧠 차선 주행 및 아르코마커 인식

&nbsp;

## 🔗 참고자료
https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/

&nbsp;

## 📑 목차

1. [📌 프로젝트 개요](#1--프로젝트-개요)
2. [🔧 구성 요소](#2--구성-요소)  
3. [💻 사용 기술](#3--사용-기술)  
4. [🧭 동작 흐름 요약](#4--동작-흐름-요약)
5. [🧪 Simul vs Real](#5--Simul-vs-Real)  
6. [💻 코드 실행 방법](#6--코드-실행-방법)  
7. [📷 시연 영상/이미지](#7--시연-영상--이미지)  
8. [🌟 기대 효과/ 한계점 및 개선점](#8--기대-효과)  

   
&nbsp;
## 1. 📌 프로젝트 개요

Rokey 휴게소 Autodrive 시스템은 시뮬레이션을 통해 다양한 도로 상황에 대응하는 판단 및 제어 로직을 구현한 후, **이를 실제 로봇 플랫폼에서 실환경에 맞춰 적용하는 실증 실험**을 진행하였습니다


&nbsp;
### 🎯 기획 의도

시뮬레이션 단계에서는 ROS2 기반의 가상 환경에서 Vision 및 센서 데이터를 활용한 자율주행이 구현되었으며, 실제 환경에서는 **TurtleBot3 Waffle Pi** 로봇과 **매니퓰레이터(로봇 팔)**, **Webcam**을 기반으로 시스템을 확장하여 적용하였습니다.
  
&nbsp;
### 🏭 기존 기술의 활용과 협동로봇의 확장 가능성
본 프로젝트는 **시뮬레이션에서 실제 환경으로의 성공적인 이행**을 통해  ROS2 기반 자율주행 로직의 신뢰성과 유연성을 입증하였으며, 차선 인식, 마커 인식, 센서 제어 기반 주행 로직의 **산업적 적용 가능성**을 실증했습니다.

특히, **매니퓰레이터 기반 장애물 대응** 및 **픽업-플레이스 동작**은  일반적인 주행 로봇 프로젝트에서 보기 어려운 고급 기능으로,  향후 협동로봇 및 자율물류 시스템으로의 확장 기반을 마련하였습니다.

실증을 통해 구현된 시스템은 다음과 같은 실제 분야로 확장될 수 있습니다:

- **고속도로 휴게소 자동 정차 및 주차 보조 시스템**
- **물류 센터 내 자율 피킹 및 경로 판단 시스템**
- **스마트 시티 기반 정밀 주행 로봇 플랫폼**
- **산업용 협동 로봇 기반 정적/동적 판단 시뮬레이터**

&nbsp;

## 2. 🔧 구성 요소

| 구성 요소 | 설명 |
|-----------|------|
| 🛣 **실제 맵** | 차선(lane), 장애물(obstacle) 등이 포함된 실제 주행 환경 |
| 🤖 **TurtleBot3 Waffle Pi** | ROS2 기반 자율주행 모바일 로봇 |
| 🧠 **OpenCR** | 로봇 모터 제어 및 센서 통신을 담당하는 마이크로 컨트롤러 보드 |
| 🖥 **Jetson Nano** | AI 모델 실행, 영상 처리 및 ROS2 연산을 수행하는 온보드 컴퓨팅 모듈 |
| 🦾 **OpenMANIPULATOR-X** | ROS 기반 제어가 가능한 정밀 4-DOF 매니퓰레이터 |
| 📷 **Logitech C270 HD Webcam** | `/camera/image_raw`, `/camera/image_compressed`를 통한 실시간 영상 수신 |

### TurtleBot3 Waffle Pi + OpenMANIPULATOR
<img width="400" height="400" alt="image" src="https://github.com/user-attachments/assets/540d5a0d-89de-4f26-9a52-c09338c0ae17" />

### OpenCR
<img width="529" height="349" alt="image" src="https://github.com/user-attachments/assets/23182c42-848e-47ca-bfe3-3b10077d6aa1" />

### Jetson Nano
<img width="393" height="264" alt="image" src="https://github.com/user-attachments/assets/893993ec-65ea-4bd6-8be2-4cfb78361916" />

### Logitech C270 HD Webcam
<img width="221" height="206" alt="image" src="https://github.com/user-attachments/assets/b8e6887b-6c0b-4af7-9417-9bf86340c758" />


&nbsp;
## 3. 💻 사용 기술

| 기술 | 내용 |
|------|------|
| 🖥 **OS 및 플랫폼** | Ubuntu 22.04 + ROS2 Humble (Jetson Nano 기반) |
| 🤖 **자율주행 로봇** | 실제 TurtleBot3 Waffle Pi + Jetson Nano + OpenCR 기반 주행 |
| 📷 **카메라 기반 차선 인식** | USB Camera를 통한 실시간 이미지 입력 처리 |
| ⚙️ **Lane Following 알고리즘** | ROI 설정 → grayscale 변환 → Canny Edge → Hough Line Transform으로 차선 검출 |
| 📐 **중심선 추출 및 PID 제어** | 검출된 차선 정보를 기반으로 차량 중심선 계산 후 선형 및 각속도 결정 |


&nbsp;


## 4. 🧭 동작 흐름 요약
## Lane_Follower

### 1. ⚙️ 파라미터 선언 및 동적 로딩

- 여러 제어·검출 파라미터(Hue/Sat/Val 범위, 픽셀 임계치, 복구 대기시간·회전시간, PID 게인, 속도·가속도 한계 등)를 딕셔너리 `params`에 모아 선언.
- `declare_parameter(k, v)` → `self.get_parameter(k).value`를 `setattr(self, k, ...)`으로 클래스 속성에 할당.
- 런치 파일 또는 `ros2 param set`으로 외부 값 변경 시 코드 내에서 `self.<파라미터명>`으로 실시간 반영 가능.


&nbsp;


### 2. 🧠 초기화 및 상태 변수 설정

| 변수 | 설명 |
|------|------|
| `CvBridge()` | ROS 이미지 메시지 → OpenCV 변환 |
| `self.frame` | 최신 이미지 프레임 저장 |
| `self.joints`, `self.joint_names` | 팔 관절 상태 저장 (4-DOF) |
| `self.offset`, `self.state`, `self.recover` | 오프셋 보정, FSM 상태 관리 |
| `self.prev_err`, `self.integral` | PID 오차 및 적분값 |
| `self.pick_in_progress`, `self.marker_detected` | ArUco 픽업 작업 플래그 |

&nbsp;

### 3. 🔗 구독 & 퍼블리셔 구성

| 항목 | 설명 |
|------|------|
| `/camera/image_raw/compressed` | 영상 구독 (`cb_image()`) |
| `/joint_states` | 암 상태 구독 (`cb_joint()`) |
| `/cmd_vel` | 주행 제어 퍼블리시 |
| `/arm_controller/joint_trajectory` | 암 제어 |
| `/pick_trigger` | 픽업 트리거 발행 |
| `/detected_markers` | ArUco 마커 구독 (`marker_callback()`) |
| 디버그 퍼블리셔 | 마스크, 추적 영상 출력 |


&nbsp;


### 4. ⏱️ 타이머 및 콜백 구조

- `self.create_timer(0.05, self.loop)` → 20Hz 주기로 제어 루프 실행
- `MutuallyExclusiveCallbackGroup()`으로 콜백 충돌 방지
- `self.delayed_stop_timer` 등 상태 변수 초기화


&nbsp;


### 5. 🧭 PID 제어 및 주행 구조

- PID 게인 (`kp`, `ki`, `kd`)과 제한값 (`max_a`, `min_v`, `max_v`) 설정
- `self.prev_err`, `self.integral` → 좌우 차선 중심 기반 조향 제어


&nbsp;


### 6. 📷 콜백 함수 구성

#### `cb_image(msg)`
- `/camera/image_raw/compressed` 영상 수신 → `CvBridge` 변환 후 `self.frame` 저장
- 예외 발생 시 무시하고 패스

#### `cb_joint(msg)`
- `JointState` 메시지에서 `self.joint_names`에 해당하는 조인트 이름의 위치(position)값을 `self.joints`에 업데이트

#### `marker_callback(msg)`
- ArUco `MarkerArray`에서 ID 1을 감지하면 `self.marker_detected = True` 설정
- 감지 시 `/pick_trigger` 발행, 정지 명령 `/cmd_vel` 발행, 로그 출력

#### `cmd(lin, ang)`
- `Twist` 메시지를 생성해 `/cmd_vel`로 퍼블리시
- 헬퍼 함수로 선속도 및 각속도 설정

#### `delayed_stop()`
- 빈 `Twist()` 퍼블리시 → 즉시 정지
- 타이머 정리 및 중복 정지 방지


&nbsp;


### 7. 🧠 Main 이미지 처리 루프: `image_callback()`

- **픽업 중이면 중단:** `self.pick_in_progress == True` → return
- **마커 감지 시:**
  - 정지 명령 (`Twist(0, 0)`)
  - `/pick_trigger` 퍼블리시
  - 상태 리셋
- **차선 추적 로직:**
  - HSV 변환 후 노란/흰색 마스크 생성
  - `cv2.countNonZero()` → 픽셀 수 집계
  - `only_y`, `only_w`, `both` 상태 구분


&nbsp;


### 8. 🔁 복구 로직 흐름

- **조건:** only_y 또는 only_w (노란/흰 선 하나만 감지될 경우)
- **동작:**
  - `move_joint()`로 관절 오프셋 적용
  - FSM 상태 `recovering`, 회전 방향·시작 시각 기록
  - 일정 시간 정지 후 → 회전 주행
  - `both` 감지되거나 시간 초과 시 → 관절 복귀, `restored` 상태로 전환


&nbsp;


### 9. 🎯 PID 보정 + 조인트 보정

- 기본 조향각 = PID 오차 기반 (`pid_ang`)
- 관절 보정 = `self.joints[0]` (joint1 회전 오프셋)
- 최종 조향각 `ang = pid_ang + self.joints[0]`
  - → 조향 보정 + 카메라 각도 조정 효과 동시 적용
  

&nbsp;

## Aruco_Marker
<img width="249" height="317" alt="image" src="https://github.com/user-attachments/assets/21a8fdd5-53c2-4bac-a920-fca9422b3c2d" />

&nbsp;

### 📸 Camera
- 영상 캡처 후 `/camera/image_raw/compressed` 토픽으로 퍼블리시

&nbsp;

### 🎯 Aruco_detect
- `/camera/image_raw/compressed` 구독  
- ArUco 마커 검출  
- 검출된 결과를 `/detected_markers`로 퍼블리시

&nbsp;

### 🛣️ Lane_detect
- `/detected_markers` 구독  
- 마커 감지 시 기존 `lane_detect` 정지  
- `/cmd_vel`로 정지 명령 퍼블리시

&nbsp;

### 🦾 Turtlebot_arm_controller
- 로봇 팔 제어를 위한 서비스 제공

&nbsp;

### 🖐️ Pick_and_place
- `/cmd_vel` 등 특정 조건 트리거 시  
- `moveit_control` 서비스 호출  
- 요청에 따라 로봇 암 및 그리퍼 동작 실행


&nbsp;

## 5. 🧪 Simul vs Real
## 🔍 Real-World vs Simulation: 기술 차이 비교

| 항목 | Real-World (실제 주행) | Simulation (시뮬레이션) |
|------|-------------------------|---------------------------|
| 📥 입력 토픽 | 카메라 영상 `/cmd_vel (Twist)` | Gazebo 카메라 `/lane/cmd_vel (Twist)` |
| ⚙️ 파라미터화 | HSV, PID 등 하드코딩 | 외부 파라미터로 동적 조정 (`ros2 param set`) |
| 🎨 HSV 처리 | 코드 내 고정값 `cv2.inRange(...)` | 런타임 조정 가능한 파라미터화된 inRange |
| 🛣 차선 피팅 | 하단 히스토그램 → 중심 계산 → PID 제어 | 2차 다항식 피팅 + 슬라이딩 윈도우 + 이동 평균 |
| 🧠 구조 | 단일 노드 내 차선 감지 + 제어 통합 | `DetectLane`, `ControlLane` 노드 분리 |
| 🎯 제어 알고리즘 | PID 제어 + ArUco 마커 감지 후 정지 제어 | PID 제어 + 신뢰도 기반 판단 및 장애물 회피 |
| 📤 출력(퍼블리시) | `/cmd_vel`, ArUco 마커 감지, pick 트리거 | `/detect/lane`, `/lane_state`, 예측 결과 포함 |
| 🧩 추가 기능 | ArUco 마커 감지 후 픽업 수행 | 신호등, 표지판 상태 인식, 픽업/회피 전략 포함 |



&nbsp;

## ⚙️ 파라미터 처리 방식

| 항목 | 실제 구현 | 시뮬레이션 구현 |
|------|------------|-----------------|
| 파라미터 선언 | HSV, PID 범위 하드코딩 | `declare_parameter()`로 외부에서 동적 제어 가능 |
| HSV 마스크 | `(15, 40, 100) ~ (70, 255, 255)` | 런치 파일 또는 ROS2 명령어로 실시간 변경 |
| 클래스 속성 할당 | `self.get_parameter(k).value` → `setattr(self, k, ...)` | 동일 |
| 자동화 모드 | - | `reconfigure` 통해 자동 보정 기능도 지원 |


&nbsp;

## 📊 차선 검출 알고리즘

| 항목 | 실제 구현 | 시뮬레이션 구현 |
|------|------------|-----------------|
| 입력 처리 | 영상 하단 픽셀 분포 → 히스토그램 → 중심선 | 전체 화면에서 슬라이딩 윈도우 기반 추적 |
| 검출 방식 | 이진 마스크 후 `np.argmax(hist)` | `cv2.fitLine()`으로 중심선 예측 |
| 중심 계산 | 히스토그램 분포의 절반 이상 영역 | 다수 프레임 평균으로 노이즈 제거 |
| 이동 평균 | - | 적용됨 |

&nbsp;

## 🛠 ROS2 기반 시스템 구성

| 항목 | Real Driving Node | Simulation Control Node |
|------|-------------------|--------------------------|
| 입력 토픽 | `/camera/image_raw`, `/joint_states`, `/detected_markers` | Gazebo 카메라, 라이다, 기타 센서 |
| 제어 알고리즘 | PID 제어 + ArUco 기반 판단 | FSM 제어, 센서 통합 판단 |
| 퍼블리시 토픽 | `/cmd_vel`, `/pick_trigger`, `/debug_image` | `/detect/lane`, `/lane_state`, `/predict_result` 등 |
| 특징 | 노이즈 많고 정지 정확도 필요 | 이상적인 센서 환경, 실시간 상태 시각화 |


&nbsp;

## 6. 💻 코드 실행 방법

### 🔌 Hardware Bringup
- 코드: [`hardware.launch.py`](./rokeypj_ws/src/turtlebot3_manipulation/turtlebot3_manipulation_bringup/launch/hardware.launch.py)

```bash
ros2 launch turtlebot3_manipulation_bringup hardware.launch.py
```

&nbsp;

### 🤖 MoveIt Core
- 코드: [`moveit_core.launch.py`](./rokeypj_ws/src/turtlebot3_manipulation/turtlebot3_manipulation_moveit_config/launch/moveit_core.launch.py)

```bash
ros2 launch turtlebot3_manipulation_moveit_config moveit_core.launch.py
```

&nbsp;

### 📸 Camera Publisher (Aruco + YOLO)
- 코드: [`camera_pub.py`](./rokeypj_ws/src/aruco_yolo/aruco_yolo/camera_pub)
```bash
ros2 run aruco_yolo camera_pub
```

&nbsp;
### ⚙️ Servo 
- 코드: [`servo.launch.py`](./rokeypj_ws/src/turtlebot3_manipulation/turtlebot3_manipulation_moveit_config/launch/servo.launch.py)
```bash
ros2 launch turtlebot3_manipulation_moveit_config servo.launch.py
```

&nbsp;
### 🦾 Arm Controller
- 코드: [`turtlebot_arm_controller.cpp`](./rokeypj_ws/src/turtlebot_moveit/turtlebot_moveit/src/turtlebot_arm_controller.cpp)
```bash
ros2 run turtlebot_moveit turtlebot_arm_controller
```

&nbsp;
### 🛣️ Lane Detection
- 코드: [`lane_detect.py`](./rokeypj_ws/src/lane_detector/lane_detector/lane_detect.py)
```bash
ros2 run lane_detector lane_detect
```

&nbsp;

### 🖐️ Pick and Place
- 코드: [`pick_and_place.py`](./rokeypj_ws/src/aruco_yolo/aruco_yolo/pick_and_place.py)
```bash
ros2 run aruco_yolo pick_and_place
```

&nbsp;

## 7. 📷 시연 영상 / 이미지
![3전체사진](https://github.com/user-attachments/assets/aabfcc64-3cc8-414a-bd93-be11d7ca29c1)

&nbsp;

> https://youtu.be/hl8J-E7p_yg

&nbsp;
## 8. 🌟 기대 효과
### 📌 프로젝트 기대 효과

| 구분             | 효과                        | 설명                                             |
|------------------|-----------------------------|--------------------------------------------------|
| 기술 실현        | 라스트마일 자율주행         | 공공/상업 주차장, 물류창고 등에 적용 가능        |
| 사용자 편의성 향상 | 주차 스트레스 감소          | 주차 공간 탐색/충돌 위험 감소                    |
| 🅿️ 공간 효율성   | 주차 최적화                  | 경차/전기차 등 분류 주차로 공간 활용 극대화      |
| 사회적 포용성    | 교통 약자 고려              | 장애인·고령자 차량 자동 인식 및 근접 배치        |
| 지속가능성       | 전기차 충전 구역 자동 배정  | 탄소중립 도시교통 인프라와 연결 가능             |

&nbsp;

### 📌 사업화 가능성

| 구분 | 사업 모델                 | 주요 고객                      | 수익 구조                         | 핵심 가치                    |
|------|--------------------------|-------------------------------|----------------------------------|-----------------------------|
| B2G  | 공공 주차장 자동화 시스템 | 지자체, 공공기관               | 시스템 납품 + 유지보수 계약       | 스마트시티 인프라 연계       |
| B2B  | 대형 시설 자율주차 솔루션 | 쇼핑몰, 물류센터, 아파트 단지 | 솔루션 판매 + 구독형 유지비       | 운영 효율화 + 고객 경험 개선 |
| B2C  | 개인용 스마트 주차 로봇   | 고급 EV 사용자, 스마트홈 고객 | 로봇 판매 + 앱 서비스 구독        | 개인화된 편의성과 자동화     |

&nbsp;

### 느낀 점 및 경험한 성과
- **장점**:
  - 안정적인 중앙유지 기반의 주행 및 곡선에서도 안정된 차선 인식을 위한 능동적인 움직임이 있어, 여러 환경에서 보다 안정적인 주행 가능
  - 어떤 조건에서도 시도하지 않았던 매니퓰레이터 이동으로 차선 감지 확보
    
- **추후 개선점 / 보완할 점**:
  - 매니퓰레이터가 제한되어있을 때는 사용 & 시뮬레이션 코드에서 발전시켜야 함
  - 자율주행의 기술이 뒷받침된 로직을 바탕으로 구현됨을 알게 되었으며 시뮬레이션과 실제 환경의 차이를 좁히기 위해 보완이 필요

- 자율주행 기술이 뒷받침된 로직을 기반으로 구현되었다는 것을 시뮬레이션과 실제 환경 비교를 통해 체감
- 좁은 환경, 비정형 상황 대응을 위한 보완 필요성 인지

&nbsp;

## 팀원
정서윤 나승원 이동기 홍진규

&nbsp;

## +) Advanced
lane_detect_advanced
