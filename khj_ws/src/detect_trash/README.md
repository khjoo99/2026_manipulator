# Vision-Based Waste Sorting Manipulator

ROS 2, YOLO, ArUco Homography, OpenManipulator-X를 이용한 재활용 폐기물 자동 분류 프로젝트입니다.

카메라에서 **CAN / PET / PAPER**를 인식하고 Pixel 좌표를 Robot XY 좌표로 변환한 뒤, 가장 가까운 객체를 OpenManipulator-X가 집어 종류별 위치로 이동합니다.

## Features
- ROS 2 Camera Topic
- CAN / PET / PAPER YOLO Detection
- Roboflow Inference
- ArUco ID 1~4 Calibration
- Homography Pixel → Robot XY
- Nearest Target Selection
- 거리별 Pick 좌표 보정
- 종류별 Pick 높이 / 분류 위치
- FollowJointTrajectory Action
- GripperCommand Action
- 공중 안전 이동 Pick & Place
- OpenCV / RQt / TF 확인
- Flask Web Dashboard

## System Flow
```text
Camera
  ↓
aruco_homography_calibration
  ├─ /detect_trash/image_raw → waste_detector
  │                               ↓
  └← /detect_trash/detections ────┘
          ↓
   Homography 변환
          ↓
   Nearest Target
          ↓
/detect_trash/nearest_target
          ↓
   waste_pick_node
      ├─ Arm Action
      └─ Gripper Action
          ↓
   OpenManipulator-X
```

## Package Structure
```text
khj_ws/src/detect_trash/
├── detect_trash/
│   ├── aruco_homography_calibration.py
│   ├── detector_node.py
│   ├── robot_control.py
│   └── dashboard_node.py
├── package.xml
├── setup.cfg
└── setup.py
```

## YOLO Models
| Type | Model ID | Threshold |
|---|---|---:|
| PAPER | `crumpledpaper/1` | 0.70 |
| PET | `plastic-bottles-ip5yb-uziag-hg1ll/1` | 0.85 |
| CAN | `can-a8pgu/2` | 0.75 |

```bash
export ROBOFLOW_API_KEY="YOUR_API_KEY"
```

API Key는 Repository에 Commit하지 않습니다.

## ROS 2 Interfaces

### Topics
| Topic | Type | Description |
|---|---|---|
| `/detect_trash/image_raw` | `sensor_msgs/Image` | Camera Image |
| `/detect_trash/detections` | `std_msgs/String` | YOLO Detection JSON |
| `/detect_trash/target_point` | `geometry_msgs/PointStamped` | Target Position |
| `/detect_trash/nearest_target` | `std_msgs/String` | Nearest Target JSON |
| `/joint_states` | `sensor_msgs/JointState` | Joint State |
| `/tf` | `tf2_msgs/TFMessage` | TF |

### Actions
| Action | Type |
|---|---|
| `/arm_controller/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` |
| `/gripper_controller/gripper_cmd` | `control_msgs/action/GripperCommand` |

## Coordinate Calibration
Robot origin:
```text
link1 = (0, 0)
```

ArUco coordinates:
```text
ID 1 = (0.4,  0.2)
ID 2 = (0.4, -0.2)
ID 3 = (0.0, -0.2)
ID 4 = (0.0,  0.2)
```

가장 가까운 객체는 다음 거리로 결정합니다.
```text
distance = sqrt(x² + y²)
```

## Distance Correction
기준점:
```text
X = 0.30 m
Y = 0.07 m
```

```text
Near → Negative Weight
Reference → 0 %
Far → Positive Weight
```

현재 설정:
```text
NEAR_GAIN  = 3.0
FAR_GAIN   = 1.5
MIN_WEIGHT = -0.30
MAX_WEIGHT =  0.15
```

## Pick & Place Sequence
```text
1. Gripper Open
2. Safe Pose 상승
3. Target 방향 공중 회전
4. Target 상공 이동
5. Pick Z 하강
6. Gripper Close
7. 5 cm Lift
8. Safe Flight Pose
9. Bin 방향 공중 회전
10. Bin 하강
11. Gripper Open
12. Safe Pose 상승
13. 정면 복귀
14. Home 복귀
```

Pick Z:
```text
CAN   = 0.02 m
PET   = 0.02 m
PAPER = -0.07 m
```

Bin Position:
```text
CAN   = (-0.18, -0.13, 0.05)
PET   = (-0.18,  0.00, 0.05)
PAPER = (-0.18,  0.13, 0.05)
```

## Build
```bash
cd ~/2026_manipulator/khj_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select detect_trash --symlink-install
source install/setup.bash
```

## Run

### Terminal 1 - OpenManipulator-X
```bash
source /opt/ros/jazzy/setup.bash
source ~/2026_manipulator/open_manipulator_ws/install/setup.bash
source ~/2026_manipulator/khj_ws/install/setup.bash
ros2 launch open_manipulator_bringup open_manipulator_x.launch.py
```

### Terminal 2 - Camera / ArUco / Homography
```bash
source /opt/ros/jazzy/setup.bash
source ~/2026_manipulator/khj_ws/install/setup.bash
ros2 run detect_trash aruco_homography_calibration
```

### Terminal 3 - YOLO Detector
```bash
export ROBOFLOW_API_KEY="YOUR_API_KEY"
/home/khjoo/2026_manipulator/perception_envs/depth_yolo/.venv/bin/python \
~/2026_manipulator/khj_ws/src/detect_trash/detect_trash/detector_node.py
```

### Terminal 4 - Robot Control
```bash
source /opt/ros/jazzy/setup.bash
source ~/2026_manipulator/khj_ws/install/setup.bash
ros2 run detect_trash robot_control
```

```text
Enter : 가장 가까운 객체 1개 처리
q     : 종료
```

### Terminal 5 - Dashboard
```bash
source /opt/ros/jazzy/setup.bash
source ~/2026_manipulator/khj_ws/install/setup.bash
ros2 run detect_trash dashboard_node
```

Browser:
```text
http://127.0.0.1:5000
```

### RQt
```bash
rqt_graph
```

### TF
```bash
ros2 run tf2_tools view_frames
```

## Error Handling
- Camera Open 실패 → Node 종료
- Homography 미완료 → 좌표 변환 중단
- Detection 없음 → Robot 대기
- JSON 오류 → 메시지 무시
- 작업영역 밖 Target → 제외
- 2초 이상 지난 Target → 무효
- Arm/Gripper Action Server 없음 → Sequence 중단

## Result
- [x] Camera Topic
- [x] YOLO Vision
- [x] CAN / PET / PAPER
- [x] ArUco Homography
- [x] Pixel → Robot XY
- [x] Nearest Target
- [x] Manipulator / Gripper Action
- [x] Safe Flight Pick & Place
- [x] RQt / TF
- [x] Web Dashboard
- [x] Demo Video

## Known Limitations
- 고정 평면 작업공간 가정
- 실험 기반 근사 IK
- 경험적 거리 보정 Gain
- Enter 입력 시 객체 한 개 처리
- 최종 실행 환경에서 전체 TF Tree 연결 상태 추가 검증 필요

## Future Work
- RGB-D 기반 3D 좌표
- 정확한 IK / MoveIt 2
- 자동 연속 Pick & Place
- Pick 성공/실패 Feedback
- Object Tracking
- State Machine Dashboard

## License
Apache-2.0
