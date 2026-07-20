# 2026_manipulator

2026-07-20 교재(p.123-200)


1. wsl 설치

2. github아이디 만들고 repository 생성

3. git clone으로 wsl에 복사

4. vscode 설치 및 remote wsl로 접속

5. github 계정 연동

6. ros2 설치 - jazzy

7. turtlesim node 실행

    7-1. xeyes 실행 
    sudo apt install x11-apps
    xeyes

    7-2. turtlesim node 실행

    단위 명시 없을 시 기본적으로 길이는 m, 각도는 rad

    7-3. 기본적인 거북이 소환
    ros2 run turtlesim turtlesim_node 

    7-4. 거북이 이름 바꾸기
    ros2 run turtlesim turtlesim_node  --ros-args --remap __node:=my_turtle

    7-5. 노드 정보 확인
    ros2 node list
    ros2 node info /turtlesim

    7-6. 키보드 이용
    ros2 run turtlesim turtle_teleope_key

    7-7. 토픽 로그 확인
    ros2 topic list
    ros2 topic echo /turtle1/cmd_vel

    로봇이 바라보는 방향이 x축,양수(본인 몸통 기준)

    7-8. 거북이 움직임 직접 지정하기(사람이 직접 강제로 토픽 발행)
    ros2 topic pub --rate 1 /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"

    rqt_gragh 시각화하여 보기

    7-9. rqt를 이용하여 서비스 콜 하기
    piugins -> service -> service caller

    7-10. 액션 발행
    ros2 interface proto turtlesim/action/RotateAbsolute
    ros2 action send_goal /turtle1/rotate_absolute

    7-11. 파라미터 확인
    ros2 node info /turtlesim -> 파라미터 이름은 확인 불가능
    ros2 param list <use_sim_time은 어느 파라미터 리스트를 봐도 다 있음>

    7-12. 배경 색 바꾸기
    ros2 param get /turtlesim background_b(파랑)
    ros2 param get /turtlesim background_r(빨강)
    ros2 param get /turtlesim background_g(초록)

    ros2 param set /turtlesim background_b 69(파랑색 농도 조정)
    ros2 param set /turtlesim background_r 69(빨강색 농도 조정)
    ros2 param set /turtlesim background_g 69(초록색 농도 조정)

    7-13. dump 파일 만들기(파라미터를 환경변수로 설정)
    ros2 param dump /turtlesim > turtlesim.yaml

    7-14. 저장된 파라미터(dump파일) 불러오기
    ros2 run turtlesim turtlesim_node --ros-args--param-file ./turtlesim.yaml
