# 2026_manipulator

2026-07-20


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

    기본적인 거북이 소환
    ros2 run turtlesim turtlesim_node 

    거북이 이름 바꾸기
    ros2 run turtlesim turtlesim_node  --ros-args --remap __node:=my_turtle

    노드 정보 확인
    ros2 node list
    ros2 node info /turtlesim

    키보드 이용
    ros2 run turtlesim turtle_teleope_key