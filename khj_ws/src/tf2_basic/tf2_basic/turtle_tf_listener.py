import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from turtlesim.srv import Spawn


class TurtleFollower(Node):

    def __init__(self):
        super().__init__("turtle_tf_listener")

        # turtle2에게 이동 명령을 전달하는 publisher
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            "turtle2/cmd_vel",
            10,
        )

        # TF 정보를 저장하는 Buffer
        self.tf_buffer = Buffer()

        # /tf, /tf_static 토픽을 구독하는 listener
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self,
        )

        # turtlesim의 spawn 서비스 client 생성
        self.spawn_client = self.create_client(
            Spawn,
            "spawn",
        )

        # turtlesim_node의 spawn 서비스가 실행될 때까지 대기
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                "spawn 서비스를 기다리는 중..."
            )

        # turtle2 생성 요청 데이터
        request = Spawn.Request()

        request.x = 2.0
        request.y = 2.0
        request.theta = 0.0
        request.name = "turtle2"

        # 비동기 서비스 요청
        self.spawn_future = self.spawn_client.call_async(request)

        # 서비스 응답이 오면 실행할 callback
        self.spawn_future.add_done_callback(
            self.spawn_done_callback
        )

        # 1.0초 간격으로 TF 조회 후 turtle2 제어
        self.create_timer(
            1.0,
            self.timer_callback,
        )

    def spawn_done_callback(self, future):
        """
        turtle2 생성 서비스의 결과를 확인한다.
        """

        try:
            response = future.result()

            self.get_logger().info(
                f"{response.name} 생성 완료"
            )

        except Exception as error:
            # turtle2가 이미 생성되어 있는 경우에도
            # 노드를 종료하지 않고 TF 조회는 계속 진행한다.
            self.get_logger().warning(
                f"turtle2 생성 실패: {error}"
            )

    def timer_callback(self):
        """
        1초마다 turtle2 좌표계에서 turtle1의 위치를 조회한다.

        결과에 따라 다음 세 가지 동작 중 하나를 수행한다.

        1. 회전
        2. 정지
        3. 직진
        """

        try:
            transform = self.tf_buffer.lookup_transform(
                "turtle2",   # target frame
                "turtle1",   # source frame
                Time(),      # 가장 최근 TF
            )

        except TransformException as error:
            self.get_logger().warning(
                f"TF lookup 대기 중: {error}"
            )
            return

        # turtle2 기준으로 바라본 turtle1의 위치
        x = transform.transform.translation.x
        y = transform.transform.translation.y

        # turtle1과 turtle2 사이 거리
        distance = math.hypot(x, y)

        # turtle2가 turtle1을 바라보기 위해 돌아야 하는 각도
        angle_error = math.atan2(y, x)

        cmd = Twist()

        # 정지할 거리
        stop_distance = 0.7

        # 이 각도보다 방향 오차가 크면 회전
        angle_tolerance = 0.15

        if distance <= stop_distance:
            # -----------------------
            # 정지
            # -----------------------
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

            state = "정지"

        elif abs(angle_error) > angle_tolerance:
            # -----------------------
            # 제자리 회전
            # -----------------------
            cmd.linear.x = 0.0

            # 1초 동안 angle_error만큼 회전하도록 설정
            # 너무 빠르게 회전하지 않도록 최대 ±2.0으로 제한
            cmd.angular.z = max(
                -2.0,
                min(2.0, angle_error),
            )

            state = "회전"

        else:
            # -----------------------
            # 직진
            # -----------------------
            cmd.linear.x = min(
                2.0,
                distance,
            )

            cmd.angular.z = 0.0

            state = "직진"

        # turtle2 이동 명령 발행
        self.cmd_vel_pub.publish(cmd)

        self.get_logger().info(
            f"{state} | "
            f"거리={distance:.2f}, "
            f"각도오차={angle_error:.2f}"
        )


def main(args=None):
    rclpy.init(args=args)

    node = TurtleFollower()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        print("키보드 인터럽트")

    finally:
        # 프로그램 종료 전에 turtle2 정지
        node.cmd_vel_pub.publish(Twist())
       

        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()