import math
import random

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from turtlesim.msg import Color, Pose


class Move_turtle(Node):
    def __init__(self):
        super().__init__("move_turtle")

        # 0.1초마다 timer_callback 실행
        self.create_timer(0.1, self.timer_callback)

        # 거북이 이동 명령 Publisher
        self.pub = self.create_publisher(
            Twist,
            "turtle1/cmd_vel",
            10
        )

        # 거북이 위치와 배경색 Subscriber
        self.pose_sub = self.create_subscription(
            Pose,
            "turtle1/pose",
            self.pose_callback,
            10
        )

        self.color_sub = self.create_subscription(
            Color,
            "turtle1/color_sensor",
            self.color_callback,
            10
        )

        self.pose = Pose()
        self.color = Color()

        self.pose_received = False
        self.count = 0

        self.mode_names = [
            "빙글빙글 나선",
            "지그재그",
            "술 취한 거북이",
            "직진 후 제자리 회전",
            "배경색 반응",
        ]

        self.previous_mode = -1

    def timer_callback(self):
        msg = Twist()

        # 0.1초씩 증가하는 시간
        current_time = self.count * 0.1

        # 5초마다 모드 변경
        mode = (self.count // 50) % len(self.mode_names)

        if mode != self.previous_mode:
            self.get_logger().info(
                f"움직임 변경: {self.mode_names[mode]}"
            )
            self.previous_mode = mode

        # 벽에 가까워지면 중앙 방향으로 이동
        if self.pose_received and self.is_near_wall():
            self.move_to_center(msg)

        else:
            if mode == 0:
                self.spiral_move(msg, current_time)

            elif mode == 1:
                self.zigzag_move(msg, current_time)

            elif mode == 2:
                self.drunk_move(msg, current_time)

            elif mode == 3:
                self.dash_and_spin(msg, current_time)

            elif mode == 4:
                self.color_move(msg, current_time)

        self.pub.publish(msg)
        self.count += 1

    def spiral_move(self, msg: Twist, current_time: float):
        """속도와 회전량이 계속 변하는 나선 움직임"""

        msg.linear.x = 1.5 + math.sin(current_time) * 0.8
        msg.angular.z = 1.0 + math.sin(current_time * 2.0) * 0.7

    def zigzag_move(self, msg: Twist, current_time: float):
        """좌우로 빠르게 방향을 바꾸는 지그재그 움직임"""

        msg.linear.x = 2.5

        if int(current_time * 2) % 2 == 0:
            msg.angular.z = 2.5
        else:
            msg.angular.z = -2.5

    def drunk_move(self, msg: Twist, current_time: float):
        """예측하기 어려운 이상한 움직임"""

        msg.linear.x = (
            1.5
            + math.sin(current_time * 3.0) * 0.7
            + random.uniform(-0.3, 0.3)
        )

        msg.angular.z = (
            math.sin(current_time * 5.0) * 2.0
            + math.sin(current_time * 11.0) * 0.8
            + random.uniform(-0.5, 0.5)
        )

    def dash_and_spin(self, msg: Twist, current_time: float):
        """빠르게 달리다가 갑자기 제자리에서 회전"""

        local_time = current_time % 5.0

        if local_time < 3.0:
            msg.linear.x = 3.5
            msg.angular.z = math.sin(current_time * 4.0) * 0.3

        else:
            msg.linear.x = 0.1
            msg.angular.z = 5.0

    def color_move(self, msg: Twist, current_time: float):
        """거북이가 지나가는 바닥색에 따라 움직임 변경"""

        brightness = (
            self.color.r
            + self.color.g
            + self.color.b
        ) / 765.0

        # 밝은 곳에서는 느리게, 어두운 곳에서는 빠르게 이동
        msg.linear.x = 0.5 + (1.0 - brightness) * 3.0

        # 빨간색과 파란색 차이에 따라 회전 방향 변화
        color_difference = (
            float(self.color.r) - float(self.color.b)
        ) / 255.0

        msg.angular.z = (
            color_difference * 3.0
            + math.sin(current_time * 4.0)
        )

    def is_near_wall(self):
        """거북이가 벽 근처에 있는지 확인"""

        margin = 1.2

        return (
            self.pose.x < margin
            or self.pose.x > 11.0 - margin
            or self.pose.y < margin
            or self.pose.y > 11.0 - margin
        )

    def move_to_center(self, msg: Twist):
        """벽에 닿지 않도록 화면 중앙 방향으로 회전"""

        center_x = 5.5
        center_y = 5.5

        target_angle = math.atan2(
            center_y - self.pose.y,
            center_x - self.pose.x
        )

        angle_error = target_angle - self.pose.theta

        # 각도 범위를 -π ~ π로 정리
        angle_error = math.atan2(
            math.sin(angle_error),
            math.cos(angle_error)
        )

        msg.linear.x = 1.5
        msg.angular.z = angle_error * 3.0

    def pose_callback(self, msg: Pose):
        self.pose = msg
        self.pose_received = True

    def color_callback(self, msg: Color):
        self.color = msg


def main(args=None):
    rclpy.init(args=args)

    node = Move_turtle()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info("키보드 인터럽트")

    finally:
        # 종료할 때 거북이 정지
        stop_msg = Twist()
        node.pub.publish(stop_msg)

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()