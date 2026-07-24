import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class MoveU2D2(Node):

    VALID_MODES = {"all", "head", "wheel", "arm"}

    def __init__(self):
        super().__init__("move_u2d2")

        self.declare_parameter("mode", "all")
        self.mode = str(self.get_parameter("mode").value).lower()

        if self.mode not in self.VALID_MODES:
            self.get_logger().warning(
                f"알 수 없는 mode '{self.mode}'입니다. all 모드로 실행합니다."
            )
            self.mode = "all"

        self.joint_state_publisher = self.create_publisher(
            JointState,
            "joint_states",
            10,
        )

        self.start_time = self.get_clock().now()

        # 0.02초 = 50 Hz
        self.timer = self.create_timer(
            0.02,
            self.publish_joint_states,
        )

        self.get_logger().info(
            f"move_u2d2 실행: mode={self.mode}"
        )

    def publish_joint_states(self):
        now = self.get_clock().now()

        elapsed = (
            now - self.start_time
        ).nanoseconds / 1_000_000_000.0

        head_angle = 0.0
        wheel_angle = 0.0
        right_elbow_angle = 0.0
        left_elbow_angle = 0.0

        # 머리를 좌우로 회전
        if self.mode in {"all", "head"}:
            head_angle = 0.9 * math.sin(0.8 * elapsed)

        # 네 바퀴를 계속 회전
        if self.mode in {"all", "wheel"}:
            wheel_angle = math.fmod(
                2.5 * elapsed,
                2.0 * math.pi,
            )

        # 양쪽 엘보우를 서로 반대로 굽힘
        if self.mode in {"all", "arm"}:
            right_elbow_angle = (
                0.7
                + 0.65 * math.sin(0.9 * elapsed)
            )

            left_elbow_angle = (
                0.7
                + 0.65 * math.sin(
                    0.9 * elapsed + math.pi
                )
            )

        msg = JointState()
        msg.header.stamp = now.to_msg()

        # URDF의 움직이는 모든 관절 이름
        msg.name = [
            "right_front_wheel_joint",
            "right_back_wheel_joint",
            "left_front_wheel_joint",
            "left_back_wheel_joint",
            "gripper_extension",
            "left_gripper_joint",
            "right_gripper_joint",
            "head_swivel",
            "right_elbow_joint",
            "left_elbow_joint",
        ]

        msg.position = [
            wheel_angle,
            wheel_angle,
            wheel_angle,
            wheel_angle,
            0.0,                  # gripper_extension
            0.0,                  # left_gripper_joint
            0.0,                  # right_gripper_joint
            head_angle,
            right_elbow_angle,
            left_elbow_angle,
        ]

        self.joint_state_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = MoveU2D2()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info("move_u2d2 종료")

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()