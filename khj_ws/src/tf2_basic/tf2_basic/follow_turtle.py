import math

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros.transform_broadcaster import TransformBroadcaster
from turtlesim.msg import Pose


def yaw_to_quaternion(yaw):
    """
    turtlesim의 theta 값은 yaw 회전값이다.
    yaw를 quaternion으로 변환한다.
    """
    half_yaw = yaw * 0.5

    x = 0.0
    y = 0.0
    z = math.sin(half_yaw)
    w = math.cos(half_yaw)

    return x, y, z, w


class TurtleTFBroadcaster(Node):

    def __init__(self):
        super().__init__("turtle_tf_broadcaster")

        # 동적으로 변하는 TF를 발행하는 broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # turtle1 위치 구독
        self.create_subscription(
            Pose,
            "turtle1/pose",
            self.turtle1_pose_callback,
            10,
        )

        # turtle2 위치 구독
        self.create_subscription(
            Pose,
            "turtle2/pose",
            self.turtle2_pose_callback,
            10,
        )

    def turtle1_pose_callback(self, msg):
        self.publish_turtle_tf(msg, "turtle1")

    def turtle2_pose_callback(self, msg):
        self.publish_turtle_tf(msg, "turtle2")

    def publish_turtle_tf(self, msg, child_frame_id):
        """
        turtlesim의 Pose 메시지를 TF로 변환하여 발행한다.

        turtle1:
            world → turtle1

        turtle2:
            world → turtle2
        """

        t = TransformStamped()

        # TF 메시지 시간
        t.header.stamp = self.get_clock().now().to_msg()

        # 상위 프레임
        t.header.frame_id = "world"

        # 하위 프레임
        t.child_frame_id = child_frame_id

        # turtlesim 위치
        t.transform.translation.x = float(msg.x)
        t.transform.translation.y = float(msg.y)
        t.transform.translation.z = 0.0

        # turtlesim 방향 theta를 quaternion으로 변환
        x, y, z, w = yaw_to_quaternion(msg.theta)

        t.transform.rotation.x = x
        t.transform.rotation.y = y
        t.transform.rotation.z = z
        t.transform.rotation.w = w

        # /tf 토픽으로 발행
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)

    node = TurtleTFBroadcaster()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        print("키보드 인터럽트")

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()