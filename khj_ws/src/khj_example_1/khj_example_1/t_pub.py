import rclpy
from rclpy.node import Node
from std_msgs.msg import Header


class Time_pub(Node):
    def __init__(self):
        super().__init__("t_pub")  # 노드 이름

        self.create_timer(1 / 10, self.timer_callback)  # timer 등록
        self.pub = self.create_publisher(
            Header,
            "time",
            10,
        )  # publisher 등록

    def timer_callback(self):
        msg = Header()  # DDS에 보낼 객체 초기화
        msg.stamp = self.get_clock().now().to_msg()
        msg.frame_id = "time"

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = Time_pub()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
