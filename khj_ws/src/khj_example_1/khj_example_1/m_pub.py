import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Message_pub(Node):
    def __init__(self):
        super().__init__("m_pub")  # 노드 이름

        self.create_timer(1, self.timer_callback)  # timer 등록

        self.pub1 = self.create_publisher(
            String,
            "message1",
            10,
        )  # message1 publisher 등록

        self.pub2 = self.create_publisher(
            String,
            "message2",
            10,
        )  # message2 publisher 등록

    def timer_callback(self):
        msg1 = String()  # DDS에 보낼 객체 초기화
        msg1.data = "message1"
        self.pub1.publish(msg1)

        msg2 = String()  # DDS에 보낼 객체 초기화
        msg2.data = "message2"
        self.pub2.publish(msg2)


def main(args=None):
    rclpy.init(args=args)

    node = Message_pub()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
