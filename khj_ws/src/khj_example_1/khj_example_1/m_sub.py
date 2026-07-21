import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Message_sub(Node):
    def __init__(self):
        super().__init__("m_sub")  # 노드 이름

        self.create_subscription(
            String,
            "message1",
            self.sub_callback,
            10,
        )  # subscriber 등록

    def sub_callback(self, msg):
        self.get_logger().info(msg.data)  # 받은 message1 로깅


def main(args=None):
    rclpy.init(args=args)

    node = Message_sub()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
