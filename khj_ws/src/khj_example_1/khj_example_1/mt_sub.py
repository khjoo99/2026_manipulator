import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from std_msgs.msg import String


class Message_time_sub(Node):
    def __init__(self):
        super().__init__("mt_sub")  # 노드 이름

        self.create_subscription(
            String,
            "message1",
            self.message_callback,
            10,
        )  # message1 subscriber 등록

        self.create_subscription(
            Header,
            "time",
            self.time_callback,
            10,
        )  # time subscriber 등록

    def message_callback(self, msg):
        self.get_logger().info(msg.data)  # 받은 message1 로깅

    def time_callback(self, msg):
        self.get_logger().info(
            f"time: {msg.stamp.sec}.{msg.stamp.nanosec}"
        )  # 받은 time 로깅


def main(args=None):
    rclpy.init(args=args)

    node = Message_time_sub()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
