import rclpy
from rclpy.node import Node
from std_msgs.msg import Header


class Header_pub(Node):

    def __init__(self):
        super().__init__('header_pub')  # 노드 이름

        # 0.1초마다 콜백 실행 = 10 Hz
        self.create_timer(0.1, self.timer_callback)

        # Header 메시지를 /time 토픽으로 발행
        self.pub = self.create_publisher(Header, 'time', 10)

    def timer_callback(self):
        msg = Header()

        msg.frame_id = 'time test'
        msg.stamp = self.get_clock().now().to_msg()

        self.pub.publish(msg)

        self.get_logger().info(
            f'frame_id: {msg.frame_id}, '
            f'time: {msg.stamp.sec}.{msg.stamp.nanosec}'
        )


def main(args=None):
    rclpy.init(args=args)

    node = Header_pub()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt')

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()