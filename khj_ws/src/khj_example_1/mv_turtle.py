import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class Move_tutle(Node):
    def __init__(self):
        super().__init__("move_turtle")  # 노드 이름
        # timer 등록
        self.create_timer(0.1, self.timer_callback)
        self.pub = self.create_publisher(Twist, "turtle1/cmd_vel", 10)
        self.count = 0.0

    def timer_callback(self):
        msg = Twist()  # DDS 에 보낼 객체 초기화
        msg.linear.x = 0.0 + self.count
        msg.angular.z = 1.0
        self.pub.publish(msg)  # DDS 로 보내는 기능 수행
        self.count += 0.01
        if self.count > 3.0:
            self.count = 0.0


def main(args=None):
    rclpy.init(args=args)  # rmw 활성화
    node = Move_tutle()
    try:
        rclpy.spin(node)  # 블럭 (무한 루프)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt")
    finally:
        node.destroy_node()


if __name__ == "__main__":
    main() 