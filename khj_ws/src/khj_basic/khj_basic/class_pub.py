import rclpy
from rclpy.node import Node


class M_pub(Node):
    def __init__(self):
        super().__init__("message_pub")  # 노드 이름
        self.create_timer(1, self.timer_callback)  #  timer 등록
        self.count =0


    def timer_callback(self):
        self.get_logger().info(f"첫번째 프로그램 {self.count}")
        self.count += 1

def main(args=None):
    rclpy.init(args=args)   # rmw 활성화
    node = M_pub()  # 노드 이름
    
    try:
        rclpy.spin(node)  # 블럭 (무한 루프)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt")
    finally:
        node.destroy_node()

    if __name__ == '__main__':
        main()