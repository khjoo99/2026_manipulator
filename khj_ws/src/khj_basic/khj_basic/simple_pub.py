import rclpy
from rclpy.node import Node


def timer_callback():
    print("첫번째 프로그램")


def main(args=None):
    rclpy.init(args=args)   # rmw 활성화
    node = Node("message_pub")  # 노드 이름
    #  timer 등록
    node.create_timer(1, timer_callback) 
    
    try:
        rclpy.spin(node)  # 블럭 (무한 루프)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt")
    finally:
        node.destroy_node()

    if __name__ == '__main__':
        main()