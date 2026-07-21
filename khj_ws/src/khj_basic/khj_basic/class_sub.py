import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class M_sub(Node):
    def __init__(self):
        super().__init__("message_sub")  # 노드 이름
        # subscrition callback 등록
        self.create_subscription(String, "message", self.sub_callback, 10)
        self.count = 0
        
    def sub_callback(self, msg: String):
        self.get_logger().info(msg.data) 


def main(args=None):
    rclpy.init(args=args)   # rmw 활성화
    node = M_sub()  # 노드 이름
    
    try:
        rclpy.spin(node)  # 블럭 (무한 루프)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt")
    finally:
        node.destroy_node()

    if __name__ == '__main__':
        main()