import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from user_int_msg.msg import UserInt


class M_pub(Node):
    def __init__(self):
        super().__init__("m_pub")  # 노드 이름
        self.create_timer(1, self.timer_callback)  #  timer 등록
        self.pub = self.create_publisher(UserInt, "message", 10)  # publisher 등록


    def timer_callback(self):
        msg = UserInt()
        msg.user_int = 12
        msg.user_int2 = 23
        msg.user_int3 = 32
        self.pub.publish(msg)

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