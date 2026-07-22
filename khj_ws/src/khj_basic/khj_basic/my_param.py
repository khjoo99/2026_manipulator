import rclpy
from rclpy.node import Node


class TParam(Node):
    def __init__(self):
        super().__init__("tparam")  # 노드 이름
        self.declare_parameter("my_param", "내가 만든 클래스 노드 안의 파라미터")
        self.my_param = self.get_parameter("my_param").get_parameter_value().string_value
        self.create_timer(1, self.timer_callback)  #  timer 등록


    def timer_callback(self):
        self.get_logger().info(self.my_param)
       

def main(args=None):
    rclpy.init(args=args)   # rmw 활성화
    node = TParam()  # 노드 이름
    
    try:
        rclpy.spin(node)  # 블럭 (무한 루프)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt")
    finally:
        node.destroy_node()

    if __name__ == '__main__':
        main()