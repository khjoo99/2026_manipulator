import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import QoSDurabilityPolicy, QoSReliabilityPolicy, QosProfile, qos_profile_default
from std_msgs.msg import String


class Qos_M_pub(Node):
    def __init__(self):
        super().__init__("message_pub")  # 노드 이름
        #  timer 등록
        self.create_timer(1, self.timer_callback)  
        self.qos_profile = QoSProfile(
            history = QosHistoryPolicy.KEEP_ALL,
            reliability = QoSReliabilityPolicy.RELIABLE
            durability = QoSDurabilityPolicy.TRANSIENT_LOCAL)  # QoS 설정
        self.pub = self.create_publisher(String, "message", self.qos_profile)  # publisher 등록
        #   self.pub = self.create_publisher(String, "message", self.qos_profile_default)  # publisher 등록
        self.count = 0


    def timer_callback(self):
        msg = String()
        msg.data = f"첫번째 프로그램 {self.count}"
        self.get_logger().info(msg.data)
        self.pub.publish(msg)   #DDS로 보내는 역할을 수행
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