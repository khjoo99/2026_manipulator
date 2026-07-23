import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from std_msgs.msg import String
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


class M_pub(Node):
    def __init__(self):
        super().__init__("massage_pub")  # 노드 이름
        # timer 등록
        transformation = [1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        self.make_transforms(transformation)

    def make_transforms(self, transformation):
        # tf 데이터 저장
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "world"  # 중요!!(상위 tf2 명시)
        t.child_frame_id = "map"
        t.transform.translation.x = transformation[0]
        t.transform.translation.y = transformation[1]
        t.transform.translation.z = transformation[2]
        t.transform.rotation.x = transformation[3]
        t.transform.rotation.y = transformation[4]
        t.transform.rotation.z = transformation[5]
        t.transform.rotation.w = transformation[6]
        # topic /tf 에 발행
        self.tf_static_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)  # rmw 활성화
    node = M_pub()
    try:
        rclpy.spin(node)  # 블럭 (무한 루프)
    except KeyboardInterrupt:
        print("키보드 인터럽트")
    finally:
        node.destroy_node()


if __name__ == "__main__":
    main()