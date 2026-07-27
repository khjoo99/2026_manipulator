import rclpy
from control_msgs.action import GripperCommand
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class Manipulator_pub(Node):
    def __init__(self):
        super().__init__("manipulator_pub")  # 노드 이름
        # timer 등록
        self.create_timer(0.5, self.timer_callback)
        self.pub = self.create_publisher(JointTrajectory, "arm_controller/joint_trajectory", 10)
        self.gripper_client = ActionClient(self, GripperCommand, "/gripper_controller/gripper_cmd")
        self.joint_state_subscription = self.create_subscription(
            JointState, "joint_states", self.joint_callback, 10
        )
        self.current_joint_position = [0.0, 0.0, 0.0, 0.0]
        self.current_gripper_position = 0.0
        self.joint_state_received = False
        self.count = True
        self.duration_sec = 1

    def timer_callback(self):
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "move_manipulator"
        msg.joint_names = ["joint1", "joint2", "joint3", "joint4"]
        point = JointTrajectoryPoint()
        if self.count:
            point.positions = [
                0.8995922516973869,
                -0.5832234352774157,
                -0.26300971181849175,
                -0.6258641614575486,
            ]
            self.count = False
        else:
            point.positions = [
                0.4795922516973865,
                -0.883223435277416,
                0.3969902881815083,
                0.8941358385424522,
            ]
            self.count = True
        seconds = int(self.duration_sec)
        nanoseconds = int((self.duration_sec - seconds) * 1_000_000_000)

        point.time_from_start.sec = seconds
        point.time_from_start.nanosec = nanoseconds

        msg.points.append(point)  # type: ignore
        self.pub.publish(msg)

    def joint_callback(self, msg: JointState):
        self.current_joint_position = msg.position


def main(args=None):
    rclpy.init(args=args)  # rmw 활성화
    node = Manipulator_pub()
    try:
        rclpy.spin(node)  # 블럭 (무한 루프)
    except KeyboardInterrupt:
        print("키보드 인터럽트")
    finally:
        node.destroy_node()


if __name__ == "__main__":
    main()