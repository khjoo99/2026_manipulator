import random
import time
from pathlib import Path
from typing import Any

import yaml
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from control_msgs.action import FollowJointTrajectory, GripperCommand
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint


class DanceManipulatorAction(Node):
    
    # YAML 자세를 무작위로 선택해 액션으로 실행하는 로봇팔 노드
    def __init__(self) -> None:
        super().__init__("dance_manipulator_action")

        self.declare_parameter("data_file", "dance_positions.yaml")
        self.declare_parameter("seed", -1)
        self.declare_parameter("startup_delay", 2.0)
        self.declare_parameter("use_gripper", True)

        data_file = str(self.get_parameter("data_file").value)
        seed = int(self.get_parameter("seed").value)
        startup_delay = float(self.get_parameter("startup_delay").value)
        self.use_gripper = bool(self.get_parameter("use_gripper").value)

        self.random = random.Random(None if seed < 0 else seed)

        # 기존 /arm_controller/joint_trajectory 토픽 publisher 대신
        # FollowJointTrajectory 액션 클라이언트를 사용한다.
        self.arm_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/arm_controller/follow_joint_trajectory",
        )
        self.gripper_client = ActionClient(
            self,
            GripperCommand,
            "/gripper_controller/gripper_cmd",
        )

        self.data_path = self.resolve_data_path(data_file)
        self.joint_names, self.poses, self.start_pose_name = self.load_dance_data(
            self.data_path
        )

        self.pose_bag: list[int] = []
        self.last_pose_index: int | None = None
        self.start_pose_sent = False

        self.current_pose: dict[str, Any] | None = None
        self.current_pose_index: int | None = None
        self.arm_goal_active = False

        self.waiting_for_arm_logged = False
        self.waiting_for_gripper_logged = False
        self.next_motion_time = time.monotonic() + max(0.0, startup_delay)
        self.last_feedback_log_time = 0.0

        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info(f"춤 데이터 로드 완료: {self.data_path}")
        self.get_logger().info(
            f"관절={self.joint_names}, 자세={len(self.poses)}개, random seed={seed}"
        )

    def resolve_data_path(self, data_file: str) -> Path:
        requested_path = Path(data_file).expanduser()
        if requested_path.is_absolute():
            return requested_path

        package_share = Path(get_package_share_directory("tf2_basic"))
        return package_share / "data" / requested_path

    def load_dance_data(
        self,
        data_path: Path,
    ) -> tuple[list[str], list[dict[str, Any]], str]:
        if not data_path.is_file():
            raise FileNotFoundError(f"춤 데이터 파일을 찾을 수 없습니다: {data_path}")

        with data_path.open("r", encoding="utf-8") as file:
            loaded = yaml.safe_load(file)

        if not isinstance(loaded, dict):
            raise ValueError("YAML 최상위 데이터는 딕셔너리여야 합니다.")

        joint_names = loaded.get("joint_names")
        poses = loaded.get("poses")
        start_pose_name = str(loaded.get("start_pose", ""))

        if not isinstance(joint_names, list) or not joint_names:
            raise ValueError("joint_names는 비어 있지 않은 리스트여야 합니다.")
        joint_names = [str(name) for name in joint_names]

        if not isinstance(poses, list) or not poses:
            raise ValueError("poses는 비어 있지 않은 리스트여야 합니다.")

        validated_poses: list[dict[str, Any]] = []
        for index, pose in enumerate(poses):
            if not isinstance(pose, dict):
                raise ValueError(f"poses[{index}]는 딕셔너리여야 합니다.")

            name = str(pose.get("name", f"pose_{index}"))
            positions = pose.get("positions")
            if not isinstance(positions, list):
                raise ValueError(f"{name}: positions는 리스트여야 합니다.")
            if len(positions) != len(joint_names):
                raise ValueError(
                    f"{name}: positions 개수({len(positions)})와 "
                    f"joint_names 개수({len(joint_names)})가 다릅니다."
                )

            duration = float(pose.get("duration", 2.0))
            pause = float(pose.get("pause", 0.2))
            if duration <= 0.0:
                raise ValueError(f"{name}: duration은 0보다 커야 합니다.")

            validated_pose: dict[str, Any] = {
                "name": name,
                "positions": [float(value) for value in positions],
                "duration": duration,
                "pause": max(0.0, pause),
            }
            if "gripper" in pose:
                validated_pose["gripper"] = float(pose["gripper"])

            validated_poses.append(validated_pose)

        if start_pose_name and not any(
            pose["name"] == start_pose_name for pose in validated_poses
        ):
            raise ValueError(f"start_pose '{start_pose_name}'가 poses 안에 없습니다.")

        return joint_names, validated_poses, start_pose_name

    def timer_callback(self) -> None:
        # 이전 액션의 Result가 돌아오기 전에는 새 Goal을 보내지 않는다.
        if self.arm_goal_active or time.monotonic() < self.next_motion_time:
            return

        if not self.arm_client.server_is_ready():
            if not self.waiting_for_arm_logged:
                self.get_logger().warning(
                    "/arm_controller/follow_joint_trajectory 액션 서버를 기다리는 중입니다. "
                    "OpenMANIPULATOR bringup을 먼저 실행하세요."
                )
                self.waiting_for_arm_logged = True
            self.next_motion_time = time.monotonic() + 1.0
            return

        self.waiting_for_arm_logged = False
        pose_index = self.select_next_pose_index()
        self.current_pose_index = pose_index
        self.current_pose = self.poses[pose_index]
        self.send_arm_goal(self.current_pose)

    def select_next_pose_index(self) -> int:
        if not self.start_pose_sent and self.start_pose_name:
            self.start_pose_sent = True
            for index, pose in enumerate(self.poses):
                if pose["name"] == self.start_pose_name:
                    return index

        if not self.pose_bag:
            self.pose_bag = list(range(len(self.poses)))
            self.random.shuffle(self.pose_bag)

            if (
                len(self.pose_bag) > 1
                and self.last_pose_index is not None
                and self.pose_bag[0] == self.last_pose_index
            ):
                self.pose_bag[0], self.pose_bag[1] = (
                    self.pose_bag[1],
                    self.pose_bag[0],
                )

        return self.pose_bag.pop(0)

    def send_arm_goal(self, pose: dict[str, Any]) -> None:
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = list(pose["positions"])

        duration = float(pose["duration"])
        seconds = int(duration)
        point.time_from_start.sec = seconds
        point.time_from_start.nanosec = int(
            (duration - seconds) * 1_000_000_000
        )

        goal.trajectory.points.append(point)
        self.arm_goal_active = True

        self.get_logger().info(
            f"팔 Goal 전송: {pose['name']} | positions={pose['positions']} | "
            f"duration={duration:.2f}s"
        )

        future = self.arm_client.send_goal_async(
            goal,
            feedback_callback=self.arm_feedback_callback,
        )
        future.add_done_callback(self.arm_goal_response_callback)

    def arm_goal_response_callback(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as error:
            self.get_logger().error(f"팔 Goal 전송 실패: {error}")
            self.finish_failed_arm_goal()
            return

        if not goal_handle.accepted:
            self.get_logger().warning("팔 Goal이 액션 서버에서 거절되었습니다.")
            self.finish_failed_arm_goal()
            return

        self.get_logger().info(f"팔 Goal 승인: {self.get_current_pose_name()}")

        # 팔이 움직이기 시작할 때 그리퍼 Goal도 같이 보낸다.
        if (
            self.use_gripper
            and self.current_pose is not None
            and "gripper" in self.current_pose
        ):
            self.send_gripper_goal(float(self.current_pose["gripper"]))

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.arm_result_callback)

    def arm_feedback_callback(self, feedback_message) -> None:
        # 피드백이 매우 자주 오므로 1초에 한 번만 출력한다.
        current_time = time.monotonic()
        if current_time - self.last_feedback_log_time < 1.0:
            return

        self.last_feedback_log_time = current_time
        feedback = feedback_message.feedback
        actual = list(feedback.actual.positions)
        desired = list(feedback.desired.positions)

        if actual and len(actual) == len(desired):
            max_error = max(
                abs(target - present)
                for target, present in zip(desired, actual)
            )
            self.get_logger().info(
                f"팔 Feedback: {self.get_current_pose_name()} | "
                f"최대 위치 오차={max_error:.4f} rad"
            )

    def arm_result_callback(self, future) -> None:
        try:
            wrapped_result = future.result()
            status = wrapped_result.status
            result = wrapped_result.result
        except Exception as error:
            self.get_logger().error(f"팔 Result 수신 실패: {error}")
            self.finish_failed_arm_goal()
            return

        pose_name = self.get_current_pose_name()
        succeeded = (
            status == GoalStatus.STATUS_SUCCEEDED
            and result.error_code == FollowJointTrajectory.Result.SUCCESSFUL
        )

        if succeeded:
            self.get_logger().info(f"팔 동작 완료: {pose_name}")
            if self.current_pose_index is not None:
                self.last_pose_index = self.current_pose_index

            pause = 0.0
            if self.current_pose is not None:
                pause = float(self.current_pose["pause"])

            self.arm_goal_active = False
            self.next_motion_time = time.monotonic() + pause
        else:
            error_string = result.error_string or "상세 오류 없음"
            self.get_logger().error(
                f"팔 동작 실패: {pose_name} | status={status}, "
                f"error_code={result.error_code}, error_string={error_string}"
            )
            self.finish_failed_arm_goal()

        self.current_pose = None
        self.current_pose_index = None

    def finish_failed_arm_goal(self) -> None:
        self.arm_goal_active = False
        self.current_pose = None
        self.current_pose_index = None
        self.next_motion_time = time.monotonic() + 1.0

    def get_current_pose_name(self) -> str:
        if self.current_pose is None:
            return "unknown"
        return str(self.current_pose["name"])

    def send_gripper_goal(self, position: float) -> None:
        if not self.gripper_client.server_is_ready():
            if not self.waiting_for_gripper_logged:
                self.get_logger().warning(
                    "/gripper_controller/gripper_cmd 액션 서버가 준비되지 않아 "
                    "이번 그리퍼 동작은 건너뜁니다."
                )
                self.waiting_for_gripper_logged = True
            return

        self.waiting_for_gripper_logged = False
        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = 10.0

        future = self.gripper_client.send_goal_async(goal)
        future.add_done_callback(self.gripper_goal_response_callback)

    def gripper_goal_response_callback(self, future) -> None:
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().warning("그리퍼 동작 요청이 거절되었습니다.")
        except Exception as error:
            self.get_logger().error(f"그리퍼 동작 요청 실패: {error}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None

    try:
        node = DanceManipulatorAction()
        rclpy.spin(node)
    except (FileNotFoundError, ValueError, yaml.YAMLError) as error:
        print(f"[dance_manipulator_action] 데이터 파일 오류: {error}")
    except KeyboardInterrupt:
        print("키보드 인터럽트")
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
