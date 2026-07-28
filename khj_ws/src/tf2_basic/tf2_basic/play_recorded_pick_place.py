"""기록된 OpenMANIPULATOR-X pick-and-place 동작을 순서대로 재생한다."""

import os
from pathlib import Path
import time
from typing import Any

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from control_msgs.action import FollowJointTrajectory, GripperCommand
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint
import yaml


class PlayRecordedPickPlace(Node):
    GRIPPER_MIN = -0.011
    GRIPPER_MAX = 0.020

    def __init__(self) -> None:
        super().__init__("play_recorded_pick_place")

        self.declare_parameter(
            "data_file",
            str(Path.home() / "pick_place_record.yaml"),
        )
        self.declare_parameter("pattern_name", "")
        self.declare_parameter("repeat_count", -1)
        self.declare_parameter("startup_delay", 2.0)
        self.declare_parameter("use_gripper", True)

        data_file = str(self.get_parameter("data_file").value)
        pattern_name = str(self.get_parameter("pattern_name").value).strip()
        repeat_override = int(self.get_parameter("repeat_count").value)
        startup_delay = float(self.get_parameter("startup_delay").value)
        self.use_gripper = bool(self.get_parameter("use_gripper").value)

        self.data_path = self.resolve_data_path(data_file)
        (
            self.joint_names,
            self.steps,
            yaml_repeat_count,
            self.gripper_max_effort,
        ) = self.load_recording(self.data_path, pattern_name)

        self.repeat_count = (
            yaml_repeat_count
            if repeat_override < 0
            else repeat_override
        )
        if self.repeat_count <= 0:
            raise ValueError("repeat_count는 1 이상이어야 합니다.")

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

        self.step_index = 0
        self.completed_repeats = 0
        self.current_step: dict[str, Any] | None = None
        self.phase = "waiting"
        self.next_start_time = time.monotonic() + max(0.0, startup_delay)
        self.server_warning_printed = False
        self.finished = False

        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info(f"재생 파일: {self.data_path}")
        self.get_logger().info(
            f"관절={self.joint_names}, 자세={len(self.steps)}개, "
            f"반복={self.repeat_count}회"
        )
        self.get_logger().warning(
            "동작 순서는 YAML에 저장된 순서 그대로 실행됩니다. "
            "주변을 비우고 비상 정지 준비 후 실행하세요."
        )

    def resolve_data_path(self, data_file: str) -> Path:
        expanded = Path(os.path.expandvars(data_file)).expanduser()

        if expanded.is_absolute():
            return expanded.resolve()

        cwd_candidate = (Path.cwd() / expanded).resolve()
        if cwd_candidate.is_file():
            return cwd_candidate

        package_candidate = (
            Path(get_package_share_directory("tf2_basic"))
            / "data"
            / expanded
        )
        return package_candidate.resolve()

    def load_recording(
        self,
        path: Path,
        requested_pattern: str,
    ) -> tuple[list[str], list[dict[str, Any]], int, float]:
        if not path.is_file():
            raise FileNotFoundError(f"녹화 YAML을 찾을 수 없습니다: {path}")

        with path.open("r", encoding="utf-8") as stream:
            document = yaml.safe_load(stream)

        if not isinstance(document, dict):
            raise ValueError("YAML 최상위는 딕셔너리여야 합니다.")

        joint_names = document.get("joint_names")
        if not isinstance(joint_names, list) or not joint_names:
            raise ValueError("joint_names는 비어 있지 않은 리스트여야 합니다.")
        joint_names = [str(name) for name in joint_names]

        execution = document.get("execution", {})
        if not isinstance(execution, dict):
            execution = {}

        repeat_count = int(execution.get("repeat_count", 1))
        default_duration = float(execution.get("default_duration", 2.0))
        default_pause = float(execution.get("default_pause", 0.5))
        gripper_max_effort = float(
            execution.get("gripper_max_effort", 10.0)
        )

        raw_steps: Any = None

        # 새 녹화 형식: patterns -> steps
        patterns = document.get("patterns")
        if isinstance(patterns, list) and patterns:
            selected_pattern = None

            if requested_pattern:
                selected_pattern = next(
                    (
                        pattern
                        for pattern in patterns
                        if isinstance(pattern, dict)
                        and str(pattern.get("name", "")) == requested_pattern
                    ),
                    None,
                )
                if selected_pattern is None:
                    raise ValueError(
                        f"pattern_name '{requested_pattern}'을 찾지 못했습니다."
                    )
            else:
                selected_pattern = patterns[0]

            if not isinstance(selected_pattern, dict):
                raise ValueError("선택된 pattern은 딕셔너리여야 합니다.")

            raw_steps = selected_pattern.get("steps")

        # 단순 형식: steps
        if raw_steps is None:
            raw_steps = document.get("steps")

        # 기존 dance 형식도 읽을 수 있게 지원
        if raw_steps is None:
            raw_steps = document.get("poses")

        if not isinstance(raw_steps, list) or not raw_steps:
            raise ValueError(
                "patterns[].steps, steps 또는 poses에서 동작을 찾지 못했습니다."
            )

        validated_steps: list[dict[str, Any]] = []

        for index, raw_step in enumerate(raw_steps):
            if not isinstance(raw_step, dict):
                raise ValueError(f"step[{index}]는 딕셔너리여야 합니다.")

            name = str(raw_step.get("name", f"step_{index + 1:03d}"))
            positions = raw_step.get("positions")

            if not isinstance(positions, list):
                raise ValueError(f"{name}: positions는 리스트여야 합니다.")
            if len(positions) != len(joint_names):
                raise ValueError(
                    f"{name}: positions 개수({len(positions)})와 "
                    f"joint_names 개수({len(joint_names)})가 다릅니다."
                )

            duration = float(raw_step.get("duration", default_duration))
            pause = float(raw_step.get("pause", default_pause))

            if duration <= 0.0:
                raise ValueError(f"{name}: duration은 0보다 커야 합니다.")
            if pause < 0.0:
                raise ValueError(f"{name}: pause는 0 이상이어야 합니다.")

            step: dict[str, Any] = {
                "name": name,
                "positions": [float(value) for value in positions],
                "duration": duration,
                "pause": pause,
            }

            if "gripper" in raw_step:
                raw_gripper = raw_step["gripper"]

                # 예전 teach_data.yaml의 [0.019] 형태도 읽는다.
                if isinstance(raw_gripper, list):
                    if len(raw_gripper) != 1:
                        raise ValueError(
                            f"{name}: gripper 리스트에는 값이 하나만 있어야 합니다."
                        )
                    raw_gripper = raw_gripper[0]

                gripper = float(raw_gripper)

                if not self.GRIPPER_MIN <= gripper <= self.GRIPPER_MAX:
                    raise ValueError(
                        f"{name}: gripper={gripper}가 안전 범위 "
                        f"[{self.GRIPPER_MIN}, {self.GRIPPER_MAX}] 밖입니다. "
                        "새 record_pick_place로 다시 녹화하세요."
                    )

                step["gripper"] = gripper

            validated_steps.append(step)

        return (
            joint_names,
            validated_steps,
            repeat_count,
            gripper_max_effort,
        )

    def timer_callback(self) -> None:
        if self.finished or self.phase != "waiting":
            return

        if time.monotonic() < self.next_start_time:
            return

        arm_ready = self.arm_client.server_is_ready()
        gripper_ready = (
            not self.use_gripper
            or "gripper" not in self.steps[self.step_index]
            or self.gripper_client.server_is_ready()
        )

        if not arm_ready or not gripper_ready:
            if not self.server_warning_printed:
                self.get_logger().warning(
                    "Action 서버를 기다리는 중: "
                    f"arm={arm_ready}, gripper={gripper_ready}"
                )
                self.server_warning_printed = True
            self.next_start_time = time.monotonic() + 1.0
            return

        self.server_warning_printed = False
        self.current_step = self.steps[self.step_index]
        self.send_arm_goal(self.current_step)

    def send_arm_goal(self, step: dict[str, Any]) -> None:
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = list(step["positions"])

        duration = float(step["duration"])
        seconds = int(duration)
        point.time_from_start.sec = seconds
        point.time_from_start.nanosec = int(
            (duration - seconds) * 1_000_000_000
        )

        goal.trajectory.points.append(point)

        self.phase = "arm"
        self.get_logger().info(
            f"[{self.step_index + 1}/{len(self.steps)}] "
            f"팔 이동 시작: {step['name']}"
        )

        future = self.arm_client.send_goal_async(
            goal,
            feedback_callback=self.arm_feedback_callback,
        )
        future.add_done_callback(self.arm_goal_response_callback)

    def arm_goal_response_callback(self, future: Any) -> None:
        try:
            goal_handle = future.result()
        except Exception as error:
            self.stop_with_error(f"팔 Goal 전송 실패: {error}")
            return

        if not goal_handle.accepted:
            self.stop_with_error("팔 Goal이 거절되었습니다.")
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.arm_result_callback)

    def arm_feedback_callback(self, feedback_message: Any) -> None:
        # 피드백은 컨트롤러가 자주 발행하므로 화면에는 생략한다.
        _ = feedback_message

    def arm_result_callback(self, future: Any) -> None:
        try:
            wrapped = future.result()
        except Exception as error:
            self.stop_with_error(f"팔 Result 수신 실패: {error}")
            return

        result = wrapped.result
        succeeded = (
            wrapped.status == GoalStatus.STATUS_SUCCEEDED
            and result.error_code
            == FollowJointTrajectory.Result.SUCCESSFUL
        )

        if not succeeded:
            self.stop_with_error(
                "팔 동작 실패: "
                f"status={wrapped.status}, "
                f"error_code={result.error_code}, "
                f"error_string={result.error_string}"
            )
            return

        if (
            self.current_step is not None
            and self.use_gripper
            and "gripper" in self.current_step
        ):
            self.send_gripper_goal(float(self.current_step["gripper"]))
        else:
            self.finish_current_step()

    def send_gripper_goal(self, position: float) -> None:
        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = self.gripper_max_effort

        self.phase = "gripper"
        state = "OPEN" if position > 0.0 else "CLOSED"
        self.get_logger().info(
            f"그리퍼 실행: {state} ({position:+.3f})"
        )

        future = self.gripper_client.send_goal_async(goal)
        future.add_done_callback(self.gripper_goal_response_callback)

    def gripper_goal_response_callback(self, future: Any) -> None:
        try:
            goal_handle = future.result()
        except Exception as error:
            self.stop_with_error(f"그리퍼 Goal 전송 실패: {error}")
            return

        if not goal_handle.accepted:
            self.stop_with_error("그리퍼 Goal이 거절되었습니다.")
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.gripper_result_callback)

    def gripper_result_callback(self, future: Any) -> None:
        try:
            wrapped = future.result()
        except Exception as error:
            self.stop_with_error(f"그리퍼 Result 수신 실패: {error}")
            return

        if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            self.stop_with_error(
                f"그리퍼 동작 실패: status={wrapped.status}"
            )
            return

        result = wrapped.result
        self.get_logger().info(
            "그리퍼 완료: "
            f"position={result.position:.4f}, "
            f"effort={result.effort:.4f}, "
            f"stalled={result.stalled}, "
            f"reached_goal={result.reached_goal}"
        )
        self.finish_current_step()

    def finish_current_step(self) -> None:
        if self.current_step is None:
            self.stop_with_error("현재 step 정보가 없습니다.")
            return

        self.get_logger().info(f"자세 완료: {self.current_step['name']}")
        pause = float(self.current_step["pause"])

        self.step_index += 1
        self.current_step = None

        if self.step_index >= len(self.steps):
            self.step_index = 0
            self.completed_repeats += 1
            self.get_logger().info(
                f"{self.completed_repeats}/{self.repeat_count}회 재생 완료"
            )

            if self.completed_repeats >= self.repeat_count:
                self.finished = True
                self.phase = "done"
                self.get_logger().info("전체 pick-and-place 재생이 끝났습니다.")
                return

        self.phase = "waiting"
        self.next_start_time = time.monotonic() + pause

    def stop_with_error(self, message: str) -> None:
        self.finished = True
        self.phase = "error"
        self.get_logger().error(message)
        self.get_logger().error(
            "안전을 위해 다음 동작을 중단했습니다."
        )


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = None

    try:
        node = PlayRecordedPickPlace()
        rclpy.spin(node)
    except (
        ExternalShutdownException,
        FileNotFoundError,
        KeyboardInterrupt,
        ValueError,
        yaml.YAMLError,
    ) as error:
        if not isinstance(error, (ExternalShutdownException, KeyboardInterrupt)):
            if node is not None:
                node.get_logger().fatal(str(error))
            else:
                print(f"play_recorded_pick_place 오류: {error}")
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
