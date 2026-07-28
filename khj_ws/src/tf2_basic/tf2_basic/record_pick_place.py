"""OpenMANIPULATOR-X의 팔 자세와 그리퍼 상태를 YAML로 기록한다.

키:
  O: 다음 저장 자세의 그리퍼를 열림으로 설정
  C: 다음 저장 자세의 그리퍼를 닫힘으로 설정
  SPACE: 현재 팔 자세와 선택된 그리퍼 상태 저장
  U: 마지막 저장 자세 삭제
  H: 도움말
  Q: 저장 후 종료
"""

from datetime import datetime
import os
from pathlib import Path
import select
import sys
import termios
import tty
from typing import Any

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool
import yaml


class RecordPickPlace(Node):
    JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4"]
    GRIPPER_JOINT = "gripper_left_joint"

    JOINT_LIMITS = {
        "joint1": [-3.14159265359, 3.14159265359],
        "joint2": [-1.5, 1.5],
        "joint3": [-1.5, 1.4],
        "joint4": [-1.7, 1.97],
    }

    # GripperCommand에 넣을 명령값이다.
    GRIPPER_OPEN = 0.019
    GRIPPER_CLOSED = -0.010
    GRIPPER_LIMITS = [-0.011, 0.020]

    def __init__(self) -> None:
        super().__init__("record_pick_place")

        self.declare_parameter("output_file", str(Path.home() / "pick_place_record.yaml"))
        self.declare_parameter(
            "torque_service",
            "/dynamixel_hardware_interface/set_dxl_torque",
        )
        self.declare_parameter("pattern_name", "pick_and_place")
        self.declare_parameter("step_duration", 2.0)
        self.declare_parameter("step_pause", 0.5)
        self.declare_parameter("overwrite", True)

        self.output_path = self._resolve_output_path()
        self.pattern_name = str(self.get_parameter("pattern_name").value).strip()
        self.step_duration = float(self.get_parameter("step_duration").value)
        self.step_pause = float(self.get_parameter("step_pause").value)

        if not self.pattern_name:
            raise ValueError("pattern_name은 비어 있을 수 없습니다.")
        if self.step_duration <= 0.0:
            raise ValueError("step_duration은 0보다 커야 합니다.")
        if self.step_pause < 0.0:
            raise ValueError("step_pause는 0 이상이어야 합니다.")

        self.latest_positions: dict[str, float] = {}
        self.steps: list[dict[str, Any]] = []
        self.selected_gripper = self.GRIPPER_OPEN

        self.torque_disabled = False
        self.torque_request_in_flight = False
        self.service_wait_count = 0
        self.quit_requested = False

        self.stdin_fd: int | None = None
        self.terminal_settings: list[Any] | None = None

        if not sys.stdin.isatty():
            raise RuntimeError("키보드 입력이 가능한 터미널에서 실행해야 합니다.")

        self.stdin_fd = sys.stdin.fileno()
        self.terminal_settings = termios.tcgetattr(self.stdin_fd)
        tty.setcbreak(self.stdin_fd)

        self.joint_state_subscription = self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_callback,
            10,
        )

        service_name = str(self.get_parameter("torque_service").value)
        self.torque_client = self.create_client(SetBool, service_name)

        self.startup_timer = self.create_timer(0.2, self.disable_torque_when_ready)
        self.keyboard_timer = self.create_timer(0.03, self.poll_keyboard)

        self.get_logger().info(f"저장 파일: {self.output_path}")
        self.get_logger().warning(
            "토크 OFF 서비스를 기다리는 중입니다. 아직 로봇팔을 손으로 움직이지 마세요."
        )

    def _resolve_output_path(self) -> Path:
        configured = str(self.get_parameter("output_file").value).strip()
        if configured:
            path = Path(os.path.expandvars(configured)).expanduser()
        else:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            path = Path.home() / f"pick_place_record_{timestamp}.yaml"

        if not path.is_absolute():
            path = Path.cwd() / path

        path = path.resolve()
        overwrite = bool(self.get_parameter("overwrite").value)

        if path.exists() and not overwrite:
            raise FileExistsError(
                f"파일이 이미 존재합니다: {path}. "
                "-p overwrite:=true를 지정하면 덮어씁니다."
            )

        return path

    def joint_state_callback(self, message: JointState) -> None:
        available = {
            name: float(position)
            for name, position in zip(message.name, message.position)
        }

        required = self.JOINT_NAMES + [self.GRIPPER_JOINT]
        if not all(name in available for name in required):
            return

        self.latest_positions = {name: available[name] for name in required}

    def disable_torque_when_ready(self) -> None:
        if self.torque_disabled or self.torque_request_in_flight:
            return

        if not self.torque_client.service_is_ready():
            self.service_wait_count += 1
            if self.service_wait_count % 25 == 1:
                self.get_logger().info("토크 OFF 서비스를 기다리는 중...")
            return

        request = SetBool.Request()
        request.data = False

        self.torque_request_in_flight = True
        future = self.torque_client.call_async(request)
        future.add_done_callback(self.torque_off_response)

    def torque_off_response(self, future: Any) -> None:
        self.torque_request_in_flight = False

        try:
            response = future.result()
        except Exception as error:
            self.get_logger().error(f"토크 OFF 서비스 호출 실패: {error}")
            return

        if response is None or not response.success:
            message = "응답 없음" if response is None else response.message
            self.get_logger().error(f"토크 OFF 실패: {message}")
            return

        self.torque_disabled = True
        self.startup_timer.cancel()
        self.destroy_timer(self.startup_timer)

        self.get_logger().warning(
            "토크가 OFF 되었습니다. 로봇팔을 손으로 지지하면서 움직이세요."
        )
        self.print_help()

    def poll_keyboard(self) -> None:
        if self.stdin_fd is None or self.quit_requested:
            return

        readable, _, _ = select.select([self.stdin_fd], [], [], 0.0)
        if not readable:
            return

        key = os.read(self.stdin_fd, 1)

        if key == b" ":
            self.capture_pose()
        elif key.lower() == b"o":
            self.selected_gripper = self.GRIPPER_OPEN
            self.get_logger().info(
                f"다음 저장 그리퍼 상태: OPEN ({self.selected_gripper:+.3f})"
            )
        elif key.lower() == b"c":
            self.selected_gripper = self.GRIPPER_CLOSED
            self.get_logger().info(
                f"다음 저장 그리퍼 상태: CLOSED ({self.selected_gripper:+.3f})"
            )
        elif key.lower() == b"u":
            self.undo_last_pose()
        elif key.lower() == b"h":
            self.print_help()
        elif key.lower() == b"q":
            self.request_quit()

    def capture_pose(self) -> None:
        if not self.torque_disabled:
            self.get_logger().warning(
                "토크 OFF가 확인되지 않아 자세를 저장하지 않았습니다."
            )
            return

        required = self.JOINT_NAMES + [self.GRIPPER_JOINT]
        missing = [name for name in required if name not in self.latest_positions]

        if missing:
            self.get_logger().warning(
                f"/joint_states에서 아직 받지 못한 관절: {missing}"
            )
            return

        positions = [
            round(self.latest_positions[name], 6)
            for name in self.JOINT_NAMES
        ]
        measured_gripper = round(
            self.latest_positions[self.GRIPPER_JOINT],
            6,
        )

        step_number = len(self.steps) + 1
        gripper_state = (
            "open"
            if self.selected_gripper == self.GRIPPER_OPEN
            else "closed"
        )

        self.steps.append(
            {
                "name": f"step_{step_number:03d}",
                "positions": positions,
                "gripper": self.selected_gripper,
                "gripper_state": gripper_state,
                "measured_gripper": measured_gripper,
                "duration": self.step_duration,
                "pause": self.step_pause,
            }
        )

        self.write_yaml()

        values = ", ".join(f"{value:+.4f}" for value in positions)
        self.get_logger().info(
            f"자세 {step_number} 저장: joint=[{values}], "
            f"gripper={gripper_state}({self.selected_gripper:+.3f}), "
            f"measured={measured_gripper:+.5f}"
        )

    def undo_last_pose(self) -> None:
        if not self.steps:
            self.get_logger().warning("삭제할 자세가 없습니다.")
            return

        removed = self.steps.pop()
        self.write_yaml()
        self.get_logger().info(f"마지막 자세 삭제: {removed['name']}")

    def write_yaml(self) -> None:
        document = {
            "joint_names": self.JOINT_NAMES,
            "joint_limits": self.JOINT_LIMITS,
            "gripper_joint": self.GRIPPER_JOINT,
            "gripper_limits": self.GRIPPER_LIMITS,
            "execution": {
                "repeat_count": 1,
                "default_duration": self.step_duration,
                "default_pause": self.step_pause,
                "gripper_max_effort": 10.0,
                "arm_first": True,
            },
            "patterns": [
                {
                    "name": self.pattern_name,
                    "steps": self.steps,
                }
            ],
        }

        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.output_path.with_suffix(
            self.output_path.suffix + ".tmp"
        )

        with temporary.open("w", encoding="utf-8") as stream:
            yaml.safe_dump(
                document,
                stream,
                allow_unicode=True,
                sort_keys=False,
            )

        temporary.replace(self.output_path)

    def print_help(self) -> None:
        self.get_logger().info(
            "[O] 그리퍼 열림 선택  [C] 그리퍼 닫힘 선택  "
            "[SPACE] 자세 저장  [U] 취소  [H] 도움말  [Q] 종료"
        )
        self.get_logger().info(
            "그리퍼는 손으로 벌린 값을 그대로 저장하지 않고, "
            "O/C로 선택한 안전한 GripperCommand 값을 저장합니다."
        )

    def request_quit(self) -> None:
        self.quit_requested = True
        self.restore_terminal()

        if self.steps:
            self.write_yaml()
            self.get_logger().info(
                f"{len(self.steps)}개 자세 저장 완료: {self.output_path}"
            )
        else:
            self.get_logger().warning("저장된 자세가 없습니다.")

        self.get_logger().warning(
            "종료 후에도 토크는 OFF입니다. "
            "재생 전 로봇을 첫 자세와 같은 안전한 자세로 놓고 토크를 ON 하세요."
        )
        rclpy.shutdown()

    def restore_terminal(self) -> None:
        if self.stdin_fd is None or self.terminal_settings is None:
            return

        termios.tcsetattr(
            self.stdin_fd,
            termios.TCSADRAIN,
            self.terminal_settings,
        )
        self.terminal_settings = None

    def destroy_node(self) -> bool:
        self.restore_terminal()
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = None

    try:
        node = RecordPickPlace()
        rclpy.spin(node)
    except (
        ExternalShutdownException,
        FileExistsError,
        KeyboardInterrupt,
        OSError,
        RuntimeError,
        ValueError,
    ) as error:
        if not isinstance(error, (ExternalShutdownException, KeyboardInterrupt)):
            if node is not None:
                node.get_logger().fatal(str(error))
            else:
                print(f"record_pick_place 오류: {error}")
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
