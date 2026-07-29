"""OpenManipulator-X MoveItPy arm·gripper 실습 launch."""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description() -> LaunchDescription:
    """MoveIt 설정 전체를 실습 노드에 전달한다."""
    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="open_manipulator_x",
            package_name="open_manipulator_moveit_config",
        )
        .robot_description_semantic(
            str(Path("config") / "open_manipulator_x" / "open_manipulator_x.srdf")
        )
        .joint_limits(str(Path("config") / "open_manipulator_x" / "joint_limits.yaml"))
        .trajectory_execution(
            str(Path("config") / "open_manipulator_x" / "moveit_controllers.yaml")
        )
        .robot_description_kinematics(
            str(Path("config") / "open_manipulator_x" / "kinematics.yaml")
        )
        .to_moveit_configs()
    )

    # MoveItConfigsBuilder의 planning_pipelines는 move_group 형식이다.
    # MoveItPy(MoveItCpp)는 planning_pipelines.pipeline_names를 요구한다.
    moveit_py_parameters = moveit_config.to_dict()
    moveit_py_parameters.pop("planning_pipelines", None)
    moveit_py_parameters.pop("default_planning_pipeline", None)
    moveit_py_parameters["planning_pipelines"] = {
        "pipeline_names": ["ompl"],
    }
    moveit_py_parameters["plan_request_params"] = {
        "planning_attempts": 1,
        "planning_pipeline": "ompl",
        "planner_id": "RRTConnectkConfigDefault",
        "max_velocity_scaling_factor": 0.3,
        "max_acceleration_scaling_factor": 0.3,
        "planning_time": 5.0,
    }
    moveit_py_parameters["planning_scene_monitor_options"] = {
        "name": "planning_scene_monitor",
        "robot_description": "robot_description",
        "joint_state_topic": "/joint_states",
        "attached_collision_object_topic": ("/moveit_cpp/planning_scene_monitor"),
        "publish_planning_scene_topic": "/moveit_cpp/publish_planning_scene",
        "monitored_planning_scene_topic": ("/moveit_cpp/monitored_planning_scene"),
        "wait_for_initial_state_timeout": 10.0,
    }
    use_sim_time = LaunchConfiguration("use_sim_time")

    urdf_xacro = PathJoinSubstitution(
        [
            FindPackageShare("open_manipulator_description"),
            "urdf",
            "open_manipulator_x",
            "open_manipulator_x.urdf.xacro",
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(
            Command(["xacro ", urdf_xacro]),
            value_type=str,
        )
    }

    moveit_py_node = Node(
        package="tf2_basic",
        executable="moveit_test",
        name="open_manipulator_moveit_py",
        output="screen",
        parameters=[
            moveit_py_parameters,
            robot_description,
            {"use_sim_time": use_sim_time},
        ],
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Gazebo의 /clock 사용 여부",
            ),
            moveit_py_node,
        ]
    )