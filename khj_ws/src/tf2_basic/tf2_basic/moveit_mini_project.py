from __future__ import annotations

import math
import os
import sys
import time
from collections.abc import Sequence

import rclpy
from geometry_msgs.msg import Pose
from moveit.planning import MoveItPy
from moveit_msgs.msg import CollisionObject
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive


class MoveItMiniProject(Node):
    """Create the obstacle course and move through its named SRDF poses."""

    ARM_CONTROLLER = "arm_controller"

    # The order deliberately avoids moving directly from +joint1 limit to
    # -joint1 limit. The arm visits the positive side, returns through the
    # center, and then visits the negative side.
    ROUTE = (
        "gap_a",
        "gap_b",
        "gap_c",
        "gap_d",
        "gap_c",
        "gap_b",
        "gap_a",
        "gap_f",
        "gap_e",
        "gap_f",
        "gap_a",
    )

    def __init__(self) -> None:
        super().__init__("moveit_mini_project")

        self.moveit = MoveItPy(node_name="open_manipulator_moveit_py")
        self.arm = self.moveit.get_planning_component("arm")
        self.planning_scene_monitor = self.moveit.get_planning_scene_monitor()

        # Publishing the same object message allows the move_group/RViz
        # planning scene to see the objects as well as this MoveItPy instance.
        self.collision_object_publisher = self.create_publisher(
            CollisionObject,
            "/collision_object",
            10,
        )

        with self.planning_scene_monitor.read_only() as scene:
            self.planning_frame = scene.planning_frame

        self.get_logger().info(f"planning frame: {self.planning_frame}")

    @staticmethod
    def yaw_to_quaternion(yaw: float) -> tuple[float, float, float, float]:
        """Return an x, y, z, w quaternion for a rotation around the Z axis."""
        half_yaw = yaw / 2.0
        return 0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw)

    def make_box_object(
        self,
        object_id: str,
        size: Sequence[float],
        position: Sequence[float],
        yaw: float = 0.0,
    ) -> CollisionObject:
        """Create one BOX CollisionObject.

        Args:
            object_id: Unique planning-scene object ID.
            size: Box dimensions in metres: (x, y, z).
            position: Box center in metres: (x, y, z).
            yaw: Rotation around the world Z axis in radians.
        """
        if len(size) != 3 or len(position) != 3:
            raise ValueError("size and position must each contain exactly 3 values")

        collision_object = CollisionObject()
        collision_object.header.frame_id = self.planning_frame
        collision_object.header.stamp = self.get_clock().now().to_msg()
        collision_object.id = object_id

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [float(value) for value in size]

        pose = Pose()
        pose.position.x = float(position[0])
        pose.position.y = float(position[1])
        pose.position.z = float(position[2])
        qx, qy, qz, qw = self.yaw_to_quaternion(yaw)
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw

        collision_object.primitives.append(primitive)
        collision_object.primitive_poses.append(pose)
        collision_object.operation = CollisionObject.ADD
        return collision_object

    def add_box(
        self,
        object_id: str,
        size: Sequence[float],
        position: Sequence[float],
        yaw: float = 0.0,
    ) -> bool:
        """Add or update one box in both the local and shared planning scenes."""
        collision_object = self.make_box_object(
            object_id=object_id,
            size=size,
            position=position,
            yaw=yaw,
        )

        success = self.planning_scene_monitor.process_collision_object(
            collision_object
        )
        self.collision_object_publisher.publish(collision_object)

        if success:
            self.get_logger().info(f"collision object added: {object_id}")
        else:
            self.get_logger().error(f"failed to add collision object: {object_id}")
        return bool(success)

    def add_table(self) -> bool:
        """Add the table whose top surface is just below world Z=0."""
        return self.add_box(
            object_id="mini_project_table",
            size=(0.90, 0.90, 0.05),
            position=(0.0, 0.0, -0.03),
        )

    def add_demo_object(self) -> bool:
        """Add a box for scene-object practice; it is not attached to the arm."""
        return self.add_box(
            object_id="mini_project_box",
            size=(0.05, 0.05, 0.08),
            position=(0.39, 0.0, 0.04),
        )

    def add_radial_walls(self) -> bool:
        """Add six walls shaped like the six radial spokes in the reference image."""
        wall_radius = 0.29
        wall_size = (0.16, 0.025, 0.22)
        wall_center_z = wall_size[2] / 2.0

        # Reference-image order:
        # wall1 top, wall2 upper-right, wall3 lower-right,
        # wall4 bottom, wall5 lower-left, wall6 upper-left.
        wall_angles_deg = (90.0, 30.0, -30.0, -90.0, -150.0, 150.0)

        all_succeeded = True
        for index, angle_deg in enumerate(wall_angles_deg, start=1):
            angle_rad = math.radians(angle_deg)
            x = wall_radius * math.cos(angle_rad)
            y = wall_radius * math.sin(angle_rad)

            succeeded = self.add_box(
                object_id=f"mini_project_wall_{index}",
                size=wall_size,
                position=(x, y, wall_center_z),
                yaw=angle_rad,
            )
            all_succeeded = all_succeeded and succeeded

        return all_succeeded

    def add_environment(self) -> bool:
        """Build all mini-project collision objects through common helpers."""
        results = (
            self.add_table(),
            self.add_demo_object(),
            self.add_radial_walls(),
        )

        # Give RViz/move_group time to receive the published object messages.
        time.sleep(1.0)
        return all(results)

    def list_scene_objects(self) -> None:
        """Print the IDs currently stored in this MoveItPy planning scene."""
        with self.planning_scene_monitor.read_only() as scene:
            object_ids = [
                collision_object.id
                for collision_object in scene.planning_scene_message.world.collision_objects
            ]

        self.get_logger().info(f"planning-scene objects: {object_ids}")

    def plan_and_execute_named_pose(self, configuration_name: str) -> bool:
        """Plan and execute one SRDF named state using its string name."""
        self.arm.set_start_state_to_current_state()
        self.arm.set_goal_state(configuration_name=configuration_name)

        self.get_logger().info(f"planning: {configuration_name}")
        plan_result = self.arm.plan()
        if not plan_result:
            self.get_logger().error(f"planning failed: {configuration_name}")
            return False

        self.get_logger().info(f"executing: {configuration_name}")
        self.moveit.execute(
            plan_result.trajectory,
            controllers=[self.ARM_CONTROLLER],
        )
        time.sleep(0.35)
        return True

    def run(self) -> bool:
        """Create the scene and move the arm through every wall gap."""
        # DDS discovery for /collision_object subscribers.
        time.sleep(1.0)

        if not self.add_environment():
            self.get_logger().error("environment creation failed")
            return False

        self.list_scene_objects()

        if not self.plan_and_execute_named_pose("init"):
            return False

        for pose_name in self.ROUTE:
            if not self.plan_and_execute_named_pose(pose_name):
                self.get_logger().error(
                    "Check that the same pose name exists in the SRDF and that "
                    "the pose is collision-free."
                )
                return False

        if not self.plan_and_execute_named_pose("init"):
            return False

        self.get_logger().info("mini project completed")
        return True


def exit_without_moveit_destructor(exit_code: int) -> None:
    """Avoid the MoveItPy shutdown SIGSEGV seen in the current Jazzy setup."""
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(exit_code)


def main() -> None:
    rclpy.init()
    node: MoveItMiniProject | None = None

    try:
        node = MoveItMiniProject()
        succeeded = node.run()
        exit_without_moveit_destructor(0 if succeeded else 1)
    except KeyboardInterrupt:
        exit_without_moveit_destructor(130)
    except Exception as exc:  # Keep terminal output useful during the lab.
        if node is not None:
            node.get_logger().error(f"mini project error: {exc}")
        else:
            print(f"mini project initialization error: {exc}", file=sys.stderr)
        exit_without_moveit_destructor(1)


if __name__ == "__main__":
    main()