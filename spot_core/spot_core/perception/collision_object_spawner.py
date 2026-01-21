from __future__ import annotations
from typing import TYPE_CHECKING, Optional, Tuple, List
import logging
import numpy as np
import rclpy
from rclpy.node import Node
from bosdyn.client.frame_helpers import VISION_FRAME_NAME
from bosdyn.client.math_helpers import Quat, SE3Pose
from geometry_msgs.msg import PoseStamped, TransformStamped, Pose, Point
from moveit_msgs.msg import CollisionObject, PlanningScene
from shape_msgs.msg import SolidPrimitive, Mesh, MeshTriangle
from std_msgs.msg import Header

from synchros2.tf_listener_wrapper import TFListenerWrapper

logger = logging.getLogger(__name__)


class CollisionObjectSpawner(Node):
    """Spawns collision objects in MoveIt planning scene relative to fiducials."""

    def __init__(self):
        super().__init__("collision_object_spawner")
        self.tf_listener_wrapper = TFListenerWrapper(self)

        # Publisher for collision objects
        self.collision_pub = self.create_publisher(
            CollisionObject, "/collision_object", 10
        )

        # Publisher for planning scene (alternative method)
        self.scene_pub = self.create_publisher(PlanningScene, "/planning_scene", 10)

        # Wait for publishers to be ready
        self.get_logger().info("Collision object spawner initialized")

    def spawn_box_relative_to_fiducial(
        self,
        fiducial_name: str,
        object_id: str,
        dimensions: Tuple[float, float, float],
        offset: Tuple[float, float, float] = (0.0, 0.0, 0.0),
        orientation: Tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0),
        operation: int = CollisionObject.ADD,
    ):
        """
        Spawn a box collision object relative to a fiducial.

        Args:
            fiducial_name: Name of the fiducial frame
            object_id: Unique identifier for the collision object
            dimensions: (length_x, width_y, height_z) in meters
            offset: (x, y, z) offset from fiducial in meters
            orientation: (x, y, z, w) quaternion orientation
            operation: ADD, REMOVE, APPEND, or MOVE
        """
        pose = self._get_pose_relative_to_fiducial(fiducial_name, offset, orientation)

        if pose is None:
            self.get_logger().error(f"Failed to get pose for fiducial {fiducial_name}")
            return False

        collision_object = CollisionObject()
        collision_object.header = pose.header
        collision_object.id = object_id
        collision_object.operation = operation

        # Define box primitive
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = list(dimensions)

        collision_object.primitives.append(box)
        collision_object.primitive_poses.append(pose.pose)

        self.collision_pub.publish(collision_object)
        self.get_logger().info(f"Spawned box '{object_id}' relative to {fiducial_name}")
        return True

    def spawn_cylinder_relative_to_fiducial(
        self,
        fiducial_name: str,
        object_id: str,
        height: float,
        radius: float,
        offset: Tuple[float, float, float] = (0.0, 0.0, 0.0),
        orientation: Tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0),
        operation: int = CollisionObject.ADD,
    ):
        """
        Spawn a cylinder collision object relative to a fiducial.

        Args:
            fiducial_name: Name of the fiducial frame
            object_id: Unique identifier for the collision object
            height: Height of cylinder in meters
            radius: Radius of cylinder in meters
            offset: (x, y, z) offset from fiducial in meters
            orientation: (x, y, z, w) quaternion orientation
            operation: ADD, REMOVE, APPEND, or MOVE
        """
        pose = self._get_pose_relative_to_fiducial(fiducial_name, offset, orientation)

        if pose is None:
            self.get_logger().error(f"Failed to get pose for fiducial {fiducial_name}")
            return False

        collision_object = CollisionObject()
        collision_object.header = pose.header
        collision_object.id = object_id
        collision_object.operation = operation

        # Define cylinder primitive
        cylinder = SolidPrimitive()
        cylinder.type = SolidPrimitive.CYLINDER
        cylinder.dimensions = [height, radius]

        collision_object.primitives.append(cylinder)
        collision_object.primitive_poses.append(pose.pose)

        self.collision_pub.publish(collision_object)
        self.get_logger().info(
            f"Spawned cylinder '{object_id}' relative to {fiducial_name}"
        )
        return True

    def spawn_sphere_relative_to_fiducial(
        self,
        fiducial_name: str,
        object_id: str,
        radius: float,
        offset: Tuple[float, float, float] = (0.0, 0.0, 0.0),
        operation: int = CollisionObject.ADD,
    ):
        """
        Spawn a sphere collision object relative to a fiducial.

        Args:
            fiducial_name: Name of the fiducial frame
            object_id: Unique identifier for the collision object
            radius: Radius of sphere in meters
            offset: (x, y, z) offset from fiducial in meters
            operation: ADD, REMOVE, APPEND, or MOVE
        """
        pose = self._get_pose_relative_to_fiducial(
            fiducial_name, offset, (0.0, 0.0, 0.0, 1.0)
        )

        if pose is None:
            self.get_logger().error(f"Failed to get pose for fiducial {fiducial_name}")
            return False

        collision_object = CollisionObject()
        collision_object.header = pose.header
        collision_object.id = object_id
        collision_object.operation = operation

        # Define sphere primitive
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [radius]

        collision_object.primitives.append(sphere)
        collision_object.primitive_poses.append(pose.pose)

        self.collision_pub.publish(collision_object)
        self.get_logger().info(
            f"Spawned sphere '{object_id}' relative to {fiducial_name}"
        )
        return True

    def spawn_hammer_relative_to_fiducial(
        self,
        fiducial_name: str,
        object_id: str = "hammer",
        handle_length: float = 0.25,
        handle_radius: float = 0.015,
        head_length: float = 0.10,
        head_width: float = 0.04,
        head_height: float = 0.04,
        offset: Tuple[float, float, float] = (0.0, 0.0, 0.0),
        operation: int = CollisionObject.ADD,
    ):
        """
        Spawn a hammer-like collision object composed of a handle and head.
        """

        base_pose = self._get_pose_relative_to_fiducial(
            fiducial_name, offset, (0.0, 0.0, 0.0, 1.0)
        )

        if base_pose is None:
            self.get_logger().error(f"Failed to get pose for fiducial {fiducial_name}")
            return False

        collision_object = CollisionObject()
        collision_object.header = base_pose.header
        collision_object.id = object_id
        collision_object.operation = operation

        collision_object.pose.position.x = base_pose.pose.position.x
        collision_object.pose.position.y = base_pose.pose.position.y
        collision_object.pose.position.z = (
            base_pose.pose.position.z + handle_length / 2.0
        )

        # ----------------
        # Handle (box)
        # ----------------
        handle = SolidPrimitive()
        handle.type = SolidPrimitive.BOX
        handle.dimensions = [
            handle_length,
            handle_radius * 2.0,
            handle_radius * 2.0,
        ]

        handle_pose = Pose()
        handle_pose.position.x = base_pose.pose.position.x
        handle_pose.position.y = base_pose.pose.position.y
        handle_pose.position.z = base_pose.pose.position.z + handle_length / 2.0
        handle_pose.orientation = base_pose.pose.orientation

        collision_object.primitives.append(handle)
        collision_object.primitive_poses.append(Pose())

        # ----------------
        # Head (box)
        # ----------------
        head = SolidPrimitive()
        head.type = SolidPrimitive.BOX
        head.dimensions = [
            head_length,
            head_width,
            head_height,
        ]

        head_pose = Pose()
        head_pose.position.x = base_pose.pose.position.x
        head_pose.position.y = base_pose.pose.position.y
        head_pose.position.z = base_pose.pose.position.z + handle_length
        head_pose.orientation.x = 0.7071068
        head_pose.orientation.w = 0.7071068

        collision_object.primitives.append(head)
        collision_object.primitive_poses.append(head_pose)

        self.collision_pub.publish(collision_object)
        self.get_logger().info(
            f"Spawned hammer '{object_id}' relative to {fiducial_name}"
        )
        return True

    def spawn_multiple_objects(self, fiducial_name: str, objects: List[dict]):
        """
        Spawn multiple collision objects at once.

        Args:
            fiducial_name: Name of the fiducial frame
            objects: List of object specifications, each containing:
                - type: 'box', 'cylinder', or 'sphere'
                - id: unique identifier
                - dimensions: shape-specific dimensions
                - offset: (x, y, z) offset from fiducial
                - orientation: (optional) (x, y, z, w) quaternion

        Example:
            objects = [
                {
                    'type': 'box',
                    'id': 'table',
                    'dimensions': (1.0, 0.8, 0.05),
                    'offset': (0.0, 0.0, -0.025)
                },
                {
                    'type': 'cylinder',
                    'id': 'pole',
                    'dimensions': (0.5, 0.05),  # (height, radius)
                    'offset': (0.3, 0.0, 0.25)
                }
            ]
        """
        for obj in objects:
            obj_type = obj.get("type", "box")
            obj_id = obj["id"]
            dimensions = obj.get("dimensions", (0.1, 0.1, 0.1))
            offset = obj.get("offset", (0.0, 0.0, 0.0))
            orientation = obj.get("orientation", (0.0, 0.0, 0.0, 1.0))

            if obj_type == "box":
                self.spawn_box_relative_to_fiducial(
                    fiducial_name, obj_id, dimensions, offset, orientation
                )
            elif obj_type == "cylinder":
                height, radius = dimensions[0], dimensions[1]
                self.spawn_cylinder_relative_to_fiducial(
                    fiducial_name, obj_id, height, radius, offset, orientation
                )
            elif obj_type == "sphere":
                radius = (
                    dimensions[0]
                    if isinstance(dimensions, (list, tuple))
                    else dimensions
                )
                self.spawn_sphere_relative_to_fiducial(
                    fiducial_name, obj_id, radius, offset
                )
            else:
                self.get_logger().warning(f"Unknown object type: {obj_type}")

    def remove_collision_object(self, object_id: str):
        """Remove a collision object from the planning scene."""
        collision_object = CollisionObject()
        collision_object.id = object_id
        collision_object.operation = CollisionObject.REMOVE

        self.collision_pub.publish(collision_object)
        self.get_logger().info(f"Removed collision object '{object_id}'")

    def clear_all_objects(self):
        """Remove all collision objects from the planning scene."""
        collision_object = CollisionObject()
        collision_object.operation = CollisionObject.REMOVE
        # Empty ID removes all objects

        self.collision_pub.publish(collision_object)
        self.get_logger().info("Cleared all collision objects")

    def _get_pose_relative_to_fiducial(
        self,
        fiducial_name: str,
        offset: Tuple[float, float, float],
        orientation: Tuple[float, float, float, float],
    ) -> Optional[PoseStamped]:
        """Get pose relative to fiducial with offset and orientation."""
        try:
            # Wait for transform
            self.tf_listener_wrapper.wait_for_a_tform_b(
                VISION_FRAME_NAME, fiducial_name, timeout_sec=2.0
            )

            # Get fiducial transform
            fiducial_transform = self.tf_listener_wrapper.lookup_a_tform_b(
                VISION_FRAME_NAME, fiducial_name
            )

            # Create pose stamped
            pose = PoseStamped()
            pose.header.frame_id = VISION_FRAME_NAME
            pose.header.stamp = self.get_clock().now().to_msg()

            # Apply offset to fiducial position
            pose.pose.position.x = (
                fiducial_transform.transform.translation.x + offset[0]
            )
            pose.pose.position.y = (
                fiducial_transform.transform.translation.y + offset[1]
            )
            pose.pose.position.z = (
                fiducial_transform.transform.translation.z + offset[2]
            )

            # Set orientation
            pose.pose.orientation.x = orientation[0]
            pose.pose.orientation.y = orientation[1]
            pose.pose.orientation.z = orientation[2]
            pose.pose.orientation.w = orientation[3]

            return pose

        except Exception as e:
            self.get_logger().error(f"Failed to get fiducial pose: {e}")
            return None


def main(args=None):
    import threading

    rclpy.init(args=args)
    spawner = CollisionObjectSpawner()

    # Start spinning in a separate thread
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(spawner)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    # Wait for node to start spinning and TF to be available
    import time

    time.sleep(2.0)

    # Example usage: spawn a table and objects on it
    fiducial_name = "filtered_fiducial_1"

    spawner.spawn_hammer_relative_to_fiducial(
        fiducial_name=fiducial_name,
        object_id="hammer",
        offset=(0.0, 0.0, 0.05),  # on top of the table
    )

    # Keep node running
    try:
        spin_thread.join()
    except KeyboardInterrupt:
        # Clean up on exit
        spawner.get_logger().info("Cleaning up collision objects...")
        spawner.clear_all_objects()
    finally:
        executor.shutdown()
        spawner.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
