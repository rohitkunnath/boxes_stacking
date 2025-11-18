#!/usr/bin/env python3
"""
Autonomous stacking demo: 
1. Picks the green box and places it on the blue box
2. Picks the red box and places it on top of the green box
Final stack: Blue (bottom) -> Green (middle) -> Red (top)
"""

from threading import Thread
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from pymoveit2 import MoveIt2, GripperInterface
from pymoveit2.robots import panda

import math
import time

class PickAndPlace(Node):
    def __init__(self):
        super().__init__("pick_and_place")

        # Parameters
        self.declare_parameter("green_position", [0.60, -0.20, 0.05])
        self.declare_parameter("blue_position", [0.61, 0.20, 0.05])
        self.declare_parameter("red_position", [0.60, 0.0, 0.05]) 
        self.declare_parameter("green_box_height", 0.05)
        self.declare_parameter("blue_box_height", 0.05)
        self.declare_parameter("red_box_height", 0.05)
        self.declare_parameter("hover_offset", 0.40)
        self.declare_parameter("grasp_offset", 0.03)
        self.declare_parameter("place_offset", 0.13)
        
        # New red-specific offsets (independent)
        self.declare_parameter("red_hover_offset", 0.50)  # higher hover only for red box placement
        self.declare_parameter("red_place_offset", 0.18)  # higher place only for red box placement

        self.declare_parameter("cartesian_step", 0.01)
        self.declare_parameter("cartesian_fraction", 0.9)

        self.green_position = self.get_parameter("green_position").value
        self.blue_position = self.get_parameter("blue_position").value
        self.red_position = self.get_parameter("red_position").value
        self.green_box_height = float(self.get_parameter("green_box_height").value)
        self.blue_box_height = float(self.get_parameter("blue_box_height").value)
        self.red_box_height = float(self.get_parameter("red_box_height").value)
        self.hover_offset = float(self.get_parameter("hover_offset").value)
        self.grasp_offset = float(self.get_parameter("grasp_offset").value)
        self.place_offset = float(self.get_parameter("place_offset").value)
        
        # Load new red-specific offsets
        self.red_hover_offset = float(self.get_parameter("red_hover_offset").value)
        self.red_place_offset = float(self.get_parameter("red_place_offset").value)

        self.cartesian_step = float(self.get_parameter("cartesian_step").value)
        self.cartesian_fraction = float(
            self.get_parameter("cartesian_fraction").value
        )

        self.quat_xyzw = [0.0, 1.0, 0.0, 0.0]

        self.callback_group = ReentrantCallbackGroup()

        # Arm MoveIt2 interface
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=panda.joint_names(),
            base_link_name=panda.base_link_name(),
            end_effector_name=panda.end_effector_name(),
            group_name=panda.MOVE_GROUP_ARM,
            callback_group=self.callback_group,
        )

        # Set lower velocity & acceleration for smoother motion
        self.moveit2.max_velocity = 0.1
        self.moveit2.max_acceleration = 0.1

        # Gripper interface
        self.gripper = GripperInterface(
            node=self,
            gripper_joint_names=panda.gripper_joint_names(),
            open_gripper_joint_positions=panda.OPEN_GRIPPER_JOINT_POSITIONS,
            closed_gripper_joint_positions=panda.CLOSED_GRIPPER_JOINT_POSITIONS,
            gripper_group_name=panda.MOVE_GROUP_GRIPPER,
            callback_group=self.callback_group,
            gripper_command_action_name="gripper_action_controller/gripper_cmd",
        )

        # Predefined joint positions (in radians)
        self.start_joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, math.radians(-125.0)]
        self.home_joints  = [0.0, 0.0, 0.0, math.radians(-90.0), 0.0, math.radians(92.0), math.radians(50.0)]
        # Move to start joint configuration
        self.moveit2.move_to_configuration(self.start_joints)
        self.moveit2.wait_until_executed()

        # Start autonomous stacking in a background thread
        self.sequence_thread = Thread(target=self.run_sequence, daemon=True)
        self.sequence_thread.start()

    def move_to_pose(self, position, *, cartesian=False):
        self.moveit2.move_to_pose(
            position=position,
            quat_xyzw=self.quat_xyzw,
            cartesian=cartesian,
            cartesian_max_step=self.cartesian_step,
            cartesian_fraction_threshold=self.cartesian_fraction if cartesian else 0.0,
        )
        self.moveit2.wait_until_executed()

    def _box_top(self, base_position, height):
        return base_position[2] + height

    def pick_from_green(self):
        gx, gy, gz = self.green_position
        green_top = self._box_top(self.green_position, self.green_box_height)
        above = [gx, gy, green_top + self.hover_offset]
        grasp = [gx, gy, green_top + self.grasp_offset]

        self.get_logger().info("Moving above the green box...")
        self.move_to_pose(above)

        self.get_logger().info("Opening gripper...")
        self.gripper.open()
        self.gripper.wait_until_executed()

        self.get_logger().info("Descending to grasp the green box...")
        self.move_to_pose(grasp, cartesian=True)

        self.get_logger().info("Closing gripper...")
        self.gripper.close()
        self.gripper.wait_until_executed()

        self.get_logger().info("Lifting the green box...")
        self.move_to_pose(above, cartesian=True)

    def place_on_blue(self):
        bx, by, bz = self.blue_position
        blue_top = self._box_top(self.blue_position, self.blue_box_height)
        stack_top = blue_top + self.green_box_height
        above = [bx, by, stack_top + self.hover_offset]
        place = [bx, by, stack_top + self.place_offset]

        self.get_logger().info("Moving above the blue box...")
        self.move_to_pose(above)
        wait_duration_seconds = 10
        self.get_logger().info("Lowering to place the green box on top...")
        self.move_to_pose(place, cartesian=True)

        self.get_logger().info("Opening gripper to release the green box...")
        self.gripper.open()
        self.gripper.wait_until_executed()
        time.sleep(10)
        self.get_logger().info("Retracting from the stack...")
        self.move_to_pose(above, cartesian=True)

    def pick_from_red(self):
        rx, ry, rz = self.red_position
        red_top = self._box_top(self.red_position, self.red_box_height)
        above = [rx, ry, red_top + self.hover_offset]
        grasp = [rx, ry, red_top + self.grasp_offset]

        self.get_logger().info("Moving above the red box...")
        self.move_to_pose(above)
        
        self.get_logger().info("Opening gripper...")
        self.gripper.open()
        self.gripper.wait_until_executed()

        self.get_logger().info("Descending to grasp the red box...")
        self.move_to_pose(grasp, cartesian=True)

        self.get_logger().info("Closing gripper...")
        self.gripper.close()
        self.gripper.wait_until_executed()
        
        self.get_logger().info("Lifting the red box...")
        self.move_to_pose(above, cartesian=True)

    def place_on_green_stack(self):
        """Place red box on top of the green box (which is on blue box)"""
        bx, by, bz = self.blue_position
        blue_top = self._box_top(self.blue_position, self.blue_box_height)
        # Stack now has: blue + green + red
        stack_top = blue_top + self.green_box_height + self.red_box_height

        # Use red-specific offsets here (different from green placing)
        above = [bx, by, stack_top + self.red_hover_offset]
        place = [bx, by, stack_top + self.red_place_offset]

        self.get_logger().info("Moving above the green stack (red-specific offset)...")
        self.move_to_pose(above)

        self.get_logger().info("Lowering to place the red box on top (red-specific offset)...")
        self.move_to_pose(place, cartesian=True)

        self.get_logger().info("Opening gripper to release the red box...")
        self.gripper.open()
        self.gripper.wait_until_executed()
        time.sleep(10)
        self.get_logger().info("Retracting from the stack...")
        self.move_to_pose(above, cartesian=True)

    def run_sequence(self):
        try:
            self.get_logger().info("Starting autonomous stacking sequence.")
            self.moveit2.move_to_configuration(self.home_joints)
            self.moveit2.wait_until_executed()

            # Step 1: Pick green box and place on blue box
            self.get_logger().info("=== Step 1: Picking green box and placing on blue box ===")
            self.pick_from_green()
            self.place_on_blue()

            # Return to home before second pick
            self.get_logger().info("Returning to home configuration...")
            self.moveit2.move_to_configuration(self.home_joints)
            self.moveit2.wait_until_executed()

            # Step 2: Pick red box and place on top of green box
            self.get_logger().info("=== Step 2: Picking red box and placing on green stack ===")
            self.pick_from_red()
            self.place_on_green_stack()

            self.get_logger().info("Returning to home configuration...")
            self.moveit2.move_to_configuration(self.home_joints)
            self.moveit2.wait_until_executed()

            self.get_logger().info("Autonomous stacking sequence complete. Stack: Blue -> Green -> Red")
        except Exception as exc:
            self.get_logger().error(f"Stacking sequence failed: {exc}")
        finally:
            rclpy.shutdown()


def main():
    rclpy.init()
    node = PickAndPlace()

    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    try:
        executor_thread.join()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
