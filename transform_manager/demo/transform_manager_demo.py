#!/usr/bin/env python3

import math
import sys
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped, PoseStamped
from rosiris_transform_interfaces.srv import (
    AddDynamicFrames,
    UpdateDynamicFrames,
    RemoveDynamicFrames,
    AddStaticFrames,
    GetPosesInFrames,
)
from rosiris_transform_interfaces.msg import PoseStampedWithTarget


def quaternion_from_roll(roll):
    qx = math.sin(roll / 2.0)
    qw = math.cos(roll / 2.0)
    return qx, 0.0, 0.0, qw


class TransformManagerTester(Node):

    def __init__(self, count, dx, roll):
        super().__init__("transform_manager_tester")

        self.count = count
        self.dx = dx
        self.roll = roll
        self.parent_frame = "end_effector_link"

        # ---- Clients ----
        self.add_dynamic_cli = self.create_client(
            AddDynamicFrames,
            "/transform_manager/add_dynamic_frames"
        )
        self.update_dynamic_cli = self.create_client(
            UpdateDynamicFrames,
            "/transform_manager/update_dynamic_frames"
        )
        self.remove_dynamic_cli = self.create_client(
            RemoveDynamicFrames,
            "/transform_manager/remove_dynamic_frames"
        )
        self.add_static_cli = self.create_client(
            AddStaticFrames,
            "/transform_manager/add_static_frames"
        )
        self.get_pose_cli = self.create_client(
            GetPosesInFrames,
            "/transform_manager/get_pose_in_frame"
        )

        for cli in [
            self.add_dynamic_cli,
            self.update_dynamic_cli,
            self.remove_dynamic_cli,
            self.add_static_cli,
            self.get_pose_cli,
        ]:
            while not cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f"Waiting for service {cli.srv_name}...")

    # ------------------------------------------------------------------
    # Dynamic Frames
    # ------------------------------------------------------------------

    def add_dynamic(self, prefix):
        request = AddDynamicFrames.Request()
        request.allow_override = True

        for i in range(self.count):
            t = TransformStamped()
            t.header.frame_id = self.parent_frame
            t.child_frame_id = f"{prefix}_{i}"

            t.transform.translation.x = self.dx * i
            t.transform.translation.y = 0.0
            t.transform.translation.z = 0.0

            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = 1.0

            request.frames.append(t)

        return self.add_dynamic_cli.call_async(request)

    def update_dynamic(self, prefix):
        request = UpdateDynamicFrames.Request()
        request.if_not_exists_add = False

        qx, qy, qz, qw = quaternion_from_roll(self.roll)

        for i in range(self.count):
            t = TransformStamped()
            t.header.frame_id = self.parent_frame
            t.child_frame_id = f"{prefix}_{i}"

            t.transform.translation.x = self.dx * i
            t.transform.translation.y = 1.0
            t.transform.translation.z = 1.0  # change z

            t.transform.rotation.x = qx
            t.transform.rotation.y = qy
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw

            request.frames_to_update.append(t)

        return self.update_dynamic_cli.call_async(request)

    def remove_dynamic(self, prefix):
        request = RemoveDynamicFrames.Request()

        for i in range(self.count):
            request.frame_names.append(f"{prefix}_{i}")

        return self.remove_dynamic_cli.call_async(request)

    # ------------------------------------------------------------------
    # Static Frames
    # ------------------------------------------------------------------

    def add_static(self):
        request = AddStaticFrames.Request()
        request.allow_override = True

        t = TransformStamped()
        t.header.frame_id = self.parent_frame
        t.child_frame_id = "static_demo_frame"

        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.5
        t.transform.translation.z = 0.0

        t.transform.rotation.w = 1.0

        request.frames.append(t)

        return self.add_static_cli.call_async(request)

    # ------------------------------------------------------------------
    # Pose Transform
    # ------------------------------------------------------------------

    def transform_pose(self, source_frame, target_frame):
        request = GetPosesInFrames.Request()

        pose = PoseStamped()
        pose.header.frame_id = source_frame
        pose.pose.position.x = 0.0
        pose.pose.position.y = 0.0
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = 1.0

        item = PoseStampedWithTarget()
        item.pose = pose
        item.target_frame = target_frame

        request.poses_to_transform.append(item)

        return self.get_pose_cli.call_async(request)


# ----------------------------------------------------------------------
# Helper
# ----------------------------------------------------------------------

def spin_print_and_wait(node, future, label, duration):
    rclpy.spin_until_future_complete(node, future)
    response = future.result()
    print(f"\n--- {label} ---")
    print(response)
    time.sleep(duration)


# ----------------------------------------------------------------------
# Main
# ----------------------------------------------------------------------

def main():
    rclpy.init()

    count = 10
    dx = 0.2
    roll = math.radians(15.0)

    if len(sys.argv) >= 2:
        count = int(sys.argv[1])
    if len(sys.argv) >= 3:
        dx = float(sys.argv[2])
    if len(sys.argv) >= 4:
        roll = math.radians(float(sys.argv[3]))

    tester = TransformManagerTester(count, dx, roll)

    prefix = "demo_frame"

    print("1. Add dynamic frames")
    spin_print_and_wait(tester, tester.add_dynamic(prefix), "Add Dynamic", 5.0)

    print("2. Update dynamic frames")
    spin_print_and_wait(tester, tester.update_dynamic(prefix), "Update Dynamic", 5.0)

    print("3. Add static frame")
    spin_print_and_wait(tester, tester.add_static(), "Add Static", 5.0)

    print("4. Transform pose")
    spin_print_and_wait(
        tester,
        tester.transform_pose(f"{prefix}_0", tester.parent_frame),
        "Transform Pose"
        , 5.0
    )

    print("5. Remove dynamic frames")
    spin_print_and_wait(tester, tester.remove_dynamic(prefix), "Remove Dynamic", 5.0)

    tester.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()