#!/usr/bin/env python3
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.time import Time

import py_trees

from std_srvs.srv import Trigger, SetBool
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


# -----------------------------
# Shared interface to Spot ROS
# -----------------------------
@dataclass
class SpotIO:
    node: Node
    odom_topic: str
    cmd_vel_topic: str

    latest_odom: Optional[Odometry] = None

    def __post_init__(self):
        self.odom_sub = self.node.create_subscription(
            Odometry, self.odom_topic, self._odom_cb, 10
        )
        self.cmd_pub = self.node.create_publisher(Twist, self.cmd_vel_topic, 10)

        # Services used in this demo
        self.srv_claim = self.node.create_client(Trigger, "/spot/claim")
        self.srv_power_on = self.node.create_client(Trigger, "/spot/power_on")
        self.srv_stand = self.node.create_client(Trigger, "/spot/stand")
        self.srv_sit = self.node.create_client(Trigger, "/spot/sit")
        self.srv_allow_motion = self.node.create_client(SetBool, "/spot/allow_motion")

    def _odom_cb(self, msg: Odometry):
        self.latest_odom = msg

    def stop(self):
        t = Twist()
        self.cmd_pub.publish(t)


# -----------------------------
# BT leaf nodes
# -----------------------------
class TriggerService(py_trees.behaviour.Behaviour):
    """Non-blocking Trigger service call."""

    def __init__(self, name: str, io: SpotIO, client, timeout_sec: float = 10.0):
        super().__init__(name)
        self.io = io
        self.client = client
        self.timeout_sec = timeout_sec
        self._future = None
        self._t0: Optional[Time] = None

    def initialise(self):
        self._future = None
        self._t0 = self.io.node.get_clock().now()

    def update(self):
        assert self._t0 is not None
        elapsed = (self.io.node.get_clock().now() - self._t0).nanoseconds * 1e-9

        if not self.client.service_is_ready():
            if elapsed > self.timeout_sec:
                self.io.node.get_logger().error(f"{self.name}: service not available")
                return py_trees.common.Status.FAILURE
            return py_trees.common.Status.RUNNING

        if self._future is None:
            self._future = self.client.call_async(Trigger.Request())
            return py_trees.common.Status.RUNNING

        if not self._future.done():
            if elapsed > self.timeout_sec:
                self.io.node.get_logger().error(f"{self.name}: service call timeout")
                return py_trees.common.Status.FAILURE
            return py_trees.common.Status.RUNNING

        try:
            resp = self._future.result()
        except Exception as e:
            self.io.node.get_logger().error(f"{self.name}: exception: {e}")
            return py_trees.common.Status.FAILURE

        return py_trees.common.Status.SUCCESS if resp.success else py_trees.common.Status.FAILURE


class SetBoolService(py_trees.behaviour.Behaviour):
    """Non-blocking SetBool service call."""

    def __init__(self, name: str, io: SpotIO, client, value: bool, timeout_sec: float = 10.0):
        super().__init__(name)
        self.io = io
        self.client = client
        self.value = value
        self.timeout_sec = timeout_sec
        self._future = None
        self._t0: Optional[Time] = None

    def initialise(self):
        self._future = None
        self._t0 = self.io.node.get_clock().now()

    def update(self):
        assert self._t0 is not None
        elapsed = (self.io.node.get_clock().now() - self._t0).nanoseconds * 1e-9

        if not self.client.service_is_ready():
            if elapsed > self.timeout_sec:
                self.io.node.get_logger().error(f"{self.name}: service not available")
                return py_trees.common.Status.FAILURE
            return py_trees.common.Status.RUNNING

        if self._future is None:
            req = SetBool.Request()
            req.data = self.value
            self._future = self.client.call_async(req)
            return py_trees.common.Status.RUNNING

        if not self._future.done():
            if elapsed > self.timeout_sec:
                self.io.node.get_logger().error(f"{self.name}: service call timeout")
                return py_trees.common.Status.FAILURE
            return py_trees.common.Status.RUNNING

        try:
            resp = self._future.result()
        except Exception as e:
            self.io.node.get_logger().error(f"{self.name}: exception: {e}")
            return py_trees.common.Status.FAILURE

        return py_trees.common.Status.SUCCESS if resp.success else py_trees.common.Status.FAILURE


class DriveForwardDistance(py_trees.behaviour.Behaviour):
    """
    Drive forward until odometry translation reaches target distance from start.
    Publishes Twist on cmd_vel_topic and reads Odometry from odom_topic.
    """

    def __init__(self, name: str, io: SpotIO, target_m: float, speed_mps: float = 0.25, timeout_sec: float = 20.0):
        super().__init__(name)
        self.io = io
        self.target_m = float(target_m)
        self.speed_mps = float(speed_mps)
        self.timeout_sec = float(timeout_sec)

        self._start_xy = None
        self._t0: Optional[Time] = None

    def initialise(self):
        self._start_xy = None
        self._t0 = self.io.node.get_clock().now()

    def update(self):
        assert self._t0 is not None
        elapsed = (self.io.node.get_clock().now() - self._t0).nanoseconds * 1e-9
        if elapsed > self.timeout_sec:
            self.io.node.get_logger().error(f"{self.name}: timeout, stopping")
            self.io.stop()
            return py_trees.common.Status.FAILURE

        if self.io.latest_odom is None:
            return py_trees.common.Status.RUNNING

        p = self.io.latest_odom.pose.pose.position
        if self._start_xy is None:
            self._start_xy = (p.x, p.y)

        sx, sy = self._start_xy
        dist = math.hypot(p.x - sx, p.y - sy)

        if dist >= self.target_m:
            self.io.stop()
            self.io.node.get_logger().info(f"{self.name}: reached {dist:.2f} m, stopping")
            return py_trees.common.Status.SUCCESS

        cmd = Twist()
        cmd.linear.x = self.speed_mps
        self.io.cmd_pub.publish(cmd)
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        # Always try to stop on exit (success/failure/interruption)
        self.io.stop()


# -----------------------------
# Main: build and tick the tree
# -----------------------------
def create_tree(io: SpotIO, target_distance: float, speed: float) -> py_trees.trees.BehaviourTree:
    root = py_trees.composites.Sequence(name="stand_walk_sit", memory=True)

    root.add_children([
        TriggerService("claim_lease", io, io.srv_claim),
        TriggerService("power_on", io, io.srv_power_on),
        SetBoolService("allow_motion", io, io.srv_allow_motion, True),
        TriggerService("stand", io, io.srv_stand),
        DriveForwardDistance("walk_forward_1m", io, target_m=target_distance, speed_mps=speed),
        TriggerService("sit", io, io.srv_sit),
    ])

    return py_trees.trees.BehaviourTree(root)


def main():
    rclpy.init()
    node = Node("my_spot_bt_mission")

    odom_topic = node.declare_parameter("odom_topic", "/spot/odometry").value
    cmd_vel_topic = node.declare_parameter("cmd_vel_topic", "/cmd_vel").value
    target_distance = float(node.declare_parameter("target_distance", 1.0).value)
    speed = float(node.declare_parameter("speed", 0.25).value)

    io = SpotIO(node=node, odom_topic=odom_topic, cmd_vel_topic=cmd_vel_topic)
    tree = create_tree(io, target_distance, speed)

    node.get_logger().info(
        f"Mission: stand → drive {target_distance} m → sit | odom={odom_topic} cmd_vel={cmd_vel_topic}"
    )

    try:
        rate_hz = 20.0
        period = 1.0 / rate_hz
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)   # process odom + service futures
            tree.tick()

            status = tree.root.status
            if status in (py_trees.common.Status.SUCCESS, py_trees.common.Status.FAILURE):
                node.get_logger().info(f"Tree finished with status: {status}")
                break

            node.get_clock().sleep_for(rclpy.duration.Duration(seconds=period))
    except KeyboardInterrupt:
        node.get_logger().warn("Interrupted, stopping robot")
        io.stop()
    finally:
        io.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
