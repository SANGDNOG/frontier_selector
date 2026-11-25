#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Based on Shimmer Frontier-Exploration, commit 008585b13ed0446f1dfed7c4a5e2575ee468100c.
# Modified to select frontier goals by estimated travel time.

import math
import time
from types import SimpleNamespace

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseArray, PoseStamped
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformException, TransformListener

from .continuous_timing import (
    InvalidMotionState, InvalidTimingPath, connect_start, validate_motion_state,
)
from .continuous_timing_v3 import evaluate_path


TIME_COST = {
    'A': 'time_distance_v3_s',
    'B': 'time_turn_v3_s',
    'C': 'time_accel_v3_s',
}
ENDPOINT_TOLERANCE_M = 0.20


def yaw_of(q):
    length = math.hypot(q.x, q.y, q.z, q.w)
    if not math.isfinite(length) or length == 0.0:
        return math.nan
    x, y, z, w = (value / length for value in (q.x, q.y, q.z, q.w))
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y ** 2 + z ** 2))


def stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


class FrontierSelector(Node):
    def __init__(self):
        super().__init__('frontier_selector')
        self.frontier_list = None
        self.current_goal_active = False
        self.grid = None
        self.odometry = None
        self.batch = None
        self.retry_at = 0.0
        self.selected_frontiers = None
        self.min_goal_distance = 0.5

        strategy = self.declare_parameter('time_strategy', 'C').value
        if strategy not in TIME_COST:
            raise ValueError('time_strategy must be A, B or C')
        self.cost_key = TIME_COST[strategy]
        self.planner_id = self.declare_parameter('planner_id', 'GridBased').value

        self.create_subscription(PoseArray, '/frontier_list', self.frontiers_callback, 10)
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, map_qos)
        self.create_subscription(Odometry, '/odom', self.odometry_callback, qos_profile_sensor_data)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.planner = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')
        self.reached_pub = self.create_publisher(Bool, '/frontier_reached', 10)
        self.selection_timer = self.create_timer(
            0.05, self.update_selection, clock=Clock(clock_type=ClockType.STEADY_TIME),
        )

    def map_callback(self, msg):
        if msg.header.frame_id != 'map':
            self.grid = None
            return
        info = msg.info
        self.grid = SimpleNamespace(
            width=info.width, height=info.height, resolution=info.resolution,
            data=tuple(msg.data), origin_x=info.origin.position.x,
            origin_y=info.origin.position.y, origin_yaw=yaw_of(info.origin.orientation),
        )

    def odometry_callback(self, msg):
        self.odometry = msg

    def get_robot_pose(self, body_frame):
        try:
            transform = self.tf_buffer.lookup_transform('map', body_frame, rclpy.time.Time())
        except TransformException:
            return None
        position = transform.transform.translation
        return position.x, position.y, yaw_of(transform.transform.rotation), transform

    def motion_snapshot(self):
        msg = self.odometry
        if msg is None or msg.header.frame_id != 'odom':
            return None
        if msg.child_frame_id not in ('base_link', 'base_footprint'):
            return None
        pose = self.get_robot_pose(msg.child_frame_id)
        if pose is None:
            return None
        x, y, yaw, transform = pose
        now = self.get_clock().now().nanoseconds
        odom_stamp = stamp_ns(msg.header.stamp)
        pose_stamp = stamp_ns(transform.header.stamp)
        if (min(odom_stamp, pose_stamp) <= 0
                or not 0 <= now - odom_stamp <= 500_000_000
                or not 0 <= now - pose_stamp <= 1_000_000_000
                or abs(odom_stamp - pose_stamp) > 500_000_000):
            return None
        v, w = msg.twist.twist.linear, msg.twist.twist.angular
        if not all(map(math.isfinite, (x, y, yaw, v.x, v.y, v.z, w.x, w.y, w.z))):
            return None
        if math.hypot(v.y, v.z) > .01 or math.hypot(w.x, w.y) > .01:
            return None
        try:
            validate_motion_state(v.x, w.z)
        except InvalidMotionState:
            return None
        start = self.goal_pose(x, y)
        start.pose.orientation = transform.transform.rotation
        return SimpleNamespace(xy=(x, y), yaw=yaw, v=v.x, w=w.z, start=start,
                               body_frame=msg.child_frame_id, odom_stamp=odom_stamp,
                               pose_stamp=pose_stamp)

    def goal_pose(self, x, y):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.w = 1.0
        return pose

    def frontiers_callback(self, msg):
        if msg.header.frame_id == 'map':
            self.frontier_list = tuple((p.position.x, p.position.y) for p in msg.poses)

    def update_selection(self):
        now = time.monotonic()
        if self.batch is not None:
            self.poll_plan(now)
            return
        if self.current_goal_active or not self.frontier_list or now < self.retry_at:
            return
        if self.grid is None or not self.planner.server_is_ready() or not self.nav_client.server_is_ready():
            return
        motion = self.motion_snapshot()
        if motion is None:
            return
        points = [point for point in self.frontier_list if all(map(math.isfinite, point))]
        candidates = [point for point in points if math.dist(motion.xy, point) >= self.min_goal_distance]
        if not candidates:
            if points:
                # Shimmer uses this topic to request new detection after a close-goal skip.
                self.frontier_list = None
                self.retry_at = now + 0.5
                self.reached_pub.publish(Bool(data=True))
            return
        self.batch = SimpleNamespace(
            frontiers=self.frontier_list, candidates=iter(candidates), motion=motion,
            grid=self.grid, deadline=now + 8.0, request=None, result=None, handle=None,
            best_point=None, best_cost=math.inf,
        )
        self.request_path()

    def request_path(self):
        batch = self.batch
        now = time.monotonic()
        if now >= batch.deadline:
            self.finish_batch()
            return
        batch.point = next(batch.candidates, None)
        if batch.point is None:
            self.finish_batch()
            return
        request = ComputePathToPose.Goal()
        request.start = batch.motion.start
        request.goal = self.goal_pose(*batch.point)
        request.use_start = True
        request.planner_id = self.planner_id
        batch.request_deadline = min(batch.deadline, now + 1.0)
        batch.expired = batch.cancel_sent = False
        try:
            batch.request = self.planner.send_goal_async(request)
        except Exception as exc:
            self.stop_selection(exc)
        batch.handle = batch.result = None

    def stop_selection(self, error):
        # A failed action exchange can leave a goal active on the server.
        self.selection_timer.cancel()
        self.get_logger().error(f'Action state is unknown. Restart the selector after checking Nav2: {error}')

    def poll_plan(self, now):
        batch = self.batch
        if not batch.expired and now >= batch.request_deadline:
            batch.expired = True
            self.get_logger().warn('Frontier planning exceeded its time budget')
        if batch.handle is None:
            if not batch.request.done():
                return
            try:
                handle = batch.request.result()
            except Exception as exc:
                self.stop_selection(exc)
                return
            if not handle.accepted:
                self.request_path()
                return
            batch.handle = handle
            try:
                batch.result = handle.get_result_async()
            except Exception as exc:
                self.stop_selection(exc)
                return

        if not batch.result.done():
            if batch.expired and not batch.cancel_sent:
                batch.cancel_sent = True
                try:
                    batch.handle.cancel_goal_async()
                except Exception as exc:
                    self.get_logger().error(f'Planner cancellation failed, waiting for its result: {exc}')
            return
        try:
            response = batch.result.result()
        except Exception as exc:
            self.stop_selection(exc)
            return
        if response.status not in (
                GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_ABORTED):
            self.stop_selection(f'planner status {response.status}')
            return
        if not batch.expired and response.status == GoalStatus.STATUS_SUCCEEDED:
            self.score_frontier(response.result.path)
        self.request_path()

    def score_frontier(self, path):
        batch = self.batch
        if path.header.frame_id != 'map' or any(
                pose.header.frame_id not in ('', 'map') for pose in path.poses):
            return
        points = [(pose.pose.position.x, pose.pose.position.y) for pose in path.poses]
        if not points or math.dist(points[-1], batch.point) > ENDPOINT_TOLERANCE_M:
            return
        motion = batch.motion
        try:
            points, connector = connect_start(
                points, motion.xy, batch.grid, max_deviation_m=ENDPOINT_TOLERANCE_M,
            )
            speed, _ = validate_motion_state(motion.v, motion.w)
            if connector['clearance_basis'] == 'bounded_planner_start_proxy' and speed > 0.0:
                return
            cost = evaluate_path(points, motion.yaw, 0.0, batch.grid, motion.v, motion.w,
                                 goal_xy=batch.point)[self.cost_key]
        except (InvalidMotionState, InvalidTimingPath):
            return
        if math.isfinite(cost) and cost < batch.best_cost:
            batch.best_point, batch.best_cost = batch.point, cost

    def finish_batch(self):
        batch, self.batch = self.batch, None
        self.retry_at = time.monotonic() + 0.5
        if (batch.best_point is None or self.grid is None
                or self.frontier_list != batch.frontiers):
            return
        current = self.motion_snapshot()
        start = batch.motion
        if (current is None or current.body_frame != start.body_frame
                or current.odom_stamp < start.odom_stamp or current.pose_stamp < start.pose_stamp
                or math.dist(current.xy, start.xy) > .03
                or abs(math.remainder(current.yaw - start.yaw, math.tau)) > .05
                or abs(current.v - start.v) > .03 or abs(current.w - start.w) > .05
                or not self.nav_client.server_is_ready()):
            return
        goal = NavigateToPose.Goal()
        goal.pose = self.goal_pose(*batch.best_point)
        self.selected_frontiers = tuple(point for point in batch.frontiers if point != batch.best_point)
        self.frontier_list = None
        self.current_goal_active = True
        try:
            future = self.nav_client.send_goal_async(goal)
        except Exception as exc:
            self.stop_selection(exc)
            return
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        try:
            goal_handle = future.result()
            if goal_handle.accepted:
                result_future = goal_handle.get_result_async()
        except Exception as exc:
            self.stop_selection(exc)
            return
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected by Nav2!")
            self.navigation_finished(succeeded=False)
            return
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        try:
            status = future.result().status
        except Exception as exc:
            self.stop_selection(exc)
            return
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Goal succeeded!")
            self.navigation_finished(succeeded=True)
        elif status in (GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_ABORTED):
            self.get_logger().warn(f"Goal ended with status {status}")
            self.navigation_finished(succeeded=False)
        else:
            self.stop_selection(f'navigation status {status}')

    def navigation_finished(self, succeeded):
        self.current_goal_active = False
        if succeeded:
            self.frontier_list = None
            self.reached_pub.publish(Bool(data=True))
        elif self.frontier_list is None:
            self.frontier_list = self.selected_frontiers
        self.selected_frontiers = None


def main(args=None):
    rclpy.init(args=args)

    node = FrontierSelector()
    rclpy.spin(node)

    rclpy.shutdown()
