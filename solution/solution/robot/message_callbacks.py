# robot/message_callbacks.py

import math

from nav_msgs.msg import OccupancyGrid
from tf_transformations import euler_from_quaternion

from .config import (
    SCAN_FRONT,
    SCAN_LEFT,
    SCAN_RIGHT,
    SCAN_BACK,
    SCAN_THRESHOLD,
    SCAN_WARN_THRESHOLD
)


def publish_position_update(node, pose, robot_id, robot_radius, costmap_publisher):
    grid_msg = OccupancyGrid()
    grid_msg.header.frame_id = 'map'
    grid_msg.header.stamp = node.get_clock().now().to_msg()

    resolution = 0.05
    width = height = 40

    grid_msg.info.resolution = resolution
    grid_msg.info.width = width
    grid_msg.info.height = height

    grid_msg.info.origin.position.x = pose.position.x - (width * resolution) / 2
    grid_msg.info.origin.position.y = pose.position.y - (height * resolution) / 2
    grid_msg.info.origin.orientation = pose.orientation

    grid_msg.data = [-1] * (width * height)

    center_x = int(width / 2)
    center_y = int(height / 2)
    robot_radius = int(robot_radius / resolution)

    for i in range(-robot_radius, robot_radius + 1):
        for j in range(-robot_radius, robot_radius + 1):
            if i * i + j * j <= robot_radius * robot_radius:
                idx = (center_y + j) * width + (center_x + i)
                if 0 <= idx < len(grid_msg.data):
                    grid_msg.data[idx] = 100

    costmap_publisher.publish(grid_msg)
    node.get_logger().debug(f'Published position update for {robot_id}')


def item_callback(robot_id, msg):
    filtered_items = []
    for item in msg.data:
        if robot_id == 'robot2':
            if item.colour.upper() == 'GREEN':
                filtered_items.append(item)
        else:
            filtered_items.append(item)
    return filtered_items


def odom_callback(pose, initial_pose, msg):
    if initial_pose is None:
        initial_pose = msg.pose.pose

    pose = msg.pose.pose
    (roll, pitch, yaw) = euler_from_quaternion([
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w
    ])
    return pose, yaw, initial_pose


def scan_callback(items_data, msg):
    front_ranges = msg.ranges[315:360] + msg.ranges[0:45]
    left_ranges = msg.ranges[45:135]
    back_ranges = msg.ranges[135:225]
    right_ranges = msg.ranges[225:315]

    valid_front = [r for r in front_ranges if not math.isinf(r) and not math.isnan(r)]
    valid_left = [r for r in left_ranges if not math.isinf(r) and not math.isnan(r)]
    valid_right = [r for r in right_ranges if not math.isinf(r) and not math.isnan(r)]
    valid_back = [r for r in back_ranges if not math.isinf(r) and not math.isnan(r)]

    scan_triggered = [False] * 4
    min_left_dist = float('inf')
    min_right_dist = float('inf')
    front_distance = float('inf')

    if valid_front:
        min_front = min(valid_front)

        if len(items_data) > 0:
            min_front = float('inf')

        scan_triggered[SCAN_FRONT] = min_front < SCAN_THRESHOLD
        if min_front < SCAN_WARN_THRESHOLD:
            front_distance = min_front

    if valid_left:
        scan_triggered[SCAN_LEFT] = min(valid_left) < SCAN_THRESHOLD
        min_left_dist = min(valid_left)

    if valid_right:
        scan_triggered[SCAN_RIGHT] = min(valid_right) < SCAN_THRESHOLD
        min_right_dist = min(valid_right)

    if valid_back:
        scan_triggered[SCAN_BACK] = min(valid_back) < SCAN_THRESHOLD

    return scan_triggered, min_left_dist, min_right_dist, front_distance


def set_initial_pose(node, robot_id, initial_pose, pose, navigator):
    robot_initial_positions = {
        "robot1": {"x": -3.5, "y": 2.0},
        "robot2": {"x": -3.5, "y": 0.0},
        "robot3": {"x": -3.5, "y": -2.0}
    }
    frame_id = "map"
    if robot_id in robot_initial_positions:
        initial_pose.header.frame_id = frame_id
        initial_pose.header.stamp = navigator.get_clock().now().to_msg()
        initial_pose.pose.position.x = robot_initial_positions[robot_id]["x"]
        initial_pose.pose.position.y = robot_initial_positions[robot_id]["y"]
        initial_pose.pose.orientation.w = pose.orientation.w
        return initial_pose
    else:
        raise ValueError(f"Unknown robot ID: {robot_id}")
