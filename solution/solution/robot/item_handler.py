# robot/item_handler.py

import random
import time

from auro_interfaces.srv import ItemRequest
from geometry_msgs.msg import Twist

from .config import (
    SCAN_WARN_THRESHOLD,
    State
)
from .motion_control import prepare_turn


def compute_nearest_item(controller, items):
    if not items:
        return None

    items_with_distance = []
    for item in items:
        distance = 32.4 * float(item.diameter) ** -0.75
        items_with_distance.append({
            'item': item,
            'distance': distance,
            'color': item.colour,
            'position': {'x': item.x, 'y': item.y}
        })

    items_with_distance.sort(key=lambda x: x['distance'])
    nearest_item = items_with_distance[0]

    return nearest_item


def approach_item(controller, distance, heading_error):
    msg = Twist()
    min_speed = 0.1

    if hasattr(controller, 'front_distance') and controller.front_distance < SCAN_WARN_THRESHOLD:
        speed_factor = max(0.40, controller.front_distance / SCAN_WARN_THRESHOLD)
        msg.linear.x = max(min_speed, min(0.3, 0.4 * distance) * speed_factor)
    else:
        msg.linear.x = max(min_speed, min(0.3, 0.4 * distance))

    msg.angular.z = 0.8 * heading_error

    controller.cmd_vel_publisher.publish(msg)


def attempt_pickup(controller):
    stop_msg = Twist()
    controller.cmd_vel_publisher.publish(stop_msg)

    if controller.current_target_item is not None:
        target_color = controller.current_target_item.colour.upper()
    else:
        target_color = "UNKNOWN"

    rqt = ItemRequest.Request()
    rqt.robot_id = controller.robot_id

    try:
        future = controller.pick_up_service.call_async(rqt)
        controller.executor.spin_until_future_complete(future)
        response = future.result()

        if response.success:
            controller.item_held = True
            controller.held_item_color = target_color
            controller.previous_pose = controller.pose
            controller.goal_distance = random.uniform(1.0, 2.0)
            controller.state = State.FORWARD
        else:
            handle_pickup_failure(controller)

    except Exception as e:
        controller.get_logger().error(f"Pickup failed: {str(e)}")
        controller.state = State.FORWARD


def handle_pickup_failure(controller):
    backup_msg = Twist()
    backup_msg.linear.x = -0.1
    controller.cmd_vel_publisher.publish(backup_msg)
    controller.state = State.FORWARD


def attempt_offload(controller):
    stop_msg = Twist()
    controller.cmd_vel_publisher.publish(stop_msg)
    rqt = ItemRequest.Request()
    rqt.robot_id = controller.robot_id

    try:
        future = controller.offload_service.call_async(rqt)
        controller.executor.spin_until_future_complete(future)
        response = future.result()

        if response.success:
            controller.item_held = False
            controller.held_item_color = None
            controller.state = State.FORWARD

        else:
            handle_offload_failure(controller)

    except Exception as e:
        controller.get_logger().error(f"Offload failed: {str(e)}")
        handle_offload_failure(controller)


def handle_offload_failure(controller):
    backup_msg = Twist()
    backup_msg.linear.x = -0.1
    controller.cmd_vel_publisher.publish(backup_msg)
    time.sleep(0.5)
    controller.previous_yaw = controller.yaw
    prepare_turn(controller, 90, 120)
    controller.previous_pose = controller.pose
    controller.state = State.DROPPING


def get_target_zone(controller):
    for zone_name, zone_info in controller.zones.items():
        if zone_info.get('color') == controller.held_item_color:
            return zone_info
    return None
