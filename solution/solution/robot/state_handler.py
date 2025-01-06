import math

from geometry_msgs.msg import Twist, PoseStamped
from nav2_simple_commander.robot_navigator import TaskResult

from .config import (
    LINEAR_VELOCITY,
    ANGULAR_VELOCITY,
    SCAN_FRONT,
    State
)
from .item_handler import (
    compute_nearest_item,
    approach_item,
    attempt_pickup,
    attempt_offload,
    get_target_zone
)
from .motion_control import (
    prepare_turn,
    complete_turn,
    check_and_avoid_obstacles
)


def handle_forward_state(controller):
    if controller.scan_triggered[SCAN_FRONT]:
        prepare_turn(controller, 90, 120)
        return

    if controller.item_held:
        controller.state = State.DELIVERING_NAV2
        return

    if len(controller.items.data) > 0 and not controller.item_held:
        nearest_item = compute_nearest_item(controller, controller.items.data)
        if nearest_item['distance'] < 2.0:
            if check_and_avoid_obstacles(controller):
                return
            controller.state = State.COLLECTING
            return
    msg = Twist()
    msg.linear.x = LINEAR_VELOCITY
    controller.cmd_vel_publisher.publish(msg)

    if check_and_avoid_obstacles(controller):
        return
    controller.state = State.COLLECTING


def handle_turning_state(controller):
    msg = Twist()
    msg.angular.z = controller.turn_direction * ANGULAR_VELOCITY
    controller.cmd_vel_publisher.publish(msg)

    yaw_difference = math.fabs(controller.yaw - controller.previous_yaw)
    if yaw_difference >= math.radians(controller.turn_angle):
        complete_turn(controller)


def handle_collecting_state(controller):
    if len(controller.items.data) == 0:
        handle_scanning(controller)
        return

    if check_and_avoid_obstacles(controller):
        return

    nearest_item = compute_nearest_item(controller, controller.items.data)
    item = nearest_item['item']
    distance = nearest_item['distance']
    heading_error = item.x / 320.0

    controller.current_target_item = nearest_item['item']
    if distance <= 0.35:
        if check_and_avoid_obstacles(controller):
            return
        attempt_pickup(controller)
    else:
        approach_item(controller, distance, heading_error)


def handle_delivering_state(controller):
    if not controller.item_held:
        controller.state = State.FORWARD
        return

    target_zone = get_target_zone(controller)
    if target_zone is None:
        controller.state = State.FORWARD
        return

    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = controller.navigator.get_clock().now().to_msg()

    goal_pose.pose.position.x = target_zone['x']
    goal_pose.pose.position.y = target_zone['y']
    goal_pose.pose.position.z = 0.0

    yaw = target_zone['target_yaw']
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    goal_pose.pose.orientation.x = 0.0
    goal_pose.pose.orientation.y = 0.0
    goal_pose.pose.orientation.z = sy
    goal_pose.pose.orientation.w = cy

    controller.navigator.goToPose(goal_pose)

    while not controller.navigator.isTaskComplete():
        feedback = controller.navigator.getFeedback()
        if feedback is not None and feedback.navigation_time.sec > 600:
            controller.navigator.cancelNav()
            break

    result = controller.navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        controller.state = State.DROPPING
    elif result == TaskResult.CANCELED:
        controller.state = State.DELIVERING_NAV2
    elif result == TaskResult.FAILED:
        controller.state = State.FORWARD


def handle_initial_navigation(controller):
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = controller.navigator.get_clock().now().to_msg()

    goal_pose.pose.position.x = 0.0
    goal_pose.pose.position.y = 0.0
    goal_pose.pose.position.z = 0.0

    yaw = 0.0
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    goal_pose.pose.orientation.x = 0.0
    goal_pose.pose.orientation.y = 0.0
    goal_pose.pose.orientation.z = sy
    goal_pose.pose.orientation.w = cy

    controller.navigator.goToPose(goal_pose)

    while not controller.navigator.isTaskComplete():
        feedback = controller.navigator.getFeedback()
        if feedback is not None and feedback.navigation_time.sec > 600:
            controller.navigator.cancelNav()
            break

    result = controller.navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        controller.initial_navigation_done = True
        controller.state = State.FORWARD
    elif result == TaskResult.CANCELED:
        controller.state = State.DELIVERING_NAV2
    elif result == TaskResult.FAILED:
        controller.state = State.DELIVERING_NAV2


def handle_scanning(controller):
    if check_and_avoid_obstacles(controller):
        return

    current_time = controller.get_clock().now()
    if controller.scan_start_time is None:
        controller.scan_start_time = current_time

    if (current_time - controller.scan_start_time).nanoseconds < controller.scan_duration * 1e9:
        msg = Twist()
        msg.angular.z = 0.3
        controller.cmd_vel_publisher.publish(msg)
    else:
        controller.scan_start_time = None
        controller.state = State.FORWARD


def handle_dropping_state(controller):
    attempt_offload(controller)
