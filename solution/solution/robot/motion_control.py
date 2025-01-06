import math
import time

from geometry_msgs.msg import Twist

import math
import time

from geometry_msgs.msg import Twist

from .config import (
    TURN_LEFT,
    TURN_RIGHT,
    LINEAR_VELOCITY,
    SCAN_WARN_THRESHOLD,
    SCAN_FRONT,
    SCAN_LEFT,
    SCAN_BACK,
    SCAN_RIGHT,
    State
)


def complete_turn(controller):
    controller.previous_pose = controller.pose
    controller.goal_distance = random.uniform(1.0, 2.0)
    controller.state = controller.State.FORWARD


def prepare_turn(controller, min_angle, max_angle, force_direction=None):
    controller.previous_yaw = controller.yaw
    controller.state = controller.State.TURNING
    controller.turn_angle = random.uniform(min_angle, max_angle)
    if force_direction is not None:
        controller.turn_direction = force_direction
    else:
        left_dist = controller.min_left_dist if not math.isinf(controller.min_left_dist) else 999.0
        right_dist = controller.min_right_dist if not math.isinf(controller.min_right_dist) else 999.0

        if left_dist > right_dist + 0.1:
            chosen_dir = TURN_LEFT
        elif right_dist > left_dist + 0.1:
            chosen_dir = TURN_RIGHT
        else:
            chosen_dir = controller.last_turn_direction or TURN_LEFT
        controller.turn_direction = chosen_dir
    controller.last_turn_direction = controller.turn_direction


def check_and_avoid_obstacles(controller):
    front_blocked = controller.scan_triggered[SCAN_FRONT]
    left_blocked = controller.scan_triggered[SCAN_LEFT]
    right_blocked = controller.scan_triggered[SCAN_RIGHT]
    back_blocked = controller.scan_triggered[SCAN_BACK]

    if not any([front_blocked, left_blocked, right_blocked, back_blocked]):
        return False

    if front_blocked:
        last_turn = getattr(controller, 'last_turn', None)

        if not left_blocked and not right_blocked:
            direction = TURN_LEFT if last_turn != TURN_LEFT else TURN_RIGHT
            controller.last_turn = direction
            smooth_turn(controller, direction)
        elif not left_blocked:
            smooth_turn(controller, TURN_LEFT)
        elif not right_blocked:
            smooth_turn(controller, TURN_RIGHT)
        else:
            emergency_maneuver(controller)

    elif left_blocked or right_blocked:
        handle_side_obstacles(controller, left_blocked, right_blocked)

    return True


def handle_side_obstacles(controller, left_blocked, right_blocked):
    if left_blocked:
        smooth_turn(controller, TURN_RIGHT)
    elif right_blocked:
        smooth_turn(controller, TURN_LEFT)
    else:
        reduce_speed(controller)


def reduce_speed(controller, reduction_factor=0.5):
    try:
        current_speed = getattr(controller, 'current_speed', LINEAR_VELOCITY)
        reduced_speed = current_speed * reduction_factor

        if reduced_speed < 0.05:
            reduced_speed = 0.05

        msg = Twist()
        msg.linear.x = reduced_speed
        controller.cmd_vel_publisher.publish(msg)

        controller.current_speed = reduced_speed
    except Exception as e:
        controller.get_logger().error(f"减速时发生错误: {str(e)}")


def smooth_turn(controller, direction, base_speed=0.2):
    msg = Twist()
    if hasattr(controller, 'front_distance'):
        turn_speed = min(0.5, max(0.2, 1.0 - controller.front_distance / SCAN_WARN_THRESHOLD))
    else:
        turn_speed = 0.3

    msg.angular.z = turn_speed * direction
    msg.linear.x = base_speed * (1.0 - abs(msg.angular.z))
    controller.cmd_vel_publisher.publish(msg)


def emergency_maneuver(controller):
    controller.get_logger().warn("执行紧急避障操作")

    backup_msg = Twist()
    backup_msg.linear.x = -0.15
    controller.cmd_vel_publisher.publish(backup_msg)

    time.sleep(0.5)

    if not any(controller.scan_triggered):
        return
    prepare_turn(controller, 170, 180)
