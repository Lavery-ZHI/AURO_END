import sys
import time
import copy

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.qos import QoSPresetProfiles
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from std_msgs.msg import Float32, Header
from geometry_msgs.msg import Twist, Pose, Point
from nav_msgs.msg import Odometry, OccupancyGrid, MapMetaData
from sensor_msgs.msg import LaserScan



from assessment_interfaces.msg import Item, ItemList
from auro_interfaces.msg import StringWithPose
from auro_interfaces.srv import ItemRequest

from tf_transformations import euler_from_quaternion
import angles
from enum import Enum
import random
import math
import numpy as np

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from copy import deepcopy
# -------------------常量定义----------------------------------------------------
LINEAR_VELOCITY = 0.6
ANGULAR_VELOCITY = 0.6

TURN_LEFT = 1
TURN_RIGHT = -1

SCAN_THRESHOLD = 0.4
SCAN_WARN_THRESHOLD = 0.45
SCAN_FRONT = 0
SCAN_LEFT = 1
SCAN_BACK = 2
SCAN_RIGHT = 3


class State(Enum):
    FORWARD = 0
    TURNING = 1
    COLLECTING = 2
    DROPPING = 3
    DELIVERING_NAV2 = 4


class RobotController(Node):

# -------------------主类和基本初始化----------------------------------------------------

    def __init__(self):
        super().__init__('robot_controller')

        self.executor = None

        self.state = State.FORWARD
        self.pose = Pose()
        self.previous_pose = Pose()
        self.yaw = 0.0
        self.previous_yaw = 0.0
        self.turn_angle = 0.0
        self.turn_direction = TURN_LEFT
        self.goal_distance = random.uniform(1.0, 2.0)

        self.initial_pose = None

        self.min_left_dist = None
        self.min_right_dist = None
        self.last_turn_direction = None
        self.scan_triggered = [False] * 4
        self.items = ItemList()
        self.item_held = False
        self.held_item_color = None

        self.scan_start_time = None
        self.scan_duration = 2.0

        self.declare_parameter('robot_id', 'robot1')
        self.robot_id = self.get_parameter('robot_id').value

        client_callback_group = MutuallyExclusiveCallbackGroup()
        timer_callback_group = MutuallyExclusiveCallbackGroup()

        self.navigator = BasicNavigator()

        self.initial_pose = PoseStamped()
        self.set_initial_pose()
        self.navigator.setInitialPose(self.initial_pose)
        self.navigator.waitUntilNav2Active()
        self.costmap_publisher = self.create_publisher(
            OccupancyGrid,
            'robot_position_grid',
            10
        )

        self.position_timer = self.create_timer(
            0.5,  # 2Hz更新频率
            self.publish_position_update,
            callback_group=timer_callback_group
        )


        self.robot_radius = 0.3

        self.declare_parameter('max_vel_x', 0.3)
        self.declare_parameter('min_vel_x', -0.3)
        self.declare_parameter('max_vel_theta', 1.0)
        self.declare_parameter('min_vel_theta', -1.0)



        self.pick_up_service = self.create_client(
            ItemRequest,
            '/pick_up_item',
            callback_group=client_callback_group
        )
        self.offload_service = self.create_client(
            ItemRequest,
            '/offload_item',
            callback_group=client_callback_group
        )

        self.item_subscriber = self.create_subscription(
            ItemList,
            'items',
            self.item_callback,
            10,
            callback_group=timer_callback_group
        )

        self.odom_subscriber = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10,
            callback_group=timer_callback_group
        )

        self.scan_subscriber = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            QoSPresetProfiles.SENSOR_DATA.value,
            callback_group=timer_callback_group
        )

        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )

        self.marker_publisher = self.create_publisher(
            StringWithPose,
            'marker_input',
            10,
            callback_group=timer_callback_group
        )

        self.timer_period = 0.1  # 10Hz
        self.timer = self.create_timer(
            self.timer_period,
            self.control_loop,
            callback_group=timer_callback_group
        )
        self.initial_navigation_done = False
        if self.robot_id == 'robot2':
            self.state = State.DELIVERING_NAV2

        self.zones = {
            'BOTTOM_RIGHT': {
                'color': 'RED',
                'x': -3.42,
                'y': -2.46,
                'target_yaw': math.pi / 20
            },
            'TOP_RIGHT': {
                'color': 'GREEN',
                'x': 2.37,
                'y': -2.5,
                'target_yaw': math.pi * 5 / 6
            },
            'TOP_LEFT': {
                'color': 'BLUE',
                'x': 2.30,
                'y': 2.40,
                'target_yaw': -3 * math.pi / 4
            },
            'BOTTOM_LEFT': {
                'color': 'BLACK',
                'x': -3.42,
                'y': 2.46,
                'target_yaw': -math.pi / 4
            }
        }
    def destroy_node(self):
        try:
            stop_msg = Twist()
            self.cmd_vel_publisher.publish(stop_msg)
        finally:
            super().destroy_node()
    def control_loop(self):

        marker_input = StringWithPose()
        marker_input.text = str(self.state)
        marker_input.pose = self.pose
        self.marker_publisher.publish(marker_input)

        if self.robot_id == 'robot2' and not self.initial_navigation_done:
            if self.state == State.DELIVERING_NAV2:
                self._handle_initial_navigation()
                return
        match self.state:
            case State.FORWARD:
                self._handle_forward_state()
            case State.TURNING:
                self._handle_turning_state()
            case State.COLLECTING:
                self._handle_collecting_state()
            case State.DROPPING:
                self._handle_attempt_offload()
            case State.DELIVERING_NAV2:
                self._handle_delivering_state()

# -------------------ROS2 消息处理相关----------------------------------------------------
    def publish_position_update(self):
        grid_msg = OccupancyGrid()
        grid_msg.header.frame_id = 'map'
        grid_msg.header.stamp = self.get_clock().now().to_msg()

        resolution = 0.05
        width = height = 40

        grid_msg.info.resolution = resolution
        grid_msg.info.width = width
        grid_msg.info.height = height

        grid_msg.info.origin.position.x = self.pose.position.x - (width * resolution) / 2
        grid_msg.info.origin.position.y = self.pose.position.y - (height * resolution) / 2
        grid_msg.info.origin.orientation = self.pose.orientation

        grid_msg.data = [-1] * (width * height)

        center_x = int(width / 2)
        center_y = int(height / 2)
        robot_radius = int(self.robot_radius / resolution)

        for i in range(-robot_radius, robot_radius + 1):
            for j in range(-robot_radius, robot_radius + 1):
                if i * i + j * j <= robot_radius * robot_radius:
                    idx = (center_y + j) * width + (center_x + i)
                    if 0 <= idx < len(grid_msg.data):
                        grid_msg.data[idx] = 100

        self.costmap_publisher.publish(grid_msg)
        self.get_logger().debug(f'Published position update for {self.robot_id}')
    def item_callback(self, msg):

        filtered_items = []

        for item in msg.data:
            if self.robot_id == 'robot2':
                if item.colour.upper() == 'GREEN':
                    filtered_items.append(item)
            else:
                filtered_items.append(item)

        self.items.data = filtered_items
    def odom_callback(self, msg):

        if self.initial_pose is None:
            self.initial_pose = msg.pose.pose


        self.pose = msg.pose.pose
        (roll, pitch, yaw) = euler_from_quaternion([
            self.pose.orientation.x,
            self.pose.orientation.y,
            self.pose.orientation.z,
            self.pose.orientation.w
        ])
        self.yaw = yaw
    def scan_callback(self, msg):
        front_ranges = msg.ranges[315:360] + msg.ranges[0:45]
        left_ranges = msg.ranges[45:135]
        back_ranges = msg.ranges[135:225]
        right_ranges = msg.ranges[225:315]

        valid_front = [r for r in front_ranges if not math.isinf(r) and not math.isnan(r)]
        valid_left = [r for r in left_ranges if not math.isinf(r) and not math.isnan(r)]
        valid_right = [r for r in right_ranges if not math.isinf(r) and not math.isnan(r)]
        valid_back = [r for r in back_ranges if not math.isinf(r) and not math.isnan(r)]

        if valid_front:
            min_front = min(valid_front)

            if len(self.items.data) > 0:
                min_front = float('inf')

            self.scan_triggered[SCAN_FRONT] = min_front < SCAN_THRESHOLD
            if min_front < SCAN_WARN_THRESHOLD:
                self.front_distance = min_front
            else:
                self.front_distance = float('inf')

        if valid_left:
            self.scan_triggered[SCAN_LEFT] = min(valid_left) < SCAN_THRESHOLD
            self.min_left_dist = min(valid_left)
        else:
            self.min_left_dist = float('inf')
        if valid_right:
            self.scan_triggered[SCAN_RIGHT] = min(valid_right) < SCAN_THRESHOLD
            self.min_right_dist = min(valid_right)
        else:
            self.min_right_dist = float('inf')
        if valid_back:
            self.scan_triggered[SCAN_BACK] = min(valid_back) < SCAN_THRESHOLD
    def set_initial_pose(self):

        robot_initial_positions = {
            "robot1": {"x": -3.5, "y": 2.0},
            "robot2": {"x": -3.5, "y": 0.0},
            "robot3": {"x": -3.5, "y": -2.0}
        }
        frame_id = "map"
        if self.robot_id in robot_initial_positions:
            self.initial_pose.header.frame_id = frame_id
            self.initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
            self.initial_pose.pose.position.x = robot_initial_positions[self.robot_id]["x"]
            self.initial_pose.pose.position.y = robot_initial_positions[self.robot_id]["y"]
            self.initial_pose.pose.orientation.w = self.pose.orientation.w

        else:
            raise ValueError(f"Unknown robot ID: {self.robot_id}")

# -------------------状态管理相关----------------------------------------------------
    def _handle_forward_state(self):
        if self.scan_triggered[SCAN_FRONT]:
            self._prepare_turn(90, 120)
            return

        if self.item_held:
            self.state = State.DELIVERING_NAV2
            return

        if len(self.items.data) > 0 and not self.item_held:

            nearest_item = self._compute_nearest_item(self.items.data)
            if nearest_item['distance'] < 2.0:
                if self._check_and_avoid_obstacles():
                    return
                self.state = State.COLLECTING
                return
        msg = Twist()
        msg.linear.x = LINEAR_VELOCITY
        self.cmd_vel_publisher.publish(msg)

        if self._check_and_avoid_obstacles():
            return
        self.state = State.COLLECTING
    def _handle_turning_state(self):
        msg = Twist()
        msg.angular.z = self.turn_direction * ANGULAR_VELOCITY
        self.cmd_vel_publisher.publish(msg)

        yaw_difference = angles.normalize_angle(self.yaw - self.previous_yaw)
        if math.fabs(yaw_difference) >= math.radians(self.turn_angle):
            self._complete_turn()
    def _handle_collecting_state(self):
        if len(self.items.data) == 0:
            self._handle_scanning()
            return

        if self._check_and_avoid_obstacles():
            return

        nearest_item = self._compute_nearest_item(self.items.data)

        item = nearest_item['item']
        distance = nearest_item['distance']

        heading_error = item.x / 320.0

        self.current_target_item = nearest_item['item']
        if distance <= 0.35:
            if self._check_and_avoid_obstacles():
                # 如果避障逻辑触发，则结束当前循环
                return
            self._attempt_pickup()
        else:
            self._approach_item(distance, heading_error)
    def _handle_delivering_state(self):

        if not self.item_held:
            self.state = State.FORWARD
            return

        target_zone = self.get_target_zone()
        if target_zone is None:
            self.state = State.FORWARD
            return


        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()

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


        self.navigator.goToPose(goal_pose)

        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback is not None and feedback.navigation_time.sec > 600:
                self.navigator.cancelNav()
                break
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.state = State.DROPPING
        elif result == TaskResult.CANCELED:
            self.state = State.DELIVERING_NAV2
        elif result == TaskResult.FAILED:
            self.state = State.FORWARD
    def _handle_initial_navigation(self):

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()

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

        self.navigator.goToPose(goal_pose)

        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback is not None and feedback.navigation_time.sec > 600:
                self.navigator.cancelNav()
                break

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.initial_navigation_done = True
            self.state = State.FORWARD
        elif result == TaskResult.CANCELED:
            self.state = State.DELIVERING_NAV2
        elif result == TaskResult.FAILED:
            self.state = State.DELIVERING_NAV2
    def _handle_scanning(self):

        if self._check_and_avoid_obstacles():
            return

        current_time = self.get_clock().now()
        if self.scan_start_time is None:
            self.scan_start_time = current_time

        if (current_time - self.scan_start_time).nanoseconds < self.scan_duration * 1e9:
            msg = Twist()
            msg.angular.z = 0.3
            self.cmd_vel_publisher.publish(msg)
        else:
            self.scan_start_time = None
            self.state = State.FORWARD
# -------------------物品操作相关----------------------------------------------------
    def _compute_nearest_item(self, items):
        if not items:
            return None, None

        items_with_distance = []
        for item in items:
            distance = 32.4 * float(item.diameter) ** -0.75
            items_with_distance.append({
                'item': item,
                'distance': distance,
                'color': item.colour,
                'position': {'x': item.x, 'y': item.y}  # 添加位置信息
            })

        items_with_distance.sort(key=lambda x: x['distance'])
        nearest_item = items_with_distance[0]


        return nearest_item
    def _approach_item(self, distance, heading_error):
        msg = Twist()
        min_speed = 0.1

        if hasattr(self, 'front_distance') and self.front_distance < SCAN_WARN_THRESHOLD:
            speed_factor = max(0.35, self.front_distance / SCAN_WARN_THRESHOLD)  # 降低最小因子到 0.35
            msg.linear.x = max(min_speed, min(0.3, 0.4 * distance) * speed_factor)  # 调整最大速度到 0.3
        else:
            msg.linear.x = max(min_speed, min(0.3, 0.4 * distance))  # 最大速度设为 0.3

        msg.angular.z = 0.8 * heading_error

        self.cmd_vel_publisher.publish(msg)
    def _attempt_pickup(self):

        stop_msg = Twist()
        self.cmd_vel_publisher.publish(stop_msg)

        if self.current_target_item is not None:
            target_color = self.current_target_item.colour.upper()
        else:
            target_color = "UNKNOWN"

        rqt = ItemRequest.Request()
        rqt.robot_id = self.robot_id

        try:
            future = self.pick_up_service.call_async(rqt)
            self.executor.spin_until_future_complete(future)
            response = future.result()

            if response.success:
                self.item_held = True
                self.held_item_color = target_color
                self.previous_pose = self.pose
                self.goal_distance = random.uniform(1.0, 2.0)
                self.state = State.FORWARD
            else:
                self._handle_pickup_failure()

        except Exception as e:
            self.state = State.FORWARD
    def _handle_pickup_failure(self):
        backup_msg = Twist()
        backup_msg.linear.x = -0.1
        self.cmd_vel_publisher.publish(backup_msg)
        self.state = State.FORWARD
    def _handle_attempt_offload(self):
        stop_msg = Twist()
        self.cmd_vel_publisher.publish(stop_msg)
        rqt = ItemRequest.Request()
        rqt.robot_id = self.robot_id

        try:
            future = self.offload_service.call_async(rqt)
            self.executor.spin_until_future_complete(future)
            response = future.result()

            if response.success:
                self.item_held = False
                self.held_item_color = None
                self.state = State.FORWARD

            else:
                self._handle_offload_failure()

        except Exception as e:
            self._handle_offload_failure()
    def _handle_offload_failure(self):
        backup_msg = Twist()
        backup_msg.linear.x = -0.1
        self.cmd_vel_publisher.publish(backup_msg)
        time.sleep(0.5)
        self.previous_yaw = self.yaw
        self._prepare_turn(90, 120)
        self.previous_pose = self.pose
        self.state = State.DROPPING
    def get_target_zone(self):

        for zone_name, zone_info in self.zones.items():
            if zone_info.get('color') == self.held_item_color:
                return zone_info
        return None
# -------------------运动控制与避障相关----------------------------------------------------
    def _complete_turn(self):
        self.previous_pose = self.pose
        self.goal_distance = random.uniform(1.0, 2.0)
        self.state = State.FORWARD
    def _prepare_turn(self, min_angle, max_angle, force_direction=None):
        self.previous_yaw = self.yaw
        self.state = State.TURNING
        self.turn_angle = random.uniform(min_angle, max_angle)
        if force_direction is not None:
            self.turn_direction = force_direction
        else:
            left_dist = self.min_left_dist if not math.isinf(self.min_left_dist) else 999.0
            right_dist = self.min_right_dist if not math.isinf(self.min_right_dist) else 999.0

            if left_dist > right_dist + 0.1:
                chosen_dir = TURN_LEFT
            elif right_dist > left_dist + 0.1:
                chosen_dir = TURN_RIGHT
            else:
                chosen_dir = self.last_turn_direction or TURN_LEFT
            self.turn_direction = chosen_dir
        self.last_turn_direction = self.turn_direction
    def _check_and_avoid_obstacles(self):
        front_blocked = self.scan_triggered[SCAN_FRONT]
        left_blocked = self.scan_triggered[SCAN_LEFT]
        right_blocked = self.scan_triggered[SCAN_RIGHT]
        back_blocked = self.scan_triggered[SCAN_BACK]

        if not any([front_blocked, left_blocked, right_blocked, back_blocked]):
            return False

        if front_blocked:
            last_turn = getattr(self, 'last_turn', None)

            if not left_blocked and not right_blocked:
                direction = TURN_LEFT if last_turn != TURN_LEFT else TURN_RIGHT
                self.last_turn = direction
                self._smooth_turn(direction)
            elif not left_blocked:
                self._smooth_turn(TURN_LEFT)
            elif not right_blocked:
                self._smooth_turn(TURN_RIGHT)
            else:
                self._emergency_maneuver()

        elif left_blocked or right_blocked:
            self._handle_side_obstacles(left_blocked, right_blocked)

        return True
    def _handle_side_obstacles(self, left_blocked, right_blocked):
        if left_blocked:
            self._smooth_turn(TURN_RIGHT)
        elif right_blocked:
            self._smooth_turn(TURN_LEFT)
        else:
            self._reduce_speed()
    def _reduce_speed(self, reduction_factor=0.5):

        try:
            current_speed = getattr(self, 0.2, LINEAR_VELOCITY)
            reduced_speed = current_speed * reduction_factor

            if reduced_speed < 0.05:
                reduced_speed = 0.05

            msg = Twist()
            msg.linear.x = reduced_speed
            self.cmd_vel_publisher.publish(msg)

            self.current_speed = reduced_speed
        except Exception as e:
            self.get_logger().error(f"减速时发生错误: {str(e)}")
    def _smooth_turn(self, direction, base_speed=0.2):
        msg = Twist()
        if hasattr(self, 'front_distance'):
            turn_speed = min(0.5, max(0.2, 1.0 - self.front_distance / SCAN_WARN_THRESHOLD))
        else:
            turn_speed = 0.3

        msg.angular.z = turn_speed * direction
        msg.linear.x = base_speed * (1.0 - abs(msg.angular.z))
        self.cmd_vel_publisher.publish(msg)
    def _emergency_maneuver(self):
        self.get_logger().warn("执行紧急避障操作")

        backup_msg = Twist()
        backup_msg.linear.x = -0.15
        self.cmd_vel_publisher.publish(backup_msg)

        time.sleep(0.5)

        if not any(self.scan_triggered):
            return
        self._prepare_turn(170, 180)


def main(args=None):

    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)

    node = RobotController()
    executor = MultiThreadedExecutor()

    node.executor = executor

    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("收到键盘中断信号，正在关闭节点...")
    except ExternalShutdownException:
        node.get_logger().error("收到外部关闭信号")
        sys.exit(1)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main
