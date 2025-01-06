import random
import sys

import rclpy
from assessment_interfaces.msg import ItemList
from auro_interfaces.msg import StringWithPose
from auro_interfaces.srv import ItemRequest
from geometry_msgs.msg import Twist, Pose, PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
from nav_msgs.msg import Odometry, OccupancyGrid
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import LaserScan

from .robot.config import (TURN_LEFT, TURN_RIGHT, State, ZONES, LINEAR_VELOCITY, ANGULAR_VELOCITY, SCAN_THRESHOLD,
                           SCAN_WARN_THRESHOLD, SCAN_FRONT, SCAN_LEFT, SCAN_BACK, SCAN_RIGHT)
from .robot.message_callbacks import (publish_position_update, item_callback, odom_callback, scan_callback,
                                      set_initial_pose)
from .robot.state_handler import (handle_forward_state, handle_turning_state, handle_collecting_state,
                                  handle_delivering_state, handle_dropping_state, handle_initial_navigation)
from .robot.motion_control import (prepare_turn, complete_turn, check_and_avoid_obstacles)
from .robot.item_handler import (compute_nearest_item, approach_item, attempt_pickup, attempt_offload, get_target_zone)


class RobotController(Node):
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
            0.5,
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

        self.zones = ZONES

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
                handle_initial_navigation(self)
                return

        match self.state:
            case State.FORWARD:
                handle_forward_state(self)
            case State.TURNING:
                handle_turning_state(self)
            case State.COLLECTING:
                handle_collecting_state(self)
            case State.DROPPING:
                handle_dropping_state(self)
            case State.DELIVERING_NAV2:
                handle_delivering_state(self)

    def publish_position_update(self):
        publish_position_update(
            self,
            self.pose,
            self.robot_id,
            self.robot_radius,
            self.costmap_publisher
        )

    def item_callback(self, msg):
        self.items.data = item_callback(self.robot_id, msg)

    def odom_callback(self, msg):
        self.pose, self.yaw, self.initial_pose = odom_callback(
            self.pose,
            self.initial_pose,
            msg
        )

    def scan_callback(self, msg):
        self.scan_triggered, self.min_left_dist, self.min_right_dist, self.front_distance = scan_callback(
            self.items.data,
            msg
        )

    def set_initial_pose(self):
        self.initial_pose = set_initial_pose(
            self,
            self.robot_id,
            self.initial_pose,
            self.pose,
            self.navigator
        )


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
    main()
