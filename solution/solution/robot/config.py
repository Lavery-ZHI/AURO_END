import math
from enum import Enum

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


# 投放区域定义
ZONES = {
    'BOTTOM_RIGHT': {
        'color': 'RED',
        'x': -3.42,
        'y': -2.46,
        'target_yaw': math.pi / 20
    },
    'TOP_RIGHT': {
        'color': 'GREEN',
        'x': 2.30,
        'y': -2.30,
        'target_yaw': math.pi * 5 / 6
    },
    'TOP_LEFT': {
        'color': 'BLUE',
        'x': 2.30,
        'y': 2.30,
        'target_yaw': -3 * math.pi / 4
    },
    'BOTTOM_LEFT': {
        'color': 'BLACK',
        'x': -3.42,
        'y': 2.46,
        'target_yaw': -math.pi / 4
    }
}
