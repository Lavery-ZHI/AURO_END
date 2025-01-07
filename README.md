# AURO Assessment Solution

## Overview
This repository contains a ROS 2-based autonomous robot control system designed for the AURO assessment task. The solution implements a multi-robot coordination system capable of efficiently collecting and sorting colored spheres in a simulated environment. Using TurtleBot3 Waffle Pi robots, the system achieves high-performance item collection through intelligent navigation and state-based task management.

### Performance Metrics
- Best Score: 435 points in 300 seconds
- Collection Rate: ~1.45 points/second
- Point Distribution:
  - Red Spheres: 5 points each
  - Green Spheres: 10 points each
  - Blue Spheres: 15 points each

## System Requirements
- ROS 2 Humble Hawksbill
- Gazebo Classic 11
- Python 3.8+
- TurtleBot3 Waffle Pi simulation package

## Installation

1. Create and initialize workspace:
```bash
mkdir -p ~/auro_ws/src
cd ~/auro_ws/src
git clone [repository-url]
git clone https://github.com/DLu/tf_transformations.git
```

2. Install dependencies:
```bash
cd ~/auro_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

## Core Components

### Robot Controller (`robot_controller.py`)
The main control node implementing a state machine architecture for task execution:
- State management for navigation and item collection
- Integration with Nav2 for path planning
- Coordination of sensor inputs and motion control

### Robot Module Components

1. **Configuration (`robot/config.py`)**
   - Robot behavior parameters
   - Zone definitions for item sorting
   - State enumeration

2. **Item Handler (`robot/item_handler.py`)**
   - Item detection and approach logic
   - Pick-up and drop-off management
   - Target zone identification

3. **Motion Control (`robot/motion_control.py`)**
   - Obstacle avoidance
   - Turn control
   - Speed management

4. **State Handler (`robot/state_handler.py`)**
   - State-specific behavior implementation
   - Navigation goal management
   - Collection sequence control

### Key Features
- Efficient multi-robot coordination
- Robust obstacle avoidance
- Optimized path planning
- Color-based zone targeting
- Dynamic item prioritization

## Running the Assessment

1. Set environment variables:
```bash
export TURTLEBOT3_MODEL=waffle_pi
```

2. Launch the assessment:
```bash
ros2 launch solution solution_launch.py
```

## State Machine Logic

The robot operates in five primary states:
1. **FORWARD**: Base exploration state
2. **TURNING**: Obstacle avoidance and direction adjustment
3. **COLLECTING**: Item approach and pickup
4. **DROPPING**: Item placement in designated zones
5. **DELIVERING_NAV2**: Navigation to drop-off zones

## Implementation Details

### Navigation Strategy
- Uses Nav2 for global path planning
- Implements reactive local planning for obstacle avoidance
- Dynamic speed adjustment based on proximity to obstacles

### Item Collection
- Visual detection of colored spheres
- Distance-based approach control
- Automated pickup service interaction

### Zone Management
- Color-coded zones for different item types
- Optimized drop-off point selection
- Efficient zone entry approach angles

## Performance Optimization

1. **Speed Optimization**
   - Tuned velocity parameters for efficient movement
   - Optimized turn radiuses for smoother navigation

2. **Collection Strategy**
   - Prioritizes nearby items
   - Efficient path planning to drop-off zones
   - Minimized state transition times

3. **Obstacle Avoidance**
   - Predictive collision detection
   - Smooth avoidance maneuvers
   - Quick recovery from blocked paths

## Author and License
[Your Name]
[License Information]
