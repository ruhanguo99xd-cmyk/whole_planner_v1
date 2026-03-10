# Autonomous Walk Package

## Overview

The `autonomous_walk` package provides a complete autonomous walking process for the shovel robot. It integrates with the Nav2 navigation stack to provide high-level autonomous navigation capabilities.

## Features

- **High-level navigation interface**: Provides a simple service interface for setting navigation goals
- **Flexible architecture**: Can use either the Nav2 action directly or the planner_client service
- **Real-time feedback**: Provides feedback on navigation progress
- **Status monitoring**: Tracks the current state of the autonomous walking process
- **Configurable parameters**: Easy to adjust navigation parameters

## Architecture

The package consists of the following components:

- `AutonomousWalk` class: Core implementation of the autonomous walking logic
- Action client: Communicates with Nav2's NavigateToPose action server
- Service client: Optional communication with planner_client service
- Service server: Provides external interface for setting goals
- Launch files: Configuration for launching the autonomous walking system

## Installation

### Dependencies

- ROS2 Humble
- Nav2
- planner_client package (included in this repository)

### Building

```bash
# In the workspace root
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

## Usage

### Launching the System

```bash
# In the workspace root
source install/setup.bash

# Launch autonomous walk with default parameters
ros2 launch autonomous_walk autonomous_walk.launch.py

# Launch with custom parameters
ros2 launch autonomous_walk autonomous_walk.launch.py use_sim_time:=true goal_tolerance:=0.3
```

### Setting a Navigation Goal

#### Using the Service

```bash
# Create a goal pose
goal="{
  goal: {
    header: {
      frame_id: 'map'
    },
    pose: {
      position: {
        x: 1.0,
        y: 2.0,
        z: 0.0
      },
      orientation: {
        x: 0.0,
        y: 0.0,
        z: 0.0,
        w: 1.0
      }
    }
  }
}"

# Call the service
ros2 service call /autonomous_walk/set_goal planner_client/srv/SetGoal "$goal"
```

#### Using the Action Directly

```bash
# Create a goal pose
goal="{
  pose: {
    header: {
      frame_id: 'map'
    },
    pose: {
      position: {
        x: 1.0,
        y: 2.0,
        z: 0.0
      },
      orientation: {
        x: 0.0,
        y: 0.0,
        z: 0.0,
        w: 1.0
      }
    }
  }
}"

# Send the action goal
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "$goal"
```

## Parameters

| Parameter | Description | Default Value |
|-----------|-------------|---------------|
| `goal_tolerance` | Tolerance in meters for goal completion | 0.2 |
| `use_planner_client` | Whether to use the planner_client service | false |
| `navigate_action_name` | Name of the NavigateToPose action | "navigate_to_pose" |
| `odom_topic` | Name of the odometry topic | "odom" |
| `log_level` | Logging verbosity | info |

## Node Information

### Topics

#### Subscriptions
- `odom`: Odometry information

#### Services
- `autonomous_walk/set_goal`: Set navigation goal

#### Actions
- `navigate_to_pose`: Navigate to a specific pose

## Status States

The autonomous walk system can be in the following states:

- `idle`: Initial state
- `initialized`: System initialized
- `goal_set`: Goal has been set
- `walking`: Currently navigating to goal
- `success`: Navigation completed successfully
- `aborted`: Navigation aborted
- `canceled`: Navigation canceled
- `failed`: Navigation failed

## Troubleshooting

### Common Issues

1. **Navigation action server not available**
   - Ensure Nav2 is properly launched
   - Check that the action name is correct

2. **Goal not reached**
   - Check the goal tolerance parameter
   - Verify the robot has a clear path to the goal
   - Check if obstacles are blocking the path

3. **Service not available**
   - Ensure the autonomous_walk node is running
   - Check the service name

### Logs

```bash
# View logs for the autonomous_walk node
ros2 topic echo /rosout | grep autonomous_walk

# View all logs
ros2 run rqt_console rqt_console
```

## Examples

### Basic Navigation

```bash
# Launch the system
ros2 launch autonomous_walk autonomous_walk.launch.py

# Set a goal
ros2 service call /autonomous_walk/set_goal planner_client/srv/SetGoal "{
  goal: {
    header: {
      frame_id: 'map'
    },
    pose: {
      position: {
        x: 5.0,
        y: 5.0,
        z: 0.0
      },
      orientation: {
        x: 0.0,
        y: 0.0,
        z: 0.0,
        w: 1.0
      }
    }
  }
}"
```

### Simulation Navigation

```bash
# Launch with simulation time
ros2 launch autonomous_walk autonomous_walk.launch.py use_sim_time:=true

# Set a goal in the simulation
ros2 service call /autonomous_walk/set_goal planner_client/srv/SetGoal "{
  goal: {
    header: {
      frame_id: 'map'
    },
    pose: {
      position: {
        x: 10.0,
        y: 10.0,
        z: 0.0
      },
      orientation: {
        x: 0.0,
        y: 0.0,
        z: 0.0,
        w: 1.0
      }
    }
  }
}"
```

## API Reference

### Service: autonomous_walk/set_goal

#### Request
```
geometry_msgs/PoseStamped goal
```

#### Response
```
bool success
string message
```

### Action: navigate_to_pose

See Nav2 documentation for NavigateToPose action details.

## License

Apache License 2.0
