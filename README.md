cat > ~/Downloads/gesture_robot_final/README.md << 'EOF'
# Human-Robot Interaction System using micro-ROS

A ROS2-based gesture-controlled robot system built with micro-ROS for real-time human-robot interaction.

## Packages

| Package | Description |
|---|---|
| `bot_bringup` | Launch files to bring up the full robot system |
| `bot_controller` | Gesture-based control logic and command publishing |
| `bot_description` | URDF/robot model and visual description |
| `imu_visualizer` | IMU data visualization and monitoring |

## Requirements

- ROS2 (Humble or later)
- micro-ROS
- Python 3.8+

## Setup

```bash
# Clone the repo
git clone https://github.com/Deepakk-06/Human-Robot-Interaction-System-using-micro-ROS.git
cd Human-Robot-Interaction-System-using-micro-ROS

# Build
colcon build

# Source
source install/setup.bash
```

## Usage

```bash
ros2 launch bot_bringup robot.launch.py
```

## License

MIT
EOF
