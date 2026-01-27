# unitree_legged_test

Python test node for publishing fixed HighCmd messages from unitree_legged_msgs package.

## Usage

1. Build the workspace:
```bash
cd /path/to/catkin_ws
catkin_make
source devel/setup.bash
```

2. Run the test node:
```bash
rosrun unitree_legged_test test_high_cmd_publisher.py
```

3. Optionally set publishing rate (default is 10 Hz):
```bash
rosrun unitree_legged_test test_high_cmd_publisher.py _rate:=20.0
```

## Topics

- **Published**: `/high_cmd` (unitree_legged_msgs/HighCmd)

## Message Details

The node publishes a fixed HighCmd message with the following default values:
- mode: 0 (idle)
- forwardSpeed: 0.0
- sideSpeed: 0.0
- rotateSpeed: 0.0
- All other fields set to default/zero values

You can modify the `create_test_high_cmd()` function in `test_high_cmd_publisher.py` to change the message values.
