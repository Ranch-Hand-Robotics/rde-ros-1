# Hello World Publisher

A simple ROS 1 Python node that publishes "hello world" messages.

## Package Structure

```
src/
├── package.xml              # Package manifest
├── CMakeLists.txt           # Build configuration
├── scripts/
│   └── hello_world_publisher.py  # Main Python node
├── launch/
│   └── hello_world.launch  # Launch file
└── README.md               # This file
```

## Dependencies

- ROS 1 (Noetic, Melodic, or Kinetic)
- rospy
- std_msgs

## Building

1. Make sure you're in your catkin workspace:
   ```bash
   cd ~/ws
   ```

2. Build the package:
   ```bash
   catkin_make
   ```

3. Source the workspace:
   ```bash
   source devel/setup.bash
   ```

## Running

### Method 1: Using the launch file
```bash
roslaunch hello_world_publisher hello_world.launch
```

### Method 2: Running the node directly
```bash
rosrun hello_world_publisher hello_world_publisher.py
```

## What it does

The node publishes "Hello World!" messages with timestamps to the `/hello_world` topic at a rate of 1 Hz.

## Monitoring

To see the published messages, you can use:
```bash
rostopic echo /hello_world
```

To see the node's log output:
```bash
rosnode info /hello_world_publisher
```

## Stopping

Press `Ctrl+C` to stop the node.
