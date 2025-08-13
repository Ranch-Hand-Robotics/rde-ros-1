# RDE C++ - Hello C++ Publisher

A C++ ROS node that publishes "hello cpp" messages.

## Package Structure

```
src/rde_cpp/
├── package.xml                    # Package manifest
├── CMakeLists.txt                 # Build configuration
├── src/
│   └── hello_cpp_publisher.cpp   # Main C++ node source
├── launch/
│   └── hello_cpp.launch          # Launch file
└── README.md                      # This file
```

## Dependencies

- ROS 1 (Noetic, Melodic, or Kinetic)
- roscpp
- std_msgs
- Boost (system, thread)

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
roslaunch rde_cpp hello_cpp.launch
```

### Method 2: Running the node directly
```bash
rosrun rde_cpp hello_cpp_publisher
```

## What it does

The C++ node publishes "Hello C++!" messages with message numbers and timestamps to the `/hello_cpp` topic at a rate of 1 Hz.

## Monitoring

To see the published messages, you can use:
```bash
rostopic echo /hello_cpp
```

To see the node's log output:
```bash
rosnode info /hello_cpp_publisher
```

## Stopping

Press `Ctrl+C` to stop the node.

## C++ Features

- Uses modern C++14 standard
- Proper ROS C++ API usage with NodeHandle
- Comprehensive error handling and logging
- Clean shutdown handling

