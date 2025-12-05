# ros2tree

ROS 2 CLI tool to view node/topic tree, inspired by the [tree](https://wikipedia.org/wiki/Tree_(command)) command. Tested on ROS Humble on Ubuntu 22.04.

## Setup
```bash
cd ~/ros2_ws/src
git clone https://github.com/ishaanbhimwal/ros2tree.git
cd ~/ros2_ws
colcon build --packages-select ros2tree
source install/setup.bash
```

## Usage
```bash
ros2 tree nodes
# or
ros2 tree topics
```

## Examples
```bash
ros2 tree nodes -p -P -c # view nodes, show pkg, show PID, colorize output
```

![node](https://github.com/user-attachments/assets/70462d4c-82eb-4488-a098-fb872119ae44)


```bash
ros2 tree topics -c -m # view topics, colorize output, show msg type
```

![topic](https://github.com/user-attachments/assets/869de9a6-b129-4b36-8ba9-19573359320f)
