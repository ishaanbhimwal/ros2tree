# ros2tree

ROS 2 CLI tool to view node/topic tree, inspired by the [tree](https://wikipedia.org/wiki/Tree_(command)) command. Tested on ROS Humble on Ubuntu 22.04</description>

## Usage
```bash
ros2 run tree node
# or
ros2 run tree topic
```

## Examples
```bash
ros2 run tree node -p -P -c # view nodes, show pkg, show PID, colorize output
```

![node](https://github.com/user-attachments/assets/a758689c-ce96-4b88-80e7-a92299463625)


```bash
ros2 run tree topic -c -m # view topics, colorize output, show msg type
```

![topic](https://github.com/user-attachments/assets/9a80cfc8-faf1-4f46-b976-aa389a0658b9)
