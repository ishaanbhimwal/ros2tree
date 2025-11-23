# ros2tree

A set of Python cli tools for ROS 2, inspired by the [tree](https://wikipedia.org/wiki/Tree_(command)) command. Tested on ROS Humble on Ubuntu 22.04.

## Setup
```bash
# clone the repo
git clone https://github.com/ishaanbhimwal/ros2tree.git && cd ros2tree

# make the scripts executable
chmod +x ros2tree.py ros2tree_node.py ros2tree_topic.py

# install into /usr/local/bin
sudo install -m 0755 ros2tree.py /usr/local/bin/ros2tree
sudo install -m 0755 ros2tree_node.py /usr/local/bin/ros2tree_node
sudo install -m 0755 ros2tree_topic.py /usr/local/bin/ros2tree_topic
```

## Docker Setup
You can also run ros2tree in a Docker container with ROS 2 Humble:

```bash
docker build -t ros2tree:latest .
```

##### run the container (requires --network host for ROS 2 communication)
```bash
docker run -it --network host ros2tree:latest
```

##### inside the container, you can use the tools directly:
```bash
ros2tree -h
ros2tree_node
ros2tree_topic
```

**Note:** The `--network host` option is required for the container to discover and communicate with ROS 2 nodes running on the host system.

## Examples
```bash
ros2tree -n -p -P -c # view nodes, show pkg, show PID, colorize output
```

![node](./res/screenshot_2025-11-19-204153.png)


```bash
ros2tree -t -c -m # view topics, colorize output, show msg type
```

![topic](./res/screenshot_2025-11-19-204120.png)

## Todo
- Add service script
- Release as a [ROS pkg](https://docs.ros.org/en/kilted/How-To-Guides/Releasing/Releasing-a-Package.html)
