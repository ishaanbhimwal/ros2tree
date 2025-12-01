FROM ros:humble

WORKDIR /ros2_ws

COPY . src/tree/

RUN bash -c ". /opt/ros/humble/setup.bash && colcon build --packages-select tree --symlink-install"

RUN echo '#!/bin/bash\n\
source /opt/ros/humble/setup.bash\n\
source /ros2_ws/install/setup.bash\n\
exec "$@"' > /entrypoint.sh && chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
