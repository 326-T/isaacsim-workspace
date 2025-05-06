FROM ros:humble

WORKDIR /app

# Install dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Copy the Python scripts
COPY workspace/ /app/workspace/

# Make the Python scripts executable
RUN chmod +x \
    /app/workspace/isaacsim/node.py \
    /app/workspace/integrator/node.py \
    /app/workspace/spacemouse/node.py \
    /app/workspace/keyboard/node.py \
    /app/workspace/logger/node.py

# Source ROS 2 setup in the entrypoint
SHELL ["/bin/bash", "-c"]
ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && exec \"$@\"", "--"]
