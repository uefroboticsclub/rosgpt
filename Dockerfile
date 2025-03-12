# Use ROS 2 Humble desktop as the base image (adjust for your ROS 2 distribution, e.g., Iron)
# FROM osrf/ros:humble-desktop
FROM arm64v8/ros:humble

# Set environment variables for non-interactive installation and headless mode
ENV DEBIAN_FRONTEND=noninteractive
ENV HEADLESS=false

# Install system dependencies, including build tools for colcon
RUN apt-get update && apt-get install -y \
    ros-humble-turtlesim \
    locales \
    xvfb \
    python3-pip \
    python3.10 \
    build-essential \
    cmake \
    git \
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-humble-ament-cmake

# Clean up to reduce image size
RUN apt-get clean && rm -rf /var/lib/apt/lists/*

# Only run rosdep update, skipping init since it's already done in the base image
RUN rosdep update || echo "rosdep update may have already been initialized; continuing..."

# Install Python dependencies from requirements.txt
COPY requirements.txt /app/
RUN pip install -r /app/requirements.txt

# Create a ROS 2 workspace in /app/ws
RUN mkdir -p /app/ws/src
WORKDIR /app/ws

# Copy the turtle_agent package to the src directory
# COPY src/turtle_agent/ /app/ws/src/turtle_agent/
COPY src/turtle_agent/ src/turtle_agent/

# Build the workspace with colcon, using /bin/bash for source
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install"
# RUN /bin/bash -c "unset CATKIN_INSTALL_INTO_PREFIX_ROOT CATKIN_SYMLINK_INSTALL && source /opt/ros/humble/setup.bash && colcon build --symlink-install"

# Set up ROS 2 environment and add start function with clean
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /app/ws/install/setup.bash" >> /root/.bashrc && \
    echo "start() { cd /app/ws && rm -rf build/ install/ log/ && colcon build --symlink-install && ros2 run turtlesim turtlesim_node & ros2 run turtle_agent turtle_agent.py; }" >> /root/.bashrc && \
    echo "export ROSLAUNCH_SSH_UNKNOWN=1" >> /root/.bashrc

# Copy other project files (e.g., tests, launch) to /app
COPY . /app/
WORKDIR /app/

# Start the container with turtlesim (if not headless) and prompt to run start
# CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && \
#     source /app/ws/install/setup.bash && \
#     sleep 5 && \
#     if [ \"$HEADLESS\" = \"false\" ]; then \
#     ros2 run turtlesim turtlesim_node & \
#     else \
#     xvfb-run -a -s \"-screen 0 1920x1080x24\" ros2 run turtlesim turtlesim_node & \
#     fi && \
#     sleep 5 && \
#     echo \"Run \\`start\\` to build and launch the ROSGPT-TurtleSim demo.\" && \
#     /bin/bash"]

CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && \
    source /app/ws/install/setup.bash && \
    echo \"Run \\`start\\` to build and launch the ROSGPT-TurtleSim demo.\" && \
    /bin/bash"]