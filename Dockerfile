# FROM osrf/ros:noetic-desktop
FROM arm64v8/ros:noetic
ENV DEBIAN_FRONTEND=noninteractive
ENV HEADLESS=false

# Install required packages
RUN apt-get update && apt-get install -y \
    ros-$(rosversion -d)-turtlesim \
    locales \
    xvfb \
    python3.9 \
    python3.9-distutils \
    python3.9-dev \
    curl \
    wget

# Clean up
RUN apt-get clean && rm -rf /var/lib/apt/lists/*

# Install pip for both Python versions using the correct bootstrap scripts
RUN wget https://bootstrap.pypa.io/pip/3.8/get-pip.py -O get-pip-3.8.py && \
    python3 get-pip-3.8.py && \
    rm get-pip-3.8.py && \
    wget https://bootstrap.pypa.io/get-pip.py -O get-pip-3.9.py && \
    python3.9 get-pip-3.9.py && \
    rm get-pip-3.9.py

# Verify pip installation for both versions
RUN python3 -m pip --version && \
    python3.9 -m pip --version

# Set up PATH to include pip-installed binaries
ENV PATH="/root/.local/bin:${PATH}"

# Install ROS tools using system Python (python3)
RUN python3 -m pip install -U python-dotenv catkin_tools

# Set up ROS environment
RUN rosdep update && \
    echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc && \
    echo "alias start='catkin build && source devel/setup.bash && roslaunch turtle_agent agent.launch'" >> /root/.bashrc && \
    echo "export ROSLAUNCH_SSH_UNKNOWN=1" >> /root/.bashrc

# Copy project files
COPY . /app/
WORKDIR /app/

# Install the package with Python 3.9
RUN python3.9 -m pip install --user -e .

# Command to run when container starts
CMD ["/bin/bash", "-c", "source /opt/ros/noetic/setup.bash && \
    roscore > /dev/null 2>&1 & \
    sleep 5 && \
    if [ \"$HEADLESS\" = \"false\" ]; then \
    rosrun turtlesim turtlesim_node & \
    else \
    xvfb-run -a -s \"-screen 0 1920x1080x24\" rosrun turtlesim turtlesim_node & \
    fi && \
    sleep 5 && \
    echo \"Run \\`start\\` to build and launch the ROSGPT-TurtleSim demo.\" && \
    /bin/bash"]