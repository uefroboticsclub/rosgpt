#!/usr/bin/env bash

# Check if Docker is installed
if ! command -v docker &> /dev/null; then
    echo "Error: Docker is not installed. Please install Docker and try again."
    exit 1
fi

# Check if docker-compose is installed
if ! command -v docker-compose &> /dev/null; then
    echo "Error: docker-compose is not installed. Please install docker-compose and try again."
    exit 1
fi

# Check if .env file exists, if not create from example
if [ ! -f .env ]; then
    if [ -f .env.example ]; then
        echo "Creating .env file from .env.example..."
        cp .env.example .env
        echo "Please edit .env file to add your Groq API key before running again."
        exit 1
    else
        echo "Error: Neither .env nor .env.example found. Please create a .env file with your configuration."
        exit 1
    fi
fi

# Create xsarm_python directory if it doesn't exist
if [ ! -d xsarm_python ]; then
    echo "Creating xsarm_python directory..."
    mkdir -p xsarm_python
fi

# Ensure the PincherX agent script exists in the xsarm_python directory
if [ ! -f xsarm_python/pincherx_agent.py ]; then
    echo "Copying PincherX agent script to xsarm_python directory..."
    # Check if the script exists in the source files
    if [ -f src/pincherx_agent/scripts/pincherx_agent.py ]; then
        cp src/pincherx_agent/scripts/pincherx_agent.py xsarm_python/
        chmod +x xsarm_python/pincherx_agent.py
    else
        echo "Error: PincherX agent script not found in src/pincherx_agent/scripts/"
        exit 1
    fi
fi

# Enable X11 forwarding for GUI applications
if xhost > /dev/null 2>&1; then
    echo "Enabling X11 forwarding..."
    xhost +local:docker
else
    echo "Warning: xhost command not found. X11 forwarding may not work."
fi

# Build the Docker image using the PincherX Dockerfile
echo "Building Docker image for PincherX agent..."
docker build -t ros_sandbox -f Dockerfile.pincherx .

# Run the container
echo "Starting the PincherX agent container..."
docker run -it --rm \
    --name ros_sandbox \
    --network host \
    --privileged \
    -e DISPLAY=$DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -e ROS_DOMAIN_ID=1 \
    -e GROQ_API_KEY=$(grep GROQ_API_KEY .env | cut -d '=' -f2) \
    -e GROQ_MODEL=$(grep GROQ_MODEL .env | cut -d '=' -f2 || echo "mixtral-8x7b-32768") \
    -e LLM_TEMPERATURE=$(grep LLM_TEMPERATURE .env | cut -d '=' -f2 || echo "0.7") \
    -e LLM_MAX_TOKENS=$(grep LLM_MAX_TOKENS .env | cut -d '=' -f2 || echo "8192") \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v /dev:/dev \
    -v "$(pwd)/src:/app/src" \
    -v "$(pwd)/xsarm_python:/home/user/xsarm_python" \
    ros_sandbox \
    bash -c "source /opt/ros/humble/setup.bash && source /home/user/install/setup.bash && cd /home/user/xsarm_python && python3 pincherx_agent.py"

# Disable X11 forwarding
if xhost > /dev/null 2>&1; then
    echo "Disabling X11 forwarding..."
    xhost -local:docker
fi

exit 0