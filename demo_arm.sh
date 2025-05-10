#!/usr/bin/env bash

# Check if Docker is installed
if ! command -v docker &> /dev/null; then
    echo "Error: Docker is not installed. Please install Docker and try again."
    exit 1
fi

# Set default headless mode
HEADLESS=${HEADLESS:-false}

# Enable X11 forwarding based on OS
case "$(uname)" in
    Linux*)
        echo "Enabling X11 forwarding..."
        export DISPLAY=:0
        xhost +local:docker
        ;;
    Darwin*)
        echo "Enabling X11 forwarding..."
        export DISPLAY=host.docker.internal:0
        ;;
    MINGW*|CYGWIN*|MSYS*)
        echo "Enabling X11 forwarding for Windows..."
        export DISPLAY=host.docker.internal:0
        ;;
    *)
        echo "Error: Unsupported operating system."
        exit 1
        ;;
esac

# Check if X11 forwarding is working
if ! xset q &>/dev/null; then
    echo "Warning: X11 forwarding is not working. Continuing anyway..."
fi

# Detect architecture and GPU availability
ARCH=$(uname -m)
RUNTIME_FLAGS=""

# Check for NVIDIA GPU and runtime
if command -v nvidia-smi &> /dev/null && docker info | grep -q nvidia; then
    echo "NVIDIA GPU detected, using nvidia runtime"
    RUNTIME_FLAGS="--runtime nvidia"
else
    echo "No NVIDIA GPU detected or nvidia-docker not installed"
fi

# Build and run the Docker container
CONTAINER_NAME="rosgpt-arm-demo"
echo "Building the $CONTAINER_NAME Docker image for architecture: $ARCH"
docker build --build-arg HEADLESS=$HEADLESS -t $CONTAINER_NAME -f Dockerfile.arm . || { echo "Error: Docker build failed"; exit 1; }

echo "Running the Docker container..."
docker run -it --rm --name $CONTAINER_NAME \
    -e DISPLAY=$DISPLAY \
    -e HEADLESS=$HEADLESS \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v "$PWD/src":/app/src \
    -v "$PWD/tests":/app/tests \
    --network host \
    --privileged \
    $RUNTIME_FLAGS \
    $CONTAINER_NAME

# Disable X11 forwarding
if command -v xhost &> /dev/null; then
    xhost -
fi

exit 0