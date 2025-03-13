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
    echo "Error: X11 forwarding is not working. Please check your X11 server and try again."
    exit 1
fi

# Build and run the Docker container
CONTAINER_NAME="rosgpt-turtlesim-demo"
echo "Building the $CONTAINER_NAME Docker image..."
docker build --build-arg HEADLESS=$HEADLESS -t $CONTAINER_NAME -f Dockerfile . || { echo "Error: Docker build failed"; exit 1; }

echo "Running the Docker container..."
docker run -it --rm --name $CONTAINER_NAME \
    -e DISPLAY=$DISPLAY \
    -e HEADLESS=$HEADLESS \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v "$PWD/src":/app/src \
    -v "$PWD/tests":/app/tests \
    --network host \
    $CONTAINER_NAME

# Disable X11 forwarding
xhost -

exit 0
