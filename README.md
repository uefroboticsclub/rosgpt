# ROSGPT: ROS Guide Powered by Transformers

ROSGPT is a framework that connects Large Language Models (LLMs) with Robot Operating System (ROS) to enable natural language control of robotic systems.

## Project Structure

The project consists of several components:

1. **ROSGPT Framework**: Core framework that provides LLM integration with ROS tools and capabilities
2. **TurtleSim Agent**: A demonstration agent that controls a turtle in the ROS TurtleSim environment
3. **PincherX-100 Agent**: An agent that controls a PincherX-100 robotic arm

## Requirements

- Docker
- Docker Compose
- X11 for GUI (not needed for headless mode)
- Groq API key

## Getting Started

1. Clone this repository:
   ```
   git clone https://github.com/uefroboticsclub/rosgpt.git
   cd rosgpt
   ```

2. Copy the environment example file and set your Groq API key:
   ```
   cp .env.example .env
   # Edit .env to add your Groq API key and other settings
   ```

3. Choose which agent to run:

### Running the TurtleSim Agent

```bash
./demo.sh
```

This will set up the Docker environment for the TurtleSim. Once inside the container, build and start the turtle agent:

```bash
start
```

You can enable streaming mode by passing the streaming argument:

```bash
start streaming:=true
```

### Running the PincherX-100 Agent

```bash
./pincherx_demo.sh
```

This will start the Docker container for the PincherX-100 and connect you to the agent.

## Environment Variables

You can customize the agent behavior by setting these variables in your `.env` file:

- `GROQ_API_KEY`: Your API key for Groq
- `GROQ_MODEL`: The model to use (default: "mixtral-8x7b-32768")
- `LLM_TEMPERATURE`: Temperature for model sampling (default: 0.7)
- `LLM_MAX_TOKENS`: Maximum tokens for model response (default: 8192)

## Agent Command Reference

Both agents support the following basic commands:
- `help`: Display help and usage information
- `examples`: Show and select from example commands
- `clear`: Clear the chat history
- `exit`: Exit the agent

### TurtleSim Agent Commands

The TurtleSim agent can understand natural language instructions to:
- Draw shapes
- Move the turtle in specific directions
- Change colors
- Teleport the turtle to specific coordinates

### PincherX-100 Agent Commands

The PincherX-100 agent has these additional commands:
- `home`: Move the arm to home position
- `sleep`: Move the arm to sleep position

The agent can understand natural language instructions to:
- Move to specific joint positions
- Move the end-effector to specific poses
- Open and close the gripper
- Pick and place objects
- Execute complex trajectories

## Docker Environments

The project uses two separate Docker environments:

1. **TurtleSim**: Uses ROS1 (Noetic) for the TurtleSim agent
2. **PincherX-100**: Uses ROS2 (Humble) for the PincherX-100 agent

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

## License

This project is licensed under the MIT License - see the LICENSE file for details.
