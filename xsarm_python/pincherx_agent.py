#!/usr/bin/env python3
import os
import sys
import asyncio
import dotenv
import rclpy
from rclpy.node import Node

# Load environment variables
dotenv.load_dotenv()

def main():
    # Add app source to Python path
    app_src = "/app/src"
    if os.path.exists(app_src):
        sys.path.insert(0, app_src)
    else:
        print(f"Warning: {app_src} not found. Trying to continue anyway...")
        # Try with current directory
        current_dir = os.path.dirname(os.path.abspath(__file__))
        parent_dir = os.path.dirname(os.path.dirname(current_dir))
        if os.path.exists(os.path.join(parent_dir, "src")):
            sys.path.insert(0, os.path.join(parent_dir, "src"))
    
    try:
        # Initialize ROS node
        rclpy.init()
        
        # Import the PincherX agent module
        from pincherx_agent.scripts.pincherx_agent import PincherXAgent
        
        # Create agent instance
        streaming = os.getenv("STREAMING", "false").lower() == "true"
        agent = PincherXAgent(streaming=streaming, verbose=False)
        
        # Run the agent
        print("Starting PincherX Agent...")
        asyncio.run(agent.run())
        
    except ImportError as e:
        print(f"Error importing PincherX agent: {e}")
        print("Possible causes:")
        print("1. Source files are not mounted correctly")
        print("2. The PincherX agent module is not installed")
        print("\nPlease check your Docker setup and file structure.")
        return 1
    except KeyboardInterrupt:
        print("\nAgent terminated by user.")
    except Exception as e:
        print(f"Error running PincherX agent: {e}")
        return 1
    finally:
        try:
            rclpy.shutdown()
        except Exception:
            pass
    
    return 0

if __name__ == "__main__":
    sys.exit(main())