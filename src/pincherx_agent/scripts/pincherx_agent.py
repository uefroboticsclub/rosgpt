#!/usr/bin/env python3

import asyncio
import os
from datetime import datetime

import dotenv
import pyinputplus as pyip
import rospy
from rich.console import Console
from rich.console import Group
from rich.live import Live
from rich.markdown import Markdown
from rich.panel import Panel
from rich.text import Text
from rosgpt import ROSGPT

import tools.pincherx as pincherx_tools
from help import get_help
from llm import get_llm
from prompts import get_prompts


class PincherXAgent(ROSGPT):

    def __init__(self, streaming: bool = False, verbose: bool = True):
        self.__blacklist = ["master", "docker"]
        self.__prompts = get_prompts()
        self.__llm = get_llm(streaming=streaming)
        self.__streaming = streaming

        super().__init__(
            ros_version=2,  # PincherX uses ROS2
            llm=self.__llm,
            tool_packages=[pincherx_tools],
            blacklist=self.__blacklist,
            prompts=self.__prompts,
            verbose=verbose,
            accumulate_chat_history=True,
            streaming=streaming,
            show_token_usage=True,
        )

        self.examples = [
            "Move to the home position",
            "Open the gripper wide",
            "Pick up the object at position x=0.15, y=0, z=0.02 and place it at x=0.15, y=0.1, z=0.02",
            "What is the current end-effector pose?",
            "Move the arm in a square pattern 10cm above the table",
            "Wave hello by moving the end-effector left and right 3 times",
        ]

        self.command_handler = {
            "help": lambda: self.submit(get_help(self.examples)),
            "examples": lambda: self.submit(self.choose_example()),
            "clear": lambda: self.clear(),
            "home": lambda: self.submit("Move to home position"),
            "sleep": lambda: self.submit("Move to sleep position"),
        }

    @property
    def greeting(self):
        greeting = Text(
            "\nHi! I'm the ROSGPT-PincherX-100 agent 🦾. How can I help you today?\n"
        )
        greeting.stylize("frame bold blue")
        greeting.append(
            f"Try {', '.join(self.command_handler.keys())} or exit.",
            style="italic",
        )
        return greeting

    def choose_example(self):
        """Get user selection from the list of examples."""
        return pyip.inputMenu(
            self.examples,
            prompt="\nEnter your choice and press enter: \n",
            numbered=True,
            blank=False,
            timeout=60,
            default="1",
        )

    async def clear(self):
        """Clear the chat history."""
        self.clear_chat()
        self.last_events = []
        self.command_handler.pop("info", None)
        os.system("clear")

    def get_input(self, prompt: str):
        """Get user input from the console."""
        return pyip.inputStr(prompt, default="help")

    async def run(self):
        """
        Run the PincherXAgent's main interaction loop.
        """
        await self.clear()
        console = Console()

        while True:
            console.print(self.greeting)
            input = self.get_input("> ")

            # Handle special commands
            if input == "exit":
                break
            elif input in self.command_handler:
                await self.command_handler[input]()
            else:
                await self.submit(input)

    async def submit(self, query: str):
        if self.__streaming:
            await self.stream_response(query)
        else:
            self.print_response(query)

    def print_response(self, query: str):
        """
        Submit the query to the agent and print the response to the console.
        """
        response = self.invoke(query)
        console = Console()
        content_panel = None

        with Live(
            console=console, auto_refresh=True, vertical_overflow="visible"
        ) as live:
            content_panel = Panel(
                Markdown(response), title="Final Response", border_style="green"
            )
            live.update(content_panel, refresh=True)

    async def stream_response(self, query: str):
        """
        Stream the agent's response with rich formatting.
        """
        console = Console()
        content = ""
        self.last_events = []

        panel = Panel("", title="Streaming Response", border_style="green")

        with Live(panel, console=console, auto_refresh=False) as live:
            async for event in self.astream(query):
                event["timestamp"] = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[
                    :-3
                ]
                if event["type"] == "token":
                    content += event["content"]
                    panel.renderable = Markdown(content)
                    live.refresh()
                elif event["type"] in ["tool_start", "tool_end", "error"]:
                    self.last_events.append(event)
                elif event["type"] == "final":
                    content = event["content"]
                    if self.last_events:
                        panel.renderable = Markdown(
                            content
                            + "\n\nType 'info' for details on how I got my answer."
                        )
                    else:
                        panel.renderable = Markdown(content)
                    panel.title = "Final Response"
                    live.refresh()

        if self.last_events:
            self.command_handler["info"] = self.show_event_details
        else:
            self.command_handler.pop("info", None)

    async def show_event_details(self):
        """
        Display detailed information about the events that occurred during the last query.
        """
        console = Console()

        if not self.last_events:
            console.print("[yellow]No events to display.[/yellow]")
            return
        else:
            console.print(Markdown("# Tool Usage and Events"))

        for event in self.last_events:
            timestamp = event["timestamp"]
            if event["type"] == "tool_start":
                console.print(
                    Panel(
                        Group(
                            Text(f"Input: {event.get('input', 'None')}"),
                            Text(f"Timestamp: {timestamp}", style="dim"),
                        ),
                        title=f"Tool Started: {event['name']}",
                        border_style="blue",
                    )
                )
            elif event["type"] == "tool_end":
                console.print(
                    Panel(
                        Group(
                            Text(f"Output: {event.get('output', 'N/A')}"),
                            Text(f"Timestamp: {timestamp}", style="dim"),
                        ),
                        title=f"Tool Completed: {event['name']}",
                        border_style="green",
                    )
                )
            elif event["type"] == "error":
                console.print(
                    Panel(
                        Group(
                            Text(f"Error: {event['content']}", style="bold red"),
                            Text(f"Timestamp: {timestamp}", style="dim"),
                        ),
                        border_style="red",
                    )
                )
            console.print()

        console.print("[bold]End of events[/bold]\n")


def main():
    dotenv.load_dotenv(dotenv.find_dotenv())

    streaming = rospy.get_param("~streaming", False)
    pincherx_agent = PincherXAgent(verbose=False, streaming=streaming)

    asyncio.run(pincherx_agent.run())


if __name__ == "__main__":
    rospy.init_node("pincherx_agent", log_level=rospy.INFO)
    main()
