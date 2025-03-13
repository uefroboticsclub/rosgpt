import inspect
from functools import wraps
from typing import Literal, List, Optional

from langchain.agents import Tool


def inject_blacklist(default_blacklist: List[str]):
    """
    Inject a blacklist parameter into @tool functions that require it. Required because we do not
    want to rely on the LLM to manually use the blacklist, as it may "forget" to do so.

    Wraps all @tool functions with a new function that injects the blacklist parameter if it is not already present.
    Ensures the function signature and parameter list are maintained, such that LangChain can properly execute the
    function. Without this, LangChain would throw an error. Tried to use partial functions, but it did not work.
    """

    def decorator(func):
        @wraps(func)
        def wrapper(*args, **kwargs):
            if args and isinstance(args[0], dict):
                if "blacklist" in args[0]:
                    args[0]["blacklist"] = default_blacklist + args[0]["blacklist"]
                else:
                    args[0]["blacklist"] = default_blacklist
            else:
                if "blacklist" in kwargs:
                    kwargs["blacklist"] = default_blacklist + kwargs["blacklist"]
                else:
                    params = inspect.signature(func).parameters
                    if "blacklist" in params:
                        kwargs["blacklist"] = default_blacklist
            return func(*args, **kwargs)

        sig = inspect.signature(func)
        new_params = [
            (
                param.replace(default=default_blacklist)
                if param.name == "blacklist"
                else param
            )
            for param in sig.parameters.values()
        ]
        wrapper.__signature__ = sig.replace(parameters=new_params)
        return wrapper

    return decorator


class ROSGPTTools:
    def __init__(
        self, ros_version: Literal[1, 2], blacklist: Optional[List[str]] = None
    ):
        self.__tools: list = []
        self.__ros_version = ros_version
        self.__blacklist = blacklist

        from . import calculation, log, system

        self.__iterative_add(calculation)
        self.__iterative_add(log)
        self.__iterative_add(system)

        if self.__ros_version == 1:
            from . import ros1

            self.__iterative_add(ros1, blacklist=blacklist)
        elif self.__ros_version == 2:
            from . import ros2

            self.__iterative_add(ros2, blacklist=blacklist)
        else:
            raise ValueError("Invalid ROS version. Must be either 1 or 2.")

    def get_tools(self) -> List[Tool]:
        return self.__tools

    def __add_tool(self, tool):
        if hasattr(tool, "name") and hasattr(tool, "func"):
            if self.__blacklist and "blacklist" in tool.func.__code__.co_varnames:
                tool.func = inject_blacklist(self.__blacklist)(tool.func)
            self.__tools.append(tool)

    def __iterative_add(self, package, blacklist: Optional[List[str]] = None):
        """
        Iterate through a package and add each @tool to the tools list.

        :param package: The package to iterate through.
        :param blacklist: A parameter used by some tools to filter out certain results.
        """
        for tool_name in dir(package):
            if not tool_name.startswith("_"):
                t = getattr(package, tool_name)
                self.__add_tool(t)

    def add_packages(self, tool_packages: List, blacklist: Optional[List[str]] = None):
        """
        Add a list of tools to the Tools object by iterating through each package.

        :param tool_packages: A list of tool packages to add to the Tools object.
        """
        for pkg in tool_packages:
            self.__iterative_add(pkg, blacklist=blacklist)

    def add_tools(self, tools: list):
        """
        Add a single tool to the Tools object.

        :param tools: A list of tools to add
        """
        for tool in tools:
            self.__add_tool(tool)
