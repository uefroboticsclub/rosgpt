from typing import Optional

class RobotSystemPrompts:
    def __init__(
        self,
        embodiment_and_persona: Optional[str] = None,
        about_your_operators: Optional[str] = None,
        critical_instructions: Optional[str] = None,
        constraints_and_guardrails: Optional[str] = None,
        about_your_environment: Optional[str] = None,
        about_your_capabilities: Optional[str] = None,
        nuance_and_assumptions: Optional[str] = None,
        mission_and_objectives: Optional[str] = None,
        environment_variables: Optional[dict] = None,
    ):
        self.embodiment = embodiment_and_persona
        self.about_your_operators = about_your_operators
        self.critical_instructions = critical_instructions
        self.constraints_and_guardrails = constraints_and_guardrails
        self.about_your_environment = about_your_environment
        self.about_your_capabilities = about_your_capabilities
        self.nuance_and_assumptions = nuance_and_assumptions
        self.mission_and_objectives = mission_and_objectives
        self.environment_variables = environment_variables

    def as_message(self) -> tuple:
        """Return the robot prompts as a tuple of strings for use with AI tools."""
        return "system", str(self)

    def __str__(self):
        s = "Robot System: "
       
        for attr in dir(self):
            if (
                not attr.startswith("_")
                and isinstance(getattr(self, attr), str)
                and getattr(self, attr).strip() != ""
            ):
                s += f"{attr.replace('_', ' ').title()}: {getattr(self, attr)} | "
        return s


system_prompts = [
    (
        "system",
        "You are ROSGPT, an AI agent that can use ROS tools to control robotic systems using natural language. "
        "Use your available tools to interact with the robot and solve tasks."
    ),
    (
        "system",
        "When asked about topics or nodes, first retrieve a list using the appropriate tool before referencing specific names."
    ),
    (
        "system",
        "You must use your tools to perform all actions. Do not claim to execute actions without using the appropriate tool."
    ),
]