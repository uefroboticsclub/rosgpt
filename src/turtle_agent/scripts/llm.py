import os

import dotenv
from langchain_groq import ChatGroq

def get_llm(streaming: bool = False):
    """A helper function to get the LLM instance using Grok from xAI."""
    dotenv.load_dotenv(dotenv.find_dotenv())

    GROQ_API_KEY = get_env_variable("GROQ_API_KEY")

    llm = ChatGroq(
        api_key=GROQ_API_KEY,
        model="mixtral-8x7b-32768",
        temperature=0.7, 
        max_tokens=8192,  
        streaming=streaming, 
    )

    return llm

def get_env_variable(var_name: str) -> str:
    """
    Retrieves the value of the specified environment variable.

    Args:
        var_name (str): The name of the environment variable to retrieve.

    Returns:
        str: The value of the environment variable.

    Raises:
        ValueError: If the environment variable is not set.

    This function provides a consistent and safe way to retrieve environment variables.
    By using this function, we ensure that all required environment variables are present
    before proceeding with any operations. If a variable is not set, the function will
    raise a ValueError, making it easier to debug configuration issues.
    """
    value = os.getenv(var_name)
    if value is None:
        raise ValueError(f"Environment variable {var_name} is not set.")
    return value