import os

import dotenv
from langchain_groq import ChatGroq


def get_llm(streaming: bool = False):
    """
    A helper function to get the LLM instance using Groq with configurable model.

    Args:
        streaming (bool): Whether to enable streaming responses. Defaults to False.

    Returns:
        ChatGroq: Configured LLM instance
    """
    dotenv.load_dotenv(dotenv.find_dotenv())

    GROQ_API_KEY = os.getenv("GROQ_API_KEY")

    GROQ_MODEL = os.getenv("GROQ_MODEL", "llama-3.3-70b-versatile")

    temperature = float(os.getenv("LLM_TEMPERATURE", "0.7"))

    max_tokens = int(os.getenv("LLM_MAX_TOKENS", "32768"))

    llm = ChatGroq(
        api_key=GROQ_API_KEY,
        model=GROQ_MODEL,
        temperature=temperature,
        max_tokens=max_tokens,
        streaming=streaming,
    )

    return llm
