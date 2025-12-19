"""
Logging infrastructure for RAG retrieval validation system.
"""
import logging
import sys
from typing import Optional


def setup_logger(name: str = "rag_validator", level: int = logging.INFO) -> logging.Logger:
    """
    Set up a logger with the specified name and level.

    Args:
        name: Name of the logger
        level: Logging level (default: INFO)

    Returns:
        logging.Logger: Configured logger instance
    """
    logger = logging.getLogger(name)

    # Avoid adding multiple handlers if logger already exists
    if logger.handlers:
        return logger

    logger.setLevel(level)

    # Create console handler
    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setLevel(level)

    # Create formatter
    formatter = logging.Formatter(
        '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )
    console_handler.setFormatter(formatter)

    # Add handler to logger
    logger.addHandler(console_handler)

    return logger


def get_logger(name: str = "rag_validator") -> logging.Logger:
    """
    Get a logger instance with the specified name.

    Args:
        name: Name of the logger (default: "rag_validator")

    Returns:
        logging.Logger: Logger instance
    """
    return logging.getLogger(name)