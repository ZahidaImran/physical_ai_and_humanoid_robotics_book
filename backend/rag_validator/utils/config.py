"""
Configuration management for RAG retrieval validation system.
"""
import os
from dataclasses import dataclass
from typing import Optional


@dataclass
class RAGValidatorConfig:
    """
    Configuration class to manage RAG validation parameters.
    """
    cohere_api_key: str = os.getenv("COHERE_API_KEY", "")
    qdrant_api_key: str = os.getenv("QDRANT_API_KEY", "")
    qdrant_url: str = os.getenv("QDRANT_URL", "")
    collection_name: str = os.getenv("QDRANT_COLLECTION_NAME", "rag-embeddings")
    cohere_model: str = os.getenv("COHERE_MODEL_NAME", "embed-multilingual-v2.0")
    default_top_k: int = int(os.getenv("DEFAULT_TOP_K", "5"))
    validation_timeout: int = int(os.getenv("VALIDATION_TIMEOUT", "30"))
    max_concurrent_requests: int = int(os.getenv("MAX_CONCURRENT_REQUESTS", "10"))
    max_query_length: int = int(os.getenv("MAX_QUERY_LENGTH", "2000"))


def get_config() -> RAGValidatorConfig:
    """
    Get the current configuration for the RAG validation system.

    Returns:
        RAGValidatorConfig: Configuration object with values from environment variables
    """
    return RAGValidatorConfig()