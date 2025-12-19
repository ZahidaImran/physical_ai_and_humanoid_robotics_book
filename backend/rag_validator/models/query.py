"""
Query model for RAG retrieval validation system.
"""
from dataclasses import dataclass
from typing import List, Optional
from datetime import datetime


@dataclass
class Query:
    """
    Represents a semantic search request that will be converted to an embedding for similarity search.
    """
    id: str
    text: str
    embedding: Optional[List[float]] = None
    expected_results: Optional[List[str]] = None
    top_k: int = 5
    created_at: datetime = datetime.now()

    def __post_init__(self):
        """
        Validate the query after initialization.
        """
        if not self.text.strip():
            raise ValueError("Query text must not be empty")

        if self.top_k <= 0:
            raise ValueError("top_k must be a positive integer")

        if self.embedding is not None:
            # If embedding exists, validate its dimensions are consistent
            # We'll use a placeholder dimension size - this will be validated against actual model dimensions
            expected_dim = 1024  # Default for Cohere multilingual-v3.0
            if len(self.embedding) != expected_dim:
                raise ValueError(f"Embedding has incorrect dimension: {len(self.embedding)}, expected: {expected_dim}")