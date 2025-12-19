"""
Content chunk model for RAG retrieval validation system.
"""
from dataclasses import dataclass
from typing import List, Optional
from datetime import datetime
from .metadata import ChunkMetadata


@dataclass
class ContentChunk:
    """
    Represents a segment of book data that has been embedded and stored in Qdrant with associated metadata.
    """
    id: str
    content: str
    embedding: Optional[List[float]] = None
    metadata: Optional[ChunkMetadata] = None
    vector_id: Optional[str] = None
    score: Optional[float] = None  # Relevance score from similarity search
    created_at: datetime = datetime.now()

    def __post_init__(self):
        """
        Validate the content chunk after initialization.
        """
        if not self.content.strip():
            raise ValueError("Content chunk must not be empty")

        if self.embedding is not None:
            # If embedding exists, validate its dimensions are consistent
            expected_dim = 1024  # Default for Cohere multilingual-v3.0
            if len(self.embedding) != expected_dim:
                raise ValueError(f"Embedding has incorrect dimension: {len(self.embedding)}, expected: {expected_dim}")

        if self.metadata is not None and not isinstance(self.metadata, ChunkMetadata):
            raise ValueError("metadata must be an instance of ChunkMetadata")