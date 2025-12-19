"""
Validation result model for RAG retrieval validation system.
"""
from dataclasses import dataclass
from typing import List, Optional
from datetime import datetime
from .chunk import ContentChunk


@dataclass
class ValidationResult:
    """
    Outcome of testing procedures including accuracy metrics, latency measurements, and compatibility status.
    """
    id: str
    query_id: str
    retrieved_chunks: List[ContentChunk]
    accuracy_score: float
    latency_ms: float
    relevance_scores: List[float]
    metadata_validation_passed: bool
    embedding_compatibility_passed: bool
    total_retrieved: int
    expected_retrieved_count: int
    error_message: Optional[str] = None
    timestamp: datetime = datetime.now()

    def __post_init__(self):
        """
        Validate the validation result after initialization.
        """
        if not 0.0 <= self.accuracy_score <= 1.0:
            raise ValueError(f"Accuracy score must be between 0.0 and 1.0, got: {self.accuracy_score}")

        if self.latency_ms < 0:
            raise ValueError(f"Latency must be non-negative, got: {self.latency_ms}")

        if len(self.relevance_scores) != len(self.retrieved_chunks):
            raise ValueError("Relevance scores length must match retrieved chunks length")

        if self.total_retrieved < 0:
            raise ValueError(f"Total retrieved must be non-negative, got: {self.total_retrieved}")