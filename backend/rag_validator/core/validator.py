"""
Validator core logic for RAG retrieval validation system.
"""
from typing import List, Dict, Any, Optional
import time

# Load environment variables
from dotenv import load_dotenv
load_dotenv()

from backend.rag_validator.models.query import Query
from backend.rag_validator.models.chunk import ContentChunk
from backend.rag_validator.models.result import ValidationResult
from backend.rag_validator.core.query_handler import QueryHandler
from backend.rag_validator.utils.logger import get_logger
from backend.rag_validator.utils.config import get_config
from backend.rag_validator.core.compatibility_checker import CompatibilityChecker


class RAGValidator:
    """
    Main validation logic for RAG retrieval pipeline.
    """
    def __init__(self):
        """
        Initialize the RAG validator with necessary components.
        """
        self.logger = get_logger("validator")
        self.config = get_config()
        self.query_handler = QueryHandler()

    def validate_query(self, query: Query) -> ValidationResult:
        """
        Validate a single query against the RAG retrieval system.

        Args:
            query: Query object containing the search text and parameters

        Returns:
            ValidationResult object containing the validation results
        """
        # Input validation
        if not isinstance(query, Query):
            raise ValueError("Query must be a Query object")

        # Limit top_k to prevent resource exhaustion
        max_top_k = 100  # Adjust based on your needs
        if query.top_k > max_top_k:
            self.logger.warning(f"Query top_k {query.top_k} exceeds maximum {max_top_k}, limiting to maximum")
            query.top_k = max_top_k

        start_time = time.time()
        result_id = f"validation_{int(start_time * 1000)}"

        try:
            # Perform the query validation
            retrieved_chunks = self.query_handler.validate_query(query)

            # Calculate accuracy score based on expected results
            accuracy_score = self._calculate_accuracy_score(query, retrieved_chunks)

            # Extract relevance scores from the chunks (this would come from the similarity scores)
            relevance_scores = self._extract_relevance_scores(retrieved_chunks)

            # Validate metadata
            metadata_validation_passed = self._validate_metadata(retrieved_chunks)

            # Validate embedding compatibility
            compatibility_checker = CompatibilityChecker()
            embedding_compatibility_passed = all(
                compatibility_checker.validate_embedding_compatibility(chunk.embedding)
                for chunk in retrieved_chunks
                if chunk.embedding is not None
            )

            # Calculate latency
            latency_ms = (time.time() - start_time) * 1000

            # Create validation result
            validation_result = ValidationResult(
                id=result_id,
                query_id=query.id,
                retrieved_chunks=retrieved_chunks,
                accuracy_score=accuracy_score,
                latency_ms=latency_ms,
                relevance_scores=relevance_scores,
                metadata_validation_passed=metadata_validation_passed,
                embedding_compatibility_passed=embedding_compatibility_passed,
                total_retrieved=len(retrieved_chunks),
                expected_retrieved_count=len(query.expected_results) if query.expected_results else 0
            )

            self.logger.info(f"Query validation completed in {latency_ms:.2f}ms with accuracy {accuracy_score:.2f}")
            return validation_result

        except Exception as e:
            latency_ms = (time.time() - start_time) * 1000
            error_msg = str(e)

            # Create a validation result with error information
            validation_result = ValidationResult(
                id=result_id,
                query_id=query.id,
                retrieved_chunks=[],
                accuracy_score=0.0,
                latency_ms=latency_ms,
                relevance_scores=[],
                metadata_validation_passed=False,
                embedding_compatibility_passed=False,
                total_retrieved=0,
                expected_retrieved_count=len(query.expected_results) if query.expected_results else 0,
                error_message=error_msg
            )

            self.logger.error(f"Query validation failed after {latency_ms:.2f}ms: {error_msg}")
            return validation_result

    def _calculate_accuracy_score(self, query: Query, retrieved_chunks: List[ContentChunk]) -> float:
        """
        Calculate accuracy score based on expected results vs actual results.

        Args:
            query: Original query with expected results
            retrieved_chunks: Chunks retrieved by the system

        Returns:
            Accuracy score between 0.0 and 1.0
        """
        if not query.expected_results:
            # If no expected results, we can't calculate accuracy in the traditional sense
            # For now, return 1.0 as a placeholder
            return 1.0

        if not retrieved_chunks:
            return 0.0

        # Count how many expected results were retrieved
        retrieved_ids = {chunk.id for chunk in retrieved_chunks}
        expected_ids = set(query.expected_results)

        # Calculate intersection over union (IoU) style accuracy
        intersection = len(retrieved_ids.intersection(expected_ids))
        union = len(retrieved_ids.union(expected_ids))

        if union == 0:
            return 0.0

        return intersection / union

    def _extract_relevance_scores(self, retrieved_chunks: List[ContentChunk]) -> List[float]:
        """
        Extract relevance scores from retrieved chunks.

        Args:
            retrieved_chunks: List of retrieved content chunks

        Returns:
            List of relevance scores
        """
        # Extract the actual relevance scores from the chunks
        scores = []
        for chunk in retrieved_chunks:
            if chunk.score is not None:
                scores.append(chunk.score)
            else:
                # If no score is available, default to 1.0 as a placeholder
                scores.append(1.0)
        return scores

    def _validate_metadata(self, retrieved_chunks: List[ContentChunk]) -> bool:
        """
        Validate that all retrieved chunks have proper metadata.

        Args:
            retrieved_chunks: List of retrieved content chunks

        Returns:
            True if all chunks have valid metadata, False otherwise
        """
        for chunk in retrieved_chunks:
            metadata = chunk.metadata
            if metadata is None:
                self.logger.warning(f"Chunk {chunk.id} has no metadata")
                return False

            # Validate required metadata fields
            # Using 'or' operator to check for empty strings as well as None values
            if not metadata.url or not metadata.section or not metadata.chapter or not metadata.chunk_id:
                self.logger.warning(f"Chunk {chunk.id} has incomplete metadata")
                return False

        return True

    def validate_batch(self, queries: List[Query]) -> List[ValidationResult]:
        """
        Validate multiple queries in batch.

        Args:
            queries: List of Query objects to validate

        Returns:
            List of ValidationResult objects
        """
        results = []
        for query in queries:
            result = self.validate_query(query)
            results.append(result)

        return results

    def validate_accuracy(self, query: Query) -> float:
        """
        Validate retrieval accuracy using predefined book queries.

        Args:
            query: Query object with expected results

        Returns:
            Accuracy score between 0.0 and 1.0
        """
        validation_result = self.validate_query(query)
        return validation_result.accuracy_score

    def run_performance_benchmark(self, concurrent_requests: int = 10, duration_seconds: int = 60):
        """
        Run performance benchmark through the validator.

        Args:
            concurrent_requests: Number of concurrent requests to simulate
            duration_seconds: Duration of benchmark in seconds

        Returns:
            Benchmark results from MetricsCollector
        """
        from backend.rag_validator.core.metrics_collector import MetricsCollector  # Local import to avoid circular dependency
        collector = MetricsCollector()
        return collector.run_benchmark(concurrent_requests, duration_seconds)