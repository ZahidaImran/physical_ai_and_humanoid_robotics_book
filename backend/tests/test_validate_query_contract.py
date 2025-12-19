"""
Contract test for POST /validate-query endpoint in RAG retrieval validation system.
"""
import pytest
from backend.rag_validator.models.query import Query
from backend.rag_validator.core.validator import RAGValidator


def test_validate_query_contract():
    """
    Contract test for POST /validate-query endpoint.
    Verifies the interface and basic functionality without deep implementation.
    """
    # Create a sample query
    query = Query(
        id="test-query-123",
        text="What are the key principles of Physical AI?",
        top_k=3
    )

    # Initialize the validator
    validator = RAGValidator()

    # Execute the validation
    result = validator.validate_query(query)

    # Verify the result structure matches the contract
    assert hasattr(result, 'id')
    assert hasattr(result, 'query_id')
    assert hasattr(result, 'retrieved_chunks')
    assert hasattr(result, 'accuracy_score')
    assert hasattr(result, 'latency_ms')
    assert hasattr(result, 'relevance_scores')
    assert hasattr(result, 'metadata_validation_passed')
    assert hasattr(result, 'embedding_compatibility_passed')
    assert hasattr(result, 'total_retrieved')
    assert hasattr(result, 'expected_retrieved_count')

    # Verify types
    assert isinstance(result.id, str)
    assert isinstance(result.query_id, str)
    assert isinstance(result.retrieved_chunks, list)
    assert isinstance(result.accuracy_score, float)
    assert isinstance(result.latency_ms, float) or isinstance(result.latency_ms, int)
    assert isinstance(result.relevance_scores, list)
    assert isinstance(result.metadata_validation_passed, bool)
    assert isinstance(result.embedding_compatibility_passed, bool)
    assert isinstance(result.total_retrieved, int)
    assert isinstance(result.expected_retrieved_count, int)

    # Verify value constraints
    assert 0.0 <= result.accuracy_score <= 1.0
    assert result.latency_ms >= 0
    assert result.total_retrieved >= 0
    assert result.expected_retrieved_count >= 0

    # Verify that relevance scores and retrieved chunks have matching lengths
    assert len(result.relevance_scores) == len(result.retrieved_chunks)

    print("✓ Contract test for POST /validate-query passed")


if __name__ == "__main__":
    test_validate_query_contract()