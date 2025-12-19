"""
Integration test for query validation flow in RAG retrieval validation system.
"""
import pytest
from backend.rag_validator.models.query import Query
from backend.rag_validator.core.validator import RAGValidator
from backend.rag_validator.core.query_handler import QueryHandler


def test_query_validation_integration():
    """
    Integration test for query validation flow.
    Tests the complete flow from query input to validation result.
    """
    # Create a sample query
    query = Query(
        id="integration-test-query-123",
        text="Test query for integration testing",
        top_k=2
    )

    # Initialize the validator
    validator = RAGValidator()

    # Execute the validation
    result = validator.validate_query(query)

    # Verify the result structure
    assert result.query_id == query.id
    assert result.total_retrieved >= 0  # Could be 0 if no results found
    assert result.latency_ms >= 0
    assert 0.0 <= result.accuracy_score <= 1.0
    assert isinstance(result.metadata_validation_passed, bool)
    assert isinstance(result.embedding_compatibility_passed, bool)

    # Verify that the number of retrieved chunks matches the top_k if results exist
    # (Note: in real usage, top_k is the maximum, not necessarily the exact count)
    assert len(result.retrieved_chunks) <= query.top_k

    # Test with a batch of queries
    queries = [
        Query(id=f"batch-query-{i}", text=f"Test query {i}", top_k=2)
        for i in range(3)
    ]

    batch_results = validator.validate_batch(queries)
    assert len(batch_results) == len(queries)

    for i, batch_result in enumerate(batch_results):
        assert batch_result.query_id == queries[i].id

    print("✓ Integration test for query validation flow passed")


def test_query_handler_integration():
    """
    Integration test for the query handler component.
    """
    # Create a sample query
    query = Query(
        id="handler-test-query-123",
        text="Test query for handler integration",
        top_k=3
    )

    # Initialize the query handler
    handler = QueryHandler()

    # Generate embedding
    embedding = handler.generate_embedding(query.text)
    assert isinstance(embedding, list)
    assert len(embedding) > 0  # Should have some dimensions

    # Search for similar chunks
    results = handler.search_similar_chunks(embedding, query.top_k)
    assert isinstance(results, list)
    assert len(results) <= query.top_k

    print("✓ Integration test for query handler passed")


if __name__ == "__main__":
    test_query_validation_integration()
    test_query_handler_integration()