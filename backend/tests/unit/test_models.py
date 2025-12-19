"""
Unit tests for RAG retrieval validation models.
"""
import pytest
from datetime import datetime
from backend.rag_validator.models.query import Query
from backend.rag_validator.models.chunk import ContentChunk
from backend.rag_validator.models.metadata import ChunkMetadata
from backend.rag_validator.models.result import ValidationResult


def test_query_model():
    """Test Query model creation and validation."""
    query = Query(
        id="test-query-123",
        text="Test query",
        top_k=5
    )

    assert query.id == "test-query-123"
    assert query.text == "Test query"
    assert query.top_k == 5
    assert isinstance(query.created_at, datetime)

    # Test validation - empty text should raise error
    with pytest.raises(ValueError):
        Query(id="test", text="")

    # Test validation - negative top_k should raise error
    with pytest.raises(ValueError):
        Query(id="test", text="test query", top_k=-1)


def test_chunk_metadata_model():
    """Test ChunkMetadata model creation and validation."""
    metadata = ChunkMetadata(
        url="https://example.com/page",
        section="Introduction",
        chapter="Chapter 1",
        chunk_id="chunk-123"
    )

    assert metadata.url == "https://example.com/page"
    assert metadata.section == "Introduction"
    assert metadata.chapter == "Chapter 1"
    assert metadata.chunk_id == "chunk-123"

    # Test validation - invalid URL should raise error
    with pytest.raises(ValueError):
        ChunkMetadata(
            url="not-a-url",
            section="Intro",
            chapter="Chapter 1",
            chunk_id="chunk-123"
        )

    # Test validation - empty required fields should raise error
    with pytest.raises(ValueError):
        ChunkMetadata(
            url="https://example.com/page",
            section="",
            chapter="Chapter 1",
            chunk_id="chunk-123"
        )


def test_content_chunk_model():
    """Test ContentChunk model creation and validation."""
    metadata = ChunkMetadata(
        url="https://example.com/page",
        section="Introduction",
        chapter="Chapter 1",
        chunk_id="chunk-123"
    )

    chunk = ContentChunk(
        id="chunk-123",
        content="Test content",
        metadata=metadata,
        score=0.85
    )

    assert chunk.id == "chunk-123"
    assert chunk.content == "Test content"
    assert chunk.metadata == metadata
    assert chunk.score == 0.85

    # Test validation - empty content should raise error
    with pytest.raises(ValueError):
        ContentChunk(id="chunk", content="")


def test_validation_result_model():
    """Test ValidationResult model creation and validation."""
    from backend.rag_validator.models.chunk import ContentChunk
    from backend.rag_validator.models.metadata import ChunkMetadata

    metadata = ChunkMetadata(
        url="https://example.com/page",
        section="Introduction",
        chapter="Chapter 1",
        chunk_id="chunk-123"
    )

    chunk = ContentChunk(
        id="chunk-123",
        content="Test content",
        metadata=metadata
    )

    result = ValidationResult(
        id="result-123",
        query_id="query-123",
        retrieved_chunks=[chunk],
        accuracy_score=0.8,
        latency_ms=100.0,
        relevance_scores=[0.85],
        metadata_validation_passed=True,
        embedding_compatibility_passed=True,
        total_retrieved=1,
        expected_retrieved_count=1
    )

    assert result.id == "result-123"
    assert result.query_id == "query-123"
    assert len(result.retrieved_chunks) == 1
    assert result.accuracy_score == 0.8
    assert result.latency_ms == 100.0
    assert result.metadata_validation_passed is True

    # Test validation - invalid accuracy score should raise error
    with pytest.raises(ValueError):
        ValidationResult(
            id="result-123",
            query_id="query-123",
            retrieved_chunks=[chunk],
            accuracy_score=1.5,  # Invalid: > 1.0
            latency_ms=100.0,
            relevance_scores=[0.85],
            metadata_validation_passed=True,
            embedding_compatibility_passed=True,
            total_retrieved=1,
            expected_retrieved_count=1
        )

    # Test validation - negative latency should raise error
    with pytest.raises(ValueError):
        ValidationResult(
            id="result-123",
            query_id="query-123",
            retrieved_chunks=[chunk],
            accuracy_score=0.8,
            latency_ms=-100.0,  # Invalid: negative
            relevance_scores=[0.85],
            metadata_validation_passed=True,
            embedding_compatibility_passed=True,
            total_retrieved=1,
            expected_retrieved_count=1
        )


if __name__ == "__main__":
    test_query_model()
    test_chunk_metadata_model()
    test_content_chunk_model()
    test_validation_result_model()
    print("✓ All model unit tests passed")