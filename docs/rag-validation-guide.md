# RAG Retrieval Validation Guide

## Overview

The RAG Retrieval Validation System provides comprehensive validation and testing capabilities for RAG (Retrieval-Augmented Generation) retrieval pipelines. It focuses on validating semantic search accuracy, embedding compatibility, and performance metrics for book-specific queries.

## Features

### 1. Query Accuracy Validation
- Performs semantic similarity search against Qdrant vector database
- Retrieves top-k relevant content chunks with full metadata
- Validates retrieval accuracy using predefined book queries
- Measures and logs retrieval latency and errors
- Documents test results and retrieval limitations

### 2. Embedding Compatibility Verification
- Validates Cohere embedding model compatibility with stored vectors in Qdrant
- Checks embedding dimension consistency
- Verifies model version compatibility
- Provides compatibility status reports

### 3. Performance Monitoring
- Measures retrieval latency under various load conditions
- Tracks system stability metrics
- Provides performance benchmarking capabilities
- Monitors error rates during validation

### 4. Batch Validation
- Supports batch validation of multiple queries at once
- Handles concurrent request processing
- Provides aggregate validation metrics

## Architecture

The system is organized into the following modules:

### Core Components
- **Query Handler**: Manages semantic search queries against Qdrant
- **Validator**: Core validation logic for accuracy, compatibility, and performance
- **Compatibility Checker**: Embedding compatibility validation
- **Metrics Collector**: Performance and benchmarking metrics

### Data Models
- **Query**: Represents a semantic search request
- **ContentChunk**: Segment of book data with embedded content
- **ChunkMetadata**: Metadata including URL, section, chapter, chunk ID
- **ValidationResult**: Outcome of validation procedures with metrics
- **EmbeddingVector**: Numerical representation of text content

### Utilities
- **Config**: Configuration management for validation parameters
- **Logger**: Logging infrastructure for validation events

## CLI Usage

### Single Query Validation
```bash
python -m backend.rag_validator.cli validate --query "What are the key principles of Physical AI?" --top-k 5
```

### Compatibility Check
```bash
python -m backend.rag_validator.cli compatibility
```

### Performance Benchmark
```bash
python -m backend.rag_validator.cli benchmark --concurrent-requests 10 --duration 60
```

### Batch Validation
```bash
python -m backend.rag_validator.cli batch --test-file backend/test_data/sample_queries.json
```

## Configuration

The system uses the following environment variables:

- `COHERE_API_KEY`: Cohere API key for embedding generation
- `QDRANT_URL`: URL of the Qdrant vector database
- `QDRANT_API_KEY`: API key for Qdrant access
- `QDRANT_COLLECTION_NAME`: Name of the collection to query (default: rag-embeddings)
- `COHERE_MODEL_NAME`: Cohere model to use (default: embed-multilingual-v2.0)
- `DEFAULT_TOP_K`: Default number of results to retrieve (default: 5)
- `VALIDATION_TIMEOUT`: Timeout for validation requests in seconds (default: 30)
- `MAX_CONCURRENT_REQUESTS`: Maximum concurrent requests for benchmarking (default: 10)

## API Contract

### POST /validate-query
Validates a single query against the RAG retrieval system.

Request:
```json
{
  "query_text": "What are the key principles of Physical AI?",
  "top_k": 5,
  "expected_results": ["chunk-001", "chunk-002"]
}
```

Response:
```json
{
  "result_id": "validation-12345",
  "query_id": "query-67890",
  "retrieved_chunks": [
    {
      "id": "chunk-001",
      "content": "Physical AI principles include...",
      "metadata": {
        "url": "https://example.com/book/chapter1",
        "section": "Introduction",
        "chapter": "Chapter 1",
        "chunk_id": "chunk-001"
      },
      "relevance_score": 0.92
    }
  ],
  "accuracy_score": 0.85,
  "latency_ms": 142.5,
  "relevance_scores": [0.92, 0.88, 0.76, 0.65, 0.52],
  "metadata_validation_passed": true,
  "embedding_compatibility_passed": true,
  "total_retrieved": 5,
  "expected_retrieved_count": 2,
  "timestamp": "2025-12-19T10:30:00Z"
}
```

## Validation Metrics

The system tracks the following metrics:

- **Accuracy Score**: 0.0 to 1.0 measurement of retrieval accuracy
- **Latency**: Time taken for retrieval in milliseconds
- **Relevance Scores**: Individual scores for each retrieved chunk
- **Metadata Validation**: Status of metadata validation (pass/fail)
- **Compatibility Status**: Embedding model compatibility (pass/fail)
- **Total Retrieved**: Count of chunks retrieved
- **Error Rate**: Proportion of failed validation requests

## Best Practices

1. **Environment Setup**: Ensure all required environment variables are properly configured
2. **Query Quality**: Use specific, book-related queries for better validation results
3. **Performance Testing**: Run benchmarks during low-usage periods to avoid impacting other services
4. **Monitoring**: Regularly check validation metrics to identify potential issues
5. **Batch Processing**: Use batch validation for comprehensive testing of multiple queries

## Troubleshooting

### Common Issues

1. **Qdrant Connection Errors**: Verify `QDRANT_URL` and `QDRANT_API_KEY` are correct
2. **Cohere API Errors**: Check that `COHERE_API_KEY` is valid and has sufficient quota
3. **Empty Results**: Ensure the Qdrant collection contains embedded content
4. **Dimension Mismatch**: Verify that the Cohere model used for queries matches the model used for embedding

### Logging

The system logs validation events with appropriate detail levels. Enable debug logging for detailed troubleshooting information.

## Security Considerations

- API keys should be stored securely and not committed to version control
- Validate and sanitize all user inputs before processing
- Implement rate limiting for validation endpoints to prevent abuse
- Ensure network connections use HTTPS/TLS where possible

## Performance Optimization

- Use appropriate indexing strategies in Qdrant for faster retrieval
- Configure optimal batch sizes for embedding generation
- Monitor resource usage during validation runs
- Consider caching frequently accessed validation results

## Integration

The validation system can be integrated into CI/CD pipelines for automated testing of RAG retrieval quality. It can also be used as part of monitoring systems to continuously validate retrieval performance.