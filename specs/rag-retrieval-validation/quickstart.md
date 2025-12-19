# Quickstart: RAG Retrieval Validation System

## Overview
This guide will help you set up and run the RAG retrieval validation system to test your RAG pipeline's accuracy, performance, and compatibility.

## Prerequisites
- Python 3.11 or higher
- Access to Qdrant vector database instance
- Cohere API key
- Book content already embedded in Qdrant

## Setup

### 1. Clone the repository
```bash
git clone <repository-url>
cd <repository-name>
```

### 2. Create virtual environment
```bash
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
```

### 3. Install dependencies
```bash
pip install -r requirements.txt
```

### 4. Configure environment variables
Create a `.env` file in the root directory:
```env
QDRANT_URL=your_qdrant_url
QDRANT_API_KEY=your_qdrant_api_key
COHERE_API_KEY=your_cohere_api_key
QDRANT_COLLECTION_NAME=your_collection_name
COHERE_MODEL_NAME=embed-multilingual-v2.0  # or your preferred model
```

## Running Validation Tests

### 1. Basic validation
```bash
python -m backend.rag_validator.cli --query "your test query here" --top-k 5
```

### 2. Batch validation with predefined queries
```bash
python -m backend.rag_validator.cli --validate-batch --test-file path/to/test_queries.json
```

### 3. Performance benchmarking
```bash
python -m backend.rag_validator.cli --benchmark --concurrent-requests 10
```

### 4. Embedding compatibility check
```bash
python -m backend.rag_validator.cli --check-compatibility
```

## Sample Usage

### Python API
```python
from backend.rag_validator.core.validator import RAGValidator
from backend.rag_validator.models.query import Query

# Initialize validator
validator = RAGValidator()

# Create a test query
query = Query(
    text="What are the key principles of Physical AI?",
    top_k=5
)

# Run validation
result = validator.validate_query(query)

# Print results
print(f"Accuracy Score: {result.accuracy_score}")
print(f"Latency: {result.latency_ms}ms")
print(f"Metadata Validation: {'Passed' if result.metadata_validation_passed else 'Failed'}")
```

## Configuration Options

### Environment Variables
- `QDRANT_URL`: URL of your Qdrant instance
- `QDRANT_API_KEY`: API key for Qdrant (if required)
- `QDRANT_COLLECTION_NAME`: Name of the collection containing your embedded content
- `COHERE_API_KEY`: Your Cohere API key
- `COHERE_MODEL_NAME`: Cohere model to use for embeddings (default: embed-multilingual-v2.0)
- `DEFAULT_TOP_K`: Default number of results to retrieve (default: 5)
- `VALIDATION_TIMEOUT`: Timeout for validation requests in seconds (default: 30)

### Command Line Options
- `--query`: Direct query text to test
- `--top-k`: Number of results to retrieve (default: 5)
- `--validate-batch`: Run batch validation
- `--test-file`: Path to file with test queries
- `--benchmark`: Run performance benchmark
- `--concurrent-requests`: Number of concurrent requests for benchmarking
- `--check-compatibility`: Check embedding model compatibility
- `--output-format`: Output format (json, text, csv)

## Sample Output
```json
{
  "query_id": "test-query-123",
  "accuracy_score": 0.85,
  "latency_ms": 142.5,
  "retrieved_chunks": [
    {
      "id": "chunk-001",
      "content": "Physical AI principles include...",
      "metadata": {
        "url": "https://example.com/book/chapter1",
        "section": "Introduction",
        "chapter": "Chapter 1"
      },
      "relevance_score": 0.92
    }
  ],
  "metadata_validation_passed": true,
  "embedding_compatibility_passed": true,
  "timestamp": "2025-12-19T10:30:00Z"
}
```

## Troubleshooting

### Common Issues
1. **Connection errors**: Verify QDRANT_URL and QDRANT_API_KEY are correct
2. **Authentication errors**: Check that your Cohere and Qdrant API keys are valid
3. **Empty results**: Ensure your collection contains embedded content
4. **Dimension mismatch**: Verify that the Cohere model used for queries matches the model used for embedding

### Logging
The system logs to standard output by default. For detailed logs, run with:
```bash
python -m backend.src.rag_validator.cli --verbose
```

## Next Steps
1. Customize test queries for your specific book content
2. Set up automated validation in your CI/CD pipeline
3. Monitor validation metrics over time
4. Adjust validation thresholds based on your requirements