# Physical AI and Humanoid Robotics: A Docusaurus Book Development Guidelines

Auto-generated from all feature plans. Last updated: 2025-12-19

## Active Technologies

- Python 3.11 (for RAG/ML workflows)
- qdrant-client (for Qdrant vector database interactions)
- cohere (for embedding generation and compatibility validation)
- python-dotenv (for environment configuration)
- pytest (for testing framework)
- pandas (for data analysis and reporting)
- numpy (for numerical operations)

## Project Structure

```text
backend/
├── src/
│   ├── rag_validator/
│   │   ├── __init__.py
│   │   ├── core/
│   │   │   ├── __init__.py
│   │   │   ├── query_handler.py      # Handles semantic search queries
│   │   │   ├── validator.py          # Main validation logic
│   │   │   ├── compatibility_checker.py  # Embedding compatibility validation
│   │   │   └── metrics_collector.py  # Performance metrics
│   │   ├── models/
│   │   │   ├── __init__.py
│   │   │   ├── query.py              # Query data model
│   │   │   ├── result.py             # Validation result model
│   │   │   └── metadata.py           # Metadata model
│   │   ├── utils/
│   │   │   ├── __init__.py
│   │   │   ├── logger.py             # Logging utilities
│   │   │   └── config.py             # Configuration utilities
│   │   └── cli/
│   │       ├── __init__.py
│   │       └── main.py               # CLI interface for validation
│   └── tests/
│       ├── __init__.py
│       ├── test_query_handler.py
│       ├── test_validator.py
│       ├── test_compatibility_checker.py
│       └── test_metrics_collector.py
└── requirements.txt
```

## Commands

### RAG Validation Commands
- `python -m backend.src.rag_validator.cli --query "your test query" --top-k 5`: Run a single query validation
- `python -m backend.src.rag_validator.cli --validate-batch --test-file path/to/test_queries.json`: Run batch validation
- `python -m backend.src.rag_validator.cli --benchmark --concurrent-requests 10`: Run performance benchmark
- `python -m backend.src.rag_validator.cli --check-compatibility`: Check embedding model compatibility

### Testing Commands
- `pytest backend/src/tests/`: Run all validation tests
- `pytest backend/src/tests/test_validator.py`: Run specific validation tests

### Environment Setup
- `pip install -r requirements.txt`: Install dependencies
- Create `.env` file with required environment variables

## Code Style

### Python
- Follow PEP 8 style guide
- Use type hints for all function parameters and return values
- Write docstrings for all public classes and functions
- Use meaningful variable names that clearly express intent
- Keep functions focused on a single responsibility

### Data Models
- Use dataclasses for structured data
- Include validation in `__post_init__` methods
- Define clear field types and constraints

## Recent Changes

- RAG Retrieval Validation and Testing: Added backend validation system for RAG retrieval pipeline with semantic search, metadata validation, and performance metrics
- RAG Ingestion Pipeline: Added system for extracting content from Docusaurus book URLs, generating embeddings using Cohere, and storing vectors in Qdrant Cloud
- Physical AI and Humanoid Robotics Book: Created Docusaurus-based book structure for the topic

<!-- MANUAL ADDITIONS START -->
<!-- MANUAL ADDITIONS END -->