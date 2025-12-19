"""
CLI interface for RAG retrieval validation system.
"""
import argparse
import sys
import json
import os
from typing import List, Dict, Any
from datetime import datetime

# Load environment variables
from dotenv import load_dotenv
load_dotenv()

from backend.rag_validator.models.query import Query
from backend.rag_validator.core.validator import RAGValidator
from backend.rag_validator.core.compatibility_checker import CompatibilityChecker
from backend.rag_validator.core.metrics_collector import MetricsCollector
from backend.rag_validator.utils.logger import setup_logger, get_logger


def create_sample_query(text: str, top_k: int = 5) -> Query:
    """
    Create a sample query for testing purposes.

    Args:
        text: Query text
        top_k: Number of results to retrieve

    Returns:
        Query object
    """
    # Input validation
    if not isinstance(text, str):
        raise ValueError("Query text must be a string")

    # Limit text length to prevent abuse
    max_length = 10000  # Adjust based on your needs
    if len(text) > max_length:
        raise ValueError(f"Query text exceeds maximum length of {max_length} characters")

    # Limit top_k to prevent resource exhaustion
    max_top_k = 100  # Adjust based on your needs
    if top_k > max_top_k:
        raise ValueError(f"top_k value {top_k} exceeds maximum {max_top_k}")

    query_id = f"query_{int(datetime.now().timestamp() * 1000)}"
    return Query(
        id=query_id,
        text=text,
        top_k=top_k
    )


def validate_single_query(args: argparse.Namespace) -> None:
    """
    Validate a single query against the RAG retrieval system.

    Args:
        args: Parsed command line arguments
    """
    logger = get_logger("cli")
    validator = RAGValidator()

    # Validate arguments
    if not isinstance(args.query, str) or len(args.query.strip()) == 0:
        print("Error: Query text is required and must be a non-empty string", file=sys.stderr)
        sys.exit(1)

    # Limit top_k to prevent resource exhaustion
    max_top_k = 100  # Adjust based on your needs
    if args.top_k > max_top_k or args.top_k < 1:
        print(f"Error: top_k must be between 1 and {max_top_k}", file=sys.stderr)
        sys.exit(1)

    try:
        query = create_sample_query(args.query, args.top_k)
        logger.info(f"Validating query: {args.query}")

        result = validator.validate_query(query)

        # Print result
        print(json.dumps({
            "result_id": result.id,
            "query_id": result.query_id,
            "accuracy_score": result.accuracy_score,
            "latency_ms": result.latency_ms,
            "total_retrieved": result.total_retrieved,
            "metadata_validation_passed": result.metadata_validation_passed,
            "embedding_compatibility_passed": result.embedding_compatibility_passed,
            "timestamp": result.timestamp.isoformat()
        }, indent=2))

        if result.error_message:
            print(f"Error: {result.error_message}", file=sys.stderr)
            sys.exit(1)
    except ValueError as e:
        print(f"Validation error: {str(e)}", file=sys.stderr)
        sys.exit(1)
    except Exception as e:
        print(f"Unexpected error: {str(e)}", file=sys.stderr)
        sys.exit(1)


def validate_compatibility(args: argparse.Namespace) -> None:
    """
    Check embedding model compatibility.

    Args:
        args: Parsed command line arguments
    """
    logger = get_logger("cli")
    checker = CompatibilityChecker()

    logger.info("Checking embedding model compatibility...")
    result = checker.check_compatibility()

    print(json.dumps(result, indent=2))


def run_performance_benchmark(args: argparse.Namespace) -> None:
    """
    Run performance benchmark.

    Args:
        args: Parsed command line arguments
    """
    logger = get_logger("cli")
    collector = MetricsCollector()

    logger.info(f"Running performance benchmark with {args.concurrent_requests} concurrent requests...")
    result = collector.run_benchmark(
        concurrent_requests=args.concurrent_requests,
        duration_seconds=args.duration
    )

    print(json.dumps(result, indent=2))


def run_batch_validation(args: argparse.Namespace) -> None:
    """
    Run batch validation with multiple queries.

    Args:
        args: Parsed command line arguments
    """
    logger = get_logger("cli")
    validator = RAGValidator()

    # Load queries from file or use provided test queries
    if args.test_file:
        # Validate file path to prevent path traversal
        import os
        # Normalize the path to prevent directory traversal
        normalized_path = os.path.normpath(args.test_file)
        # Ensure the path is within the expected directory
        if not normalized_path.startswith('backend/test_data') and not normalized_path.startswith('./backend/test_data'):
            print("Error: Invalid test file path", file=sys.stderr)
            sys.exit(1)

        try:
            with open(args.test_file, 'r') as f:
                test_data = json.load(f)
            queries_text = test_data.get('queries', [])
        except FileNotFoundError:
            print(f"Error: Test file not found: {args.test_file}", file=sys.stderr)
            sys.exit(1)
        except json.JSONDecodeError:
            print(f"Error: Invalid JSON in test file: {args.test_file}", file=sys.stderr)
            sys.exit(1)
        except Exception as e:
            print(f"Error reading test file: {str(e)}", file=sys.stderr)
            sys.exit(1)
    else:
        queries_text = [
            "What are the key principles of Physical AI?",
            "Explain humanoid robotics fundamentals",
            "Describe machine learning applications in robotics"
        ]

    # Limit the number of queries to prevent resource exhaustion
    max_batch_size = 100  # Adjust based on your needs
    if len(queries_text) > max_batch_size:
        print(f"Error: Batch size {len(queries_text)} exceeds maximum {max_batch_size}", file=sys.stderr)
        sys.exit(1)

    # Validate each query text
    for text in queries_text:
        if not isinstance(text, str) or len(text.strip()) == 0:
            print("Error: Query text must be a non-empty string", file=sys.stderr)
            sys.exit(1)

    queries = [create_sample_query(text) for text in queries_text]
    logger.info(f"Running batch validation for {len(queries)} queries...")

    results = validator.validate_batch(queries)

    summary = {
        "batch_id": f"batch_{int(datetime.now().timestamp() * 1000)}",
        "total_queries": len(queries),
        "completed_queries": len([r for r in results if not r.error_message]),
        "failed_queries": len([r for r in results if r.error_message]),
        "results": [
            {
                "result_id": r.id,
                "query_id": r.query_id,
                "accuracy_score": r.accuracy_score,
                "latency_ms": r.latency_ms,
                "metadata_validation_passed": r.metadata_validation_passed,
                "embedding_compatibility_passed": r.embedding_compatibility_passed
            } for r in results
        ],
        "summary": {
            "average_accuracy": sum(r.accuracy_score for r in results) / len(results) if results else 0,
            "average_latency_ms": sum(r.latency_ms for r in results) / len(results) if results else 0,
            "success_rate": len([r for r in results if not r.error_message]) / len(results) if results else 0
        },
        "timestamp": datetime.now().isoformat()
    }

    print(json.dumps(summary, indent=2))


def main() -> None:
    """
    Main CLI entry point.
    """
    # Set up logging
    setup_logger(level="INFO")

    parser = argparse.ArgumentParser(description="RAG Retrieval Validation System")
    subparsers = parser.add_subparsers(dest="command", help="Available commands")

    # Single query validation command
    validate_parser = subparsers.add_parser("validate", help="Validate a single query")
    validate_parser.add_argument("--query", type=str, required=True, help="Query text to validate")
    validate_parser.add_argument("--top-k", type=int, default=5, help="Number of results to retrieve (default: 5)")
    validate_parser.set_defaults(func=validate_single_query)

    # Compatibility check command
    compat_parser = subparsers.add_parser("compatibility", help="Check embedding model compatibility")
    compat_parser.set_defaults(func=validate_compatibility)

    # Performance benchmark command
    perf_parser = subparsers.add_parser("benchmark", help="Run performance benchmark")
    perf_parser.add_argument("--concurrent-requests", type=int, default=10, help="Number of concurrent requests (default: 10)")
    perf_parser.add_argument("--duration", type=int, default=60, help="Duration of benchmark in seconds (default: 60)")
    perf_parser.set_defaults(func=run_performance_benchmark)

    # Batch validation command
    batch_parser = subparsers.add_parser("batch", help="Run batch validation")
    batch_parser.add_argument("--test-file", type=str, help="Path to JSON file with test queries")
    batch_parser.set_defaults(func=run_batch_validation)

    # Parse arguments
    args = parser.parse_args()

    if args.command is None:
        parser.print_help()
        sys.exit(1)

    # Call the appropriate function
    args.func(args)


if __name__ == "__main__":
    main()