"""
Contract test for GET /performance-benchmark endpoint in RAG retrieval validation system.
"""
import pytest
from backend.rag_validator.core.metrics_collector import MetricsCollector


def test_performance_benchmark_contract():
    """
    Contract test for GET /performance-benchmark endpoint.
    Verifies the interface and basic functionality without deep implementation.
    """
    # Initialize the metrics collector
    collector = MetricsCollector()

    # Execute the performance benchmark with minimal parameters for testing
    result = collector.run_benchmark(concurrent_requests=2, duration_seconds=1)

    # Verify the result structure matches the contract
    assert 'benchmark_id' in result
    assert 'concurrent_requests' in result
    assert 'duration_seconds' in result
    assert 'total_requests' in result
    assert 'successful_requests' in result
    assert 'failed_requests' in result
    assert 'metrics' in result
    assert 'timestamp' in result

    # Verify types
    assert isinstance(result['benchmark_id'], str)
    assert isinstance(result['concurrent_requests'], int)
    assert isinstance(result['duration_seconds'], int)
    assert isinstance(result['total_requests'], int)
    assert isinstance(result['successful_requests'], int)
    assert isinstance(result['failed_requests'], int)
    assert isinstance(result['metrics'], dict)
    assert isinstance(result['timestamp'], (int, float))  # Unix timestamp

    # Verify that request counts are consistent
    assert result['total_requests'] == result['successful_requests'] + result['failed_requests']
    assert result['concurrent_requests'] >= 0
    assert result['duration_seconds'] > 0

    # Verify metrics structure
    metrics = result['metrics']
    assert 'avg_latency_ms' in metrics
    assert 'p95_latency_ms' in metrics
    assert 'p99_latency_ms' in metrics
    assert 'requests_per_second' in metrics
    assert 'error_rate' in metrics

    # Verify metric types and constraints
    assert isinstance(metrics['avg_latency_ms'], (int, float))
    assert isinstance(metrics['p95_latency_ms'], (int, float))
    assert isinstance(metrics['p99_latency_ms'], (int, float))
    assert isinstance(metrics['requests_per_second'], (int, float))
    assert isinstance(metrics['error_rate'], (int, float))

    # Verify that latencies are non-negative
    assert metrics['avg_latency_ms'] >= 0
    assert metrics['p95_latency_ms'] >= 0
    assert metrics['p99_latency_ms'] >= 0

    # Verify error rate is between 0 and 1
    assert 0.0 <= metrics['error_rate'] <= 1.0

    print("✓ Contract test for GET /performance-benchmark passed")


def test_metrics_collector_interface():
    """
    Test the metrics collector interface for latency measurement.
    """
    from backend.rag_validator.models.query import Query
    collector = MetricsCollector()

    # Create a test query
    query = Query(
        id="latency-test-query",
        text="Test query for latency measurement",
        top_k=1
    )

    # Test latency collection
    latency = collector.collect_latency_metrics(query)
    assert isinstance(latency, (int, float))
    assert latency >= 0

    print("✓ Metrics collector interface test passed")


if __name__ == "__main__":
    test_performance_benchmark_contract()
    test_metrics_collector_interface()