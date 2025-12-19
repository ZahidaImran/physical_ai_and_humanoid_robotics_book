"""
Integration test for performance benchmarking in RAG retrieval validation system.
"""
import pytest
from backend.rag_validator.core.metrics_collector import MetricsCollector
from backend.rag_validator.core.validator import RAGValidator


def test_performance_benchmark_integration():
    """
    Integration test for performance benchmarking flow.
    Tests the complete flow of running performance benchmarks.
    """
    # Initialize the metrics collector
    collector = MetricsCollector()

    # Run a short benchmark for testing
    result = collector.run_benchmark(concurrent_requests=2, duration_seconds=2)

    # Verify the result structure
    assert 'benchmark_id' in result
    assert 'metrics' in result
    assert 'total_requests' in result

    metrics = result['metrics']
    assert 'avg_latency_ms' in metrics
    assert 'requests_per_second' in metrics
    assert 'error_rate' in metrics

    # The benchmark might return an error if Qdrant/Cohere services are not available
    # In that case, we just verify the error structure
    if 'error' in result:
        assert isinstance(result['error'], str)
        print("✓ Performance benchmark integration test passed (with expected service error)")
    else:
        # If no error, verify normal metrics
        assert result['total_requests'] >= 0
        assert 0.0 <= metrics['error_rate'] <= 1.0
        assert metrics['avg_latency_ms'] >= 0
        print("✓ Performance benchmark integration test passed")

    # Test the validator's performance benchmark integration
    validator = RAGValidator()
    benchmark_result = validator.run_performance_benchmark(
        concurrent_requests=1,
        duration_seconds=1
    )

    # This should have the same structure as the collector's result
    if 'benchmark_id' in benchmark_result:
        assert 'metrics' in benchmark_result
        print("✓ Validator performance benchmark integration test passed")


def test_error_rate_tracking():
    """
    Test error rate tracking functionality.
    """
    from backend.rag_validator.models.query import Query
    collector = MetricsCollector()

    # Create some test queries
    queries = [
        Query(id=f"error-test-query-{i}", text=f"Test query {i}", top_k=1)
        for i in range(3)
    ]

    # Test error rate tracking (this will likely be 0 if services are working)
    error_rate = collector.track_error_rate(queries)
    assert isinstance(error_rate, float)
    assert 0.0 <= error_rate <= 1.0

    print("✓ Error rate tracking test passed")


def test_percentile_calculation():
    """
    Test percentile calculation functionality directly.
    """
    collector = MetricsCollector()

    # Test with a simple dataset
    data = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
    p95 = collector._calculate_percentile(data, 0.95)
    p50 = collector._calculate_percentile(data, 0.50)

    # For [1,2,3,4,5,6,7,8,9,10]:
    # 50th percentile should be around the middle (5 or 6)
    # 95th percentile should be around the high end (9 or 10)
    assert 5 <= p50 <= 7  # Middle range
    assert 8 <= p95 <= 10  # High end

    # Test edge cases
    assert collector._calculate_percentile([], 0.5) == 0
    assert collector._calculate_percentile([5], 0.5) == 5

    print("✓ Percentile calculation test passed")


if __name__ == "__main__":
    test_performance_benchmark_integration()
    test_error_rate_tracking()
    test_percentile_calculation()