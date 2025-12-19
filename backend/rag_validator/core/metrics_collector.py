"""
Metrics collector for RAG retrieval validation system.
Collects performance metrics and handles benchmarking.
"""
from typing import Dict, Any, List
import time
import asyncio
import concurrent.futures
import threading
from threading import Thread

# Load environment variables
from dotenv import load_dotenv
load_dotenv()

from backend.rag_validator.models.query import Query
from backend.rag_validator.core.validator import RAGValidator
from backend.rag_validator.utils.logger import get_logger
from backend.rag_validator.utils.config import get_config


class MetricsCollector:
    """
    Collects metrics and handles performance benchmarking for RAG retrieval.
    """
    def __init__(self):
        """
        Initialize the metrics collector.
        """
        self.logger = get_logger("metrics_collector")
        self.config = get_config()
        self.validator = RAGValidator()

    def collect_latency_metrics(self, query: Query) -> float:
        """
        Collect latency metrics for a single query.

        Args:
            query: Query to measure latency for

        Returns:
            Latency in milliseconds
        """
        start_time = time.time()
        result = self.validator.validate_query(query)
        latency_ms = (time.time() - start_time) * 1000

        self.logger.debug(f"Query {query.id} latency: {latency_ms:.2f}ms")
        return latency_ms

    def run_benchmark(self, concurrent_requests: int = 10, duration_seconds: int = 60) -> Dict[str, Any]:
        """
        Run performance benchmark with concurrent requests.

        Args:
            concurrent_requests: Number of concurrent requests to simulate
            duration_seconds: Duration of benchmark in seconds

        Returns:
            Dictionary containing benchmark results
        """
        benchmark_id = f"perf-benchmark_{int(time.time() * 1000)}"

        # Create sample queries for the benchmark
        sample_queries = [
            Query(
                id=f"benchmark_query_{i}",
                text=f"Sample benchmark query {i}",
                top_k=3
            ) for i in range(concurrent_requests)
        ]

        start_time = time.time()
        completed_requests = 0
        failed_requests = 0
        latencies = []
        latencies_lock = threading.Lock()  # Thread-safe collection of latencies

        try:
            # Execute requests concurrently for the specified duration
            with concurrent.futures.ThreadPoolExecutor(max_workers=concurrent_requests) as executor:
                futures = set()

                # Submit initial batch of requests
                for query in sample_queries:
                    future = executor.submit(self._execute_query_with_timing, query)
                    futures.add(future)

                # Continue processing for the specified duration
                benchmark_end_time = start_time + duration_seconds
                while time.time() < benchmark_end_time or futures:
                    # Use as_completed with timeout to avoid blocking indefinitely
                    try:
                        done_futures = set()
                        for future in concurrent.futures.as_completed(futures, timeout=1):
                            done_futures.add(future)

                        if not done_futures:
                            # No completed futures, check if we should exit
                            if time.time() >= benchmark_end_time and not futures:
                                break
                            continue

                        for future in done_futures:
                            futures.remove(future)
                            try:
                                result = future.result()
                                with latencies_lock:
                                    if result['success']:
                                        completed_requests += 1
                                        latencies.append(result['latency'])
                                    else:
                                        failed_requests += 1

                                # Submit a new request to maintain concurrency
                                if time.time() < benchmark_end_time:
                                    query = Query(
                                        id=f"benchmark_query_{int(time.time() * 1000)}_{completed_requests + failed_requests}",
                                        text=f"Sample benchmark query {completed_requests + failed_requests}",
                                        top_k=3
                                    )
                                    new_future = executor.submit(self._execute_query_with_timing, query)
                                    futures.add(new_future)
                            except Exception as e:
                                self.logger.error(f"Error processing future result: {str(e)}")
                                with latencies_lock:
                                    failed_requests += 1

                    except concurrent.futures.TimeoutError:
                        # Check if benchmark time has elapsed
                        if time.time() >= benchmark_end_time and not futures:
                            break
                        continue

        except Exception as e:
            self.logger.error(f"Benchmark execution error: {str(e)}")
            return {
                "benchmark_id": benchmark_id,
                "error": str(e),
                "timestamp": time.time()
            }

        # Calculate metrics
        total_requests = completed_requests + failed_requests
        if latencies:
            avg_latency = sum(latencies) / len(latencies)
            p95_latency = self._calculate_percentile(latencies, 0.95)
            p99_latency = self._calculate_percentile(latencies, 0.99)
        else:
            avg_latency = 0
            p95_latency = 0
            p99_latency = 0

        requests_per_second = completed_requests / duration_seconds if duration_seconds > 0 else 0
        error_rate = failed_requests / total_requests if total_requests > 0 else 0

        result = {
            "benchmark_id": benchmark_id,
            "concurrent_requests": concurrent_requests,
            "duration_seconds": duration_seconds,
            "total_requests": total_requests,
            "successful_requests": completed_requests,
            "failed_requests": failed_requests,
            "metrics": {
                "avg_latency_ms": avg_latency,
                "p95_latency_ms": p95_latency,
                "p99_latency_ms": p99_latency,
                "requests_per_second": requests_per_second,
                "error_rate": error_rate
            },
            "timestamp": time.time()
        }

        return result

    def _execute_query_with_timing(self, query: Query) -> Dict[str, Any]:
        """
        Execute a query and return timing information.

        Args:
            query: Query to execute

        Returns:
            Dictionary with execution result and timing
        """
        try:
            start_time = time.time()
            result = self.validator.validate_query(query)
            latency = (time.time() - start_time) * 1000

            return {
                "success": result.error_message is None,
                "latency": latency,
                "result": result
            }
        except Exception as e:
            return {
                "success": False,
                "latency": 0,
                "error": str(e)
            }

    def _calculate_percentile(self, data: List[float], percentile: float) -> float:
        """
        Calculate percentile of a list of values.

        Args:
            data: List of values
            percentile: Percentile to calculate (e.g., 0.95 for 95th percentile)

        Returns:
            Calculated percentile value
        """
        if not data:
            return 0

        sorted_data = sorted(data)
        index = int(len(sorted_data) * percentile)
        if index >= len(sorted_data):
            index = len(sorted_data) - 1

        return sorted_data[index]

    def track_error_rate(self, queries: List[Query]) -> float:
        """
        Track error rate across a series of queries.

        Args:
            queries: List of queries to execute

        Returns:
            Error rate as a percentage
        """
        total = len(queries)
        errors = 0

        for query in queries:
            result = self.validator.validate_query(query)
            if result.error_message:
                errors += 1

        return errors / total if total > 0 else 0