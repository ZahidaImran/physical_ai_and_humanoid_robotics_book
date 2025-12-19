"""
Contract test for GET /compatibility-check endpoint in RAG retrieval validation system.
"""
import pytest
from backend.rag_validator.core.compatibility_checker import CompatibilityChecker


def test_compatibility_check_contract():
    """
    Contract test for GET /compatibility-check endpoint.
    Verifies the interface and basic functionality without deep implementation.
    """
    # Initialize the compatibility checker
    checker = CompatibilityChecker()

    # Execute the compatibility check
    result = checker.check_compatibility()

    # Verify the result structure matches the contract
    assert 'check_id' in result
    assert 'model_version' in result
    assert 'expected_dimensions' in result
    assert 'actual_dimensions' in result
    assert 'compatibility_status' in result
    assert 'qdrant_collection_info' in result
    assert 'timestamp' in result

    # Verify types
    assert isinstance(result['check_id'], str)
    assert isinstance(result['model_version'], str)
    assert isinstance(result['expected_dimensions'], int)
    assert isinstance(result['actual_dimensions'], int)
    assert isinstance(result['compatibility_status'], str)
    assert isinstance(result['qdrant_collection_info'], dict)
    assert isinstance(result['timestamp'], str)  # ISO format string

    # Verify that the status is one of the expected values
    assert result['compatibility_status'] in ['compatible', 'incompatible', 'error']

    # Verify that collection info has expected structure
    collection_info = result['qdrant_collection_info']
    if 'error' not in collection_info:  # Only check structure if no error occurred
        assert 'collection_name' in collection_info
        assert 'vector_size' in collection_info
        assert 'total_vectors' in collection_info
        assert 'model_version' in collection_info

    print("✓ Contract test for GET /compatibility-check passed")


def test_embedding_compatibility_validation():
    """
    Test for embedding compatibility validation function.
    """
    checker = CompatibilityChecker()

    # Test with a mock embedding of correct dimensions (assuming 1024)
    mock_embedding_correct = [0.1] * checker._get_expected_dimensions()
    is_compatible = checker.validate_embedding_compatibility(mock_embedding_correct)
    assert is_compatible is True

    # Test with a mock embedding of wrong dimensions
    mock_embedding_wrong = [0.1] * (checker._get_expected_dimensions() - 100)  # Different size
    is_compatible = checker.validate_embedding_compatibility(mock_embedding_wrong)
    assert is_compatible is False

    print("✓ Embedding compatibility validation test passed")


if __name__ == "__main__":
    test_compatibility_check_contract()
    test_embedding_compatibility_validation()