"""
Compatibility checker for RAG retrieval validation system.
Validates embedding model compatibility with stored vectors in Qdrant.
"""
from typing import Dict, Any, List
import cohere
from qdrant_client import QdrantClient

# Load environment variables
from dotenv import load_dotenv
load_dotenv()

from backend.rag_validator.models.chunk import ContentChunk
from backend.rag_validator.models.embedding import EmbeddingVector
from backend.rag_validator.utils.config import get_config
from backend.rag_validator.utils.logger import get_logger


class CompatibilityChecker:
    """
    Validates that Cohere embedding models are compatible with stored vectors in Qdrant.
    """
    def __init__(self):
        """
        Initialize the compatibility checker with necessary clients.
        """
        self.logger = get_logger("compatibility_checker")
        self.config = get_config()

        # Initialize Cohere client
        if not self.config.cohere_api_key:
            raise ValueError("COHERE_API_KEY environment variable is not set")
        self.cohere_client = cohere.Client(self.config.cohere_api_key)

        # Initialize Qdrant client
        if not self.config.qdrant_url or not self.config.qdrant_api_key:
            raise ValueError("QDRANT_URL and QDRANT_API_KEY environment variables must be set")

        self.qdrant_client = QdrantClient(
            url=self.config.qdrant_url,
            api_key=self.config.qdrant_api_key,
        )

        self.collection_name = self.config.collection_name

    def check_compatibility(self) -> Dict[str, Any]:
        """
        Check embedding model compatibility.

        Returns:
            Dictionary containing compatibility check results
        """
        check_id = f"compatibility-check_{int(self.config.validation_timeout * 1000)}"

        try:
            # Get expected dimensions from Cohere model
            expected_dimensions = self._get_expected_dimensions()

            # Get actual dimensions from Qdrant collection
            actual_dimensions = self._get_actual_dimensions_from_qdrant()

            # Check if dimensions match
            dimensions_match = expected_dimensions == actual_dimensions

            # Get collection info
            collection_info = self._get_collection_info()

            # Create result
            result = {
                "check_id": check_id,
                "model_version": self.config.cohere_model,
                "expected_dimensions": expected_dimensions,
                "actual_dimensions": actual_dimensions,
                "compatibility_status": "compatible" if dimensions_match else "incompatible",
                "qdrant_collection_info": collection_info,
                "timestamp": self.config.validation_timeout  # This will be updated with actual time
            }

            # Update timestamp to current time
            from datetime import datetime
            result["timestamp"] = datetime.now().isoformat()

            self.logger.info(f"Compatibility check completed: {result['compatibility_status']}")
            return result

        except Exception as e:
            self.logger.error(f"Error during compatibility check: {str(e)}")
            return {
                "check_id": check_id,
                "error": str(e),
                "compatibility_status": "error",
                "timestamp": datetime.now().isoformat()
            }

    def _get_expected_dimensions(self) -> int:
        """
        Get expected embedding dimensions for the configured Cohere model.

        Returns:
            Expected dimensions for the model
        """
        # Different Cohere models have different embedding dimensions
        # For multilingual models, it's typically 1024 for v3.0
        model_to_dimensions = {
            "embed-multilingual-v3.0": 1024,
            "embed-multilingual-v2.0": 768,
            "embed-english-v3.0": 1024,
            "embed-english-v2.0": 4096
        }

        return model_to_dimensions.get(self.config.cohere_model, 1024)  # Default to 1024

    def _get_actual_dimensions_from_qdrant(self) -> int:
        """
        Get actual embedding dimensions from Qdrant collection.

        Returns:
            Actual dimensions of vectors in the collection
        """
        try:
            collection_info = self.qdrant_client.get_collection(self.collection_name)
            # Get the vector size from the collection configuration
            if hasattr(collection_info, 'config') and hasattr(collection_info.config, 'params'):
                return collection_info.config.params.vectors.size
            else:
                # Fallback: try to get a single point to check vector size
                points = self.qdrant_client.scroll(
                    collection_name=self.collection_name,
                    limit=1,
                    with_vectors=True
                )

                if points[0]:
                    point = points[0][0]
                    return len(point.vector)

                # If no points exist, return default
                return 1024
        except Exception as e:
            self.logger.warning(f"Could not get actual dimensions from Qdrant: {str(e)}, using default 1024")
            return 1024

    def _get_collection_info(self) -> Dict[str, Any]:
        """
        Get detailed information about the Qdrant collection.

        Returns:
            Dictionary with collection information
        """
        try:
            collection_info = self.qdrant_client.get_collection(self.collection_name)

            info = {
                "collection_name": self.collection_name,
                "vector_size": self._get_actual_dimensions_from_qdrant(),
                "total_vectors": collection_info.points_count,
                "model_version": self.config.cohere_model
            }

            return info
        except Exception as e:
            self.logger.error(f"Error getting collection info: {str(e)}")
            return {
                "collection_name": self.collection_name,
                "error": str(e)
            }

    def validate_embedding_compatibility(self, embedding: List[float]) -> bool:
        """
        Validate if a specific embedding is compatible with the expected dimensions.

        Args:
            embedding: Embedding vector to validate

        Returns:
            True if compatible, False otherwise
        """
        expected_dim = self._get_expected_dimensions()
        return len(embedding) == expected_dim