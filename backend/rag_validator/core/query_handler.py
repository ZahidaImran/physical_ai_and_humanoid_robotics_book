"""
Query handler for RAG retrieval validation system.
Handles semantic search queries against Qdrant.
"""
from typing import List, Dict, Any, Optional
import cohere
from qdrant_client import QdrantClient
from qdrant_client.http import models

# Load environment variables
from dotenv import load_dotenv
load_dotenv()

from backend.rag_validator.models.query import Query
from backend.rag_validator.models.chunk import ContentChunk
from backend.rag_validator.models.metadata import ChunkMetadata
from backend.rag_validator.utils.config import get_config
from backend.rag_validator.utils.logger import get_logger


class QueryHandler:
    """
    Handles semantic search queries against Qdrant vector database.
    """
    def __init__(self):
        """
        Initialize the query handler with Qdrant and Cohere clients.
        """
        self.logger = get_logger("query_handler")
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

    def generate_embedding(self, text: str) -> List[float]:
        """
        Generate embedding for the given text using Cohere API.

        Args:
            text: Text to generate embedding for

        Returns:
            List of floats representing the embedding vector
        """
        # Input validation and sanitization
        if not isinstance(text, str):
            raise ValueError("Text must be a string")

        # Limit text length to prevent abuse
        max_length = 10000  # Adjust based on your needs
        if len(text) > max_length:
            raise ValueError(f"Text exceeds maximum length of {max_length} characters")

        # Sanitize text by removing potentially harmful characters
        # (in case the input comes from user input)
        sanitized_text = text.strip()

        try:
            response = self.cohere_client.embed(
                texts=[sanitized_text],
                model=self.config.cohere_model,
                input_type="search_query"
            )
            return response.embeddings[0]
        except Exception as e:
            self.logger.error(f"Error generating embedding for text: {str(e)}")
            raise

    def search_similar_chunks(self, query_embedding: List[float], top_k: int = 5) -> List[Dict[str, Any]]:
        """
        Search for similar chunks in Qdrant using a query embedding.

        Args:
            query_embedding: Embedding vector to search for similar chunks
            top_k: Number of top similar results to return

        Returns:
            List of dictionaries containing similar chunks with their metadata
        """
        try:
            # Perform similarity search
            search_results = self.qdrant_client.search(
                collection_name=self.collection_name,
                query_vector=query_embedding,
                limit=top_k,
                with_payload=True,
                with_vectors=False
            )

            # Format results
            results = []
            for result in search_results:
                # Extract metadata from payload
                payload = result.payload
                metadata = {k: v for k, v in payload.items() if k not in ["content", "source_url"]}

                results.append({
                    "id": result.id,
                    "content": payload.get("content", ""),
                    "source_url": payload.get("source_url", ""),
                    "score": result.score,
                    "metadata": metadata
                })

            return results
        except Exception as e:
            self.logger.error(f"Error during similarity search: {str(e)}")
            raise

    def validate_query(self, query: Query) -> List[ContentChunk]:
        """
        Validate a query by performing semantic search and returning relevant chunks.

        Args:
            query: Query object containing the search text and parameters

        Returns:
            List of ContentChunk objects representing the retrieved results
        """
        # Generate embedding for the query
        query_embedding = self.generate_embedding(query.text)

        # Search for similar chunks
        search_results = self.search_similar_chunks(query_embedding, query.top_k)

        # Convert search results to ContentChunk objects
        chunks = []
        for result in search_results:
            # Create ChunkMetadata from result metadata
            metadata = ChunkMetadata(
                url=result["source_url"],
                section=result["metadata"].get("section", ""),
                chapter=result["metadata"].get("chapter", ""),
                chunk_id=result["id"],
                source_file=result["metadata"].get("source_file"),
                page_number=result["metadata"].get("page_number")
            )

            chunk = ContentChunk(
                id=result["id"],
                content=result["content"],
                metadata=metadata,
                vector_id=result["id"],
                score=result["score"]  # Include the relevance score from Qdrant
            )
            chunks.append(chunk)

        return chunks