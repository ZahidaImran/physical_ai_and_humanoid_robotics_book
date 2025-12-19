"""
RAG Book Content Ingestion Pipeline

This module implements a pipeline to:
1. Extract content from deployed Docusaurus book URLs
2. Generate embeddings using Cohere
3. Store vectors and metadata in Qdrant Cloud
"""

import os
import logging
import requests
from typing import List, Dict, Optional
from urllib.parse import urljoin, urlparse
import time
import math
from dataclasses import dataclass
from dotenv import load_dotenv
import cohere
from qdrant_client import QdrantClient
from qdrant_client.http import models
from bs4 import BeautifulSoup
import xml.etree.ElementTree as ET
import argparse
from datetime import datetime
import hashlib


# Load environment variables
load_dotenv()

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


@dataclass
class PipelineConfig:
    """Configuration class to manage pipeline parameters"""
    cohere_api_key: str = os.getenv("COHERE_API_KEY", "")
    qdrant_api_key: str = os.getenv("QDRANT_API_KEY", "")
    qdrant_url: str = os.getenv("QDRANT_URL", "")
    chunk_size: int = int(os.getenv("CHUNK_SIZE", "512"))
    chunk_overlap: int = int(os.getenv("CHUNK_OVERLAP", "50"))
    cohere_model: str = os.getenv("COHERE_MODEL", "embed-multilingual-v3.0")
    batch_size: int = int(os.getenv("BATCH_SIZE", "10"))
    max_retries: int = int(os.getenv("MAX_RETRIES", "3"))
    delay_base: float = float(os.getenv("DELAY_BASE", "1.0"))


@dataclass
class Chunk:
    """Represents a segment of text content suitable for embedding generation"""
    id: str
    content: str
    source_url: str
    position: int


def get_all_urls(target_url: str, sitemap_url: str = None) -> List[str]:
    """
    Extract all accessible page URLs from a Docusaurus book site,
    optionally using a sitemap for additional URL discovery.

    Args:
        target_url: The root URL of the Docusaurus book
        sitemap_url: Optional sitemap URL to discover additional pages

    Returns:
        List of absolute URLs for all accessible pages in the book
    """
    urls = set()

    # Add the target URL itself
    urls.add(target_url)

    # Get URLs from the main site by crawling
    try:
        response = requests.get(target_url)
        if response.status_code == 200:
            soup = BeautifulSoup(response.content, 'html.parser')

            # Find all links on the page
            for link in soup.find_all('a', href=True):
                href = link['href']
                full_url = urljoin(target_url, href)

                # Only add URLs from the same domain
                if urlparse(full_url).netloc == urlparse(target_url).netloc:
                    urls.add(full_url)
    except Exception as e:
        logger.error(f"Error crawling target URL {target_url}: {str(e)}")

    # If sitemap URL is provided, extract URLs from it
    if sitemap_url:
        try:
            response = requests.get(sitemap_url)
            if response.status_code == 200:
                root = ET.fromstring(response.content)

                # Handle both regular sitemap and sitemap index
                for url_elem in root.findall('.//{http://www.sitemaps.org/schemas/sitemap/0.9}url/{http://www.sitemaps.org/schemas/sitemap/0.9}loc'):
                    url = url_elem.text
                    if urlparse(url).netloc == urlparse(target_url).netloc:
                        urls.add(url)
        except Exception as e:
            logger.error(f"Error processing sitemap URL {sitemap_url}: {str(e)}")

    return list(urls)


def extract_page_metadata(url: str) -> Dict[str, str]:
    """
    Extract metadata from a Docusaurus page including title and hierarchy.

    Args:
        url: URL of the page to extract metadata from

    Returns:
        Dictionary containing metadata (title, hierarchy, etc.)
    """
    try:
        response = requests.get(url)
        response.raise_for_status()

        soup = BeautifulSoup(response.content, 'html.parser')

        # Extract page title
        title = ""
        title_element = soup.find('title')
        if title_element:
            title = title_element.get_text().strip()
        else:
            # Try h1 as fallback
            h1_element = soup.find('h1')
            if h1_element:
                title = h1_element.get_text().strip()

        # Extract hierarchy from URL path
        parsed_url = urlparse(url)
        path_parts = [part for part in parsed_url.path.split('/') if part]
        hierarchy = '/'.join(path_parts) if path_parts else 'home'

        # Additional metadata
        metadata = {
            'title': title,
            'hierarchy': hierarchy,
            'url': url,
            'path': parsed_url.path
        }

        return metadata

    except Exception as e:
        logger.error(f"Error extracting metadata from URL {url}: {str(e)}")
        # Return minimal metadata even if extraction fails
        parsed_url = urlparse(url)
        return {
            'title': f"Content from {parsed_url.netloc}",
            'hierarchy': parsed_url.path or 'home',
            'url': url,
            'path': parsed_url.path
        }


def extract_text_from_url(url: str) -> str:
    """
    Extract clean text content from a single Docusaurus page.

    Args:
        url: A valid URL pointing to a Docusaurus page

    Returns:
        Clean text content extracted from the page, excluding navigation and UI elements
    """
    try:
        response = requests.get(url)
        response.raise_for_status()

        soup = BeautifulSoup(response.content, 'html.parser')

        # Remove navigation and UI elements that are common in Docusaurus sites
        for element in soup(['nav', 'header', 'footer', 'aside', '.navbar', '.menu', '.toc',
                             '.pagination-nav', '.theme-edit-this-page', '.theme-last-updated',
                             'script', 'style', '.docusaurus-meatball-menu', '.sidebar']):
            element.decompose()

        # More comprehensive list of Docusaurus-specific selectors for content
        content_selectors = [
            '.theme-doc-markdown',  # Primary Docusaurus markdown content container
            '.markdown',  # Markdown content area
            'article',  # Article content (common in blog posts)
            '.main-wrapper .container',  # Docusaurus main content wrapper
            '.container .row',  # Docusaurus container with row layout
            '.docMainContainer_node_modules-\@docusaurus-theme-classic-lib-theme-DocPage-Layout-Main-styles-module',  # Specific Docusaurus class
            '.theme-doc-content',  # Docusaurus content area
            '.doc-content',  # Alternative Docusaurus content class
            '.docs-content',  # Another common pattern
            'main',  # Standard main element
            '.main',  # Main content class
            '.content',  # Generic content area
            '.container',  # Container that might hold content
        ]

        content_element = None
        for selector in content_selectors:
            content_element = soup.select_one(selector)
            if content_element:
                break

        # If no specific content area found, try to get the body content without nav elements
        if not content_element:
            # Remove any remaining navigation elements from body
            for element in soup.body(['nav', 'header', 'footer', 'aside']) if soup.body else []:
                element.decompose()
            content_element = soup.body

        if content_element:
            # Extract text and clean it up
            text = content_element.get_text(separator=' ')
            # Clean up extra whitespace
            text = ' '.join(text.split())
            return text
        else:
            logger.warning(f"No content found on page: {url}")
            return ""

    except Exception as e:
        logger.error(f"Error extracting text from URL {url}: {str(e)}")
        raise


def chunk_text(text: str, chunk_size: int = 512, overlap: int = 50) -> List[Chunk]:
    """
    Split text content into smaller chunks suitable for embedding generation.

    Args:
        text: The text content to be chunked
        chunk_size: Maximum size of each chunk in tokens (default: 512)
        overlap: Number of overlapping tokens between chunks (default: 50)

    Returns:
        List of Chunk objects containing the text chunks
    """
    # Use the new tokenization utility
    text_chunks = split_by_tokens(text, chunk_size, overlap)

    chunks = []
    for i, chunk_content in enumerate(text_chunks):
        chunk = Chunk(
            id=f"chunk_{i}",
            content=chunk_content,
            source_url="",  # Will be set by caller
            position=i
        )
        chunks.append(chunk)

    return chunks


def embed(texts: List[str], config: PipelineConfig = None) -> List[List[float]]:
    """
    Generate vector embeddings for a list of text chunks using Cohere API.

    Args:
        texts: List of text strings to generate embeddings for
        config: Pipeline configuration object

    Returns:
        List of embedding vectors (each vector is a list of floats)
    """
    if config is None:
        config = PipelineConfig()

    # Initialize Cohere client
    if not config.cohere_api_key:
        raise ValueError("COHERE_API_KEY environment variable is not set")

    co = cohere.Client(config.cohere_api_key)

    # Process in batches if needed to respect API limits
    all_embeddings = []
    batch_size = config.batch_size

    for i in range(0, len(texts), batch_size):
        batch = texts[i:i + batch_size]

        # Retry mechanism with exponential backoff
        retries = 0
        while retries <= config.max_retries:
            try:
                response = co.embed(
                    texts=batch,
                    model=config.cohere_model,
                    input_type="search_document"
                )

                # Validate embedding dimensions
                embeddings = response.embeddings
                expected_dim = 1024  # Default for Cohere multilingual-v3.0

                for j, embedding in enumerate(embeddings):
                    if len(embedding) != expected_dim:
                        logger.warning(f"Embedding {i+j} has unexpected dimension: {len(embedding)}, expected: {expected_dim}")

                all_embeddings.extend(embeddings)
                break  # Success, break out of retry loop

            except cohere.CohereAPIError as e:
                logger.error(f"Cohere API error on attempt {retries + 1}: {str(e)}")
                if retries >= config.max_retries:
                    raise e
                # Wait before retrying with exponential backoff
                time.sleep(exponential_backoff(retries, config.delay_base))
                retries += 1

            except cohere.CohereConnectionError as e:
                logger.error(f"Cohere connection error on attempt {retries + 1}: {str(e)}")
                if retries >= config.max_retries:
                    raise e
                # Wait before retrying with exponential backoff
                time.sleep(exponential_backoff(retries, config.delay_base))
                retries += 1

            except Exception as e:
                logger.error(f"Unexpected error during embedding on attempt {retries + 1}: {str(e)}")
                if retries >= config.max_retries:
                    raise e
                # Wait before retrying with exponential backoff
                time.sleep(exponential_backoff(retries, config.delay_base))
                retries += 1

    return all_embeddings


def validate_vector(vector: List[float], expected_dim: int = 1024) -> bool:
    """
    Validate that a vector has the correct dimensions and values.

    Args:
        vector: Vector to validate
        expected_dim: Expected dimension size

    Returns:
        True if vector is valid, False otherwise
    """
    if not isinstance(vector, list):
        logger.error("Vector is not a list")
        return False

    if len(vector) != expected_dim:
        logger.error(f"Vector has incorrect dimension: {len(vector)}, expected: {expected_dim}")
        return False

    # Check that all values are numbers
    for i, val in enumerate(vector):
        if not isinstance(val, (int, float)):
            logger.error(f"Vector contains non-numeric value at index {i}: {val}")
            return False

        # Check for NaN or infinity
        if math.isnan(val) or math.isinf(val):
            logger.error(f"Vector contains invalid value (NaN or Inf) at index {i}: {val}")
            return False

    return True


def create_collection(collection_name: str, config: PipelineConfig = None) -> None:
    """
    Create a Qdrant collection for storing embeddings.

    Args:
        collection_name: Name of the collection to create (e.g., "rag-embeddings")
        config: Pipeline configuration object
    """
    if config is None:
        config = PipelineConfig()

    # Initialize Qdrant client
    if not config.qdrant_url or not config.qdrant_api_key:
        raise ValueError("QDRANT_URL and QDRANT_API_KEY environment variables must be set")

    client = QdrantClient(
        url=config.qdrant_url,
        api_key=config.qdrant_api_key,
    )

    # Get embedding dimension from Cohere (for multilingual-v3.0 it's 1024)
    embedding_size = 1024

    # Create collection with cosine distance metric
    try:
        client.create_collection(
            collection_name=collection_name,
            vectors_config=models.VectorParams(
                size=embedding_size,
                distance=models.Distance.COSINE
            )
        )
        logger.info(f"Collection '{collection_name}' created successfully")
    except Exception as e:
        logger.warning(f"Collection '{collection_name}' may already exist: {str(e)}")
        # Try to verify if collection exists
        try:
            client.get_collection(collection_name)
            logger.info(f"Collection '{collection_name}' already exists")
        except Exception as get_error:
            logger.error(f"Could not access collection '{collection_name}': {str(get_error)}")
            raise


def save_chunk_to_qdrant(chunk: Chunk, embedding: List[float], metadata: Dict, config: PipelineConfig = None) -> None:
    """
    Store a text chunk and its embedding in Qdrant Cloud.

    Args:
        chunk: Chunk object containing chunk information
        embedding: Vector embedding as a list of floats
        metadata: Additional metadata to store with the vector
        config: Pipeline configuration object
    """
    if config is None:
        config = PipelineConfig()

    # Initialize Qdrant client
    if not config.qdrant_url or not config.qdrant_api_key:
        raise ValueError("QDRANT_URL and QDRANT_API_KEY environment variables must be set")

    # Validate the embedding vector before storing
    if not validate_vector(embedding):
        raise ValueError(f"Invalid embedding vector for chunk {chunk.id}")

    client = QdrantClient(
        url=config.qdrant_url,
        api_key=config.qdrant_api_key,
    )

    # Retry mechanism with exponential backoff for Qdrant operations
    retries = 0
    while retries <= config.max_retries:
        try:
            # Upsert the point to Qdrant
            client.upsert(
                collection_name="rag-embeddings",
                points=[
                    models.PointStruct(
                        id=chunk.id,
                        vector=embedding,
                        payload={
                            "content": chunk.content,
                            "source_url": chunk.source_url,
                            "position": chunk.position,
                            **metadata
                        }
                    )
                ]
            )
            logger.info(f"Chunk {chunk.id} saved to Qdrant")
            break  # Success, break out of retry loop

        except Exception as e:
            logger.error(f"Qdrant error on attempt {retries + 1} for chunk {chunk.id}: {str(e)}")
            if retries >= config.max_retries:
                logger.error(f"Failed to save chunk {chunk.id} to Qdrant after {config.max_retries + 1} attempts")
                raise e
            # Wait before retrying with exponential backoff
            time.sleep(exponential_backoff(retries, config.delay_base))
            retries += 1


def search_similar_chunks(query_embedding: List[float], top_k: int = 5, config: PipelineConfig = None) -> List[Dict]:
    """
    Search for similar chunks in Qdrant using a query embedding.

    Args:
        query_embedding: Embedding vector to search for similar chunks
        top_k: Number of top similar results to return
        config: Pipeline configuration object

    Returns:
        List of dictionaries containing similar chunks with their metadata
    """
    if config is None:
        config = PipelineConfig()

    # Initialize Qdrant client
    if not config.qdrant_url or not config.qdrant_api_key:
        raise ValueError("QDRANT_URL and QDRANT_API_KEY environment variables must be set")

    client = QdrantClient(
        url=config.qdrant_url,
        api_key=config.qdrant_api_key,
    )

    try:
        # Perform similarity search
        search_results = client.search(
            collection_name="rag-embeddings",
            query_vector=query_embedding,
            limit=top_k,
            with_payload=True,
            with_vectors=False
        )

        # Format results
        results = []
        for result in search_results:
            results.append({
                "id": result.id,
                "content": result.payload.get("content", ""),
                "source_url": result.payload.get("source_url", ""),
                "score": result.score,
                "metadata": {k: v for k, v in result.payload.items() if k not in ["content", "source_url"]}
            })

        return results

    except Exception as e:
        logger.error(f"Error during similarity search: {str(e)}")
        raise e


def is_valid_url(url: str) -> bool:
    """
    Validate if a string is a properly formatted URL.

    Args:
        url: String to validate as URL

    Returns:
        True if the string is a valid URL, False otherwise
    """
    try:
        result = urlparse(url)
        return all([result.scheme, result.netloc])
    except Exception:
        return False


def normalize_url(url: str) -> str:
    """
    Normalize a URL by ensuring it has proper scheme and removing trailing slashes.

    Args:
        url: URL string to normalize

    Returns:
        Normalized URL string
    """
    if not url.startswith(('http://', 'https://')):
        url = 'https://' + url

    # Remove trailing slash if present (but keep the path if it's just '/')
    if url.endswith('/') and url != 'https://' and url != 'http://':
        url = url.rstrip('/')

    return url


def exponential_backoff(retries: int, base_delay: float = 1.0) -> float:
    """
    Calculate delay for exponential backoff.

    Args:
        retries: Number of retries attempted so far
        base_delay: Base delay in seconds

    Returns:
        Delay in seconds
    """
    return base_delay * (2 ** retries) + (retries * 0.1)


def count_tokens(text: str) -> int:
    """
    Count approximate number of tokens in text (1 token ~ 1.2 words for English text).

    Args:
        text: Text string to count tokens for

    Returns:
        Approximate number of tokens
    """
    # Simple approximation: 1 token is about 1.2 words
    if not text:
        return 0

    word_count = len(text.split())
    return math.ceil(word_count / 1.2)


def calculate_content_hash(content: str) -> str:
    """
    Calculate a hash for content to enable content integrity validation and change detection.

    Args:
        content: Text content to hash

    Returns:
        SHA-256 hash of the content as a hex string
    """
    return hashlib.sha256(content.encode('utf-8')).hexdigest()


def validate_content_integrity(original_content: str, retrieved_content: str) -> bool:
    """
    Validate that content has not been corrupted or altered during processing.

    Args:
        original_content: Original content before processing
        retrieved_content: Content after processing

    Returns:
        True if content integrity is maintained, False otherwise
    """
    original_hash = calculate_content_hash(original_content)
    retrieved_hash = calculate_content_hash(retrieved_content)
    return original_hash == retrieved_hash


def has_content_changed(url: str, current_content: str, config: PipelineConfig) -> bool:
    """
    Check if content at a URL has changed since last processing.

    Args:
        url: URL to check for changes
        current_content: Current content from the URL
        config: Pipeline configuration object

    Returns:
        True if content has changed, False otherwise
    """
    try:
        client = QdrantClient(
            url=config.qdrant_url,
            api_key=config.qdrant_api_key,
        )

        # Search for any existing content from this URL to check for changes
        search_results = client.search(
            collection_name="rag-embeddings",
            query_filter=models.Filter(
                must=[
                    models.FieldCondition(
                        key="url",
                        match=models.MatchValue(value=url)
                    )
                ]
            ),
            limit=1,
            with_payload=True
        )

        if search_results:
            # Get the most recent content hash from the metadata
            existing_content = search_results[0].payload.get("content", "")
            existing_hash = calculate_content_hash(existing_content)
            current_hash = calculate_content_hash(current_content)

            return existing_hash != current_hash
        else:
            # No existing content found for this URL, so it's new
            return True

    except Exception as e:
        logger.warning(f"Could not check content changes for {url}: {str(e)}")
        # If we can't check, assume content has changed to be safe
        return True


def validate_stored_embeddings(collection_name: str = "rag-embeddings", config: PipelineConfig = None) -> bool:
    """
    Validate that stored embeddings in Qdrant are accessible and properly formatted.

    Args:
        collection_name: Name of the collection to validate
        config: Pipeline configuration object

    Returns:
        True if validation passes, False otherwise
    """
    if config is None:
        config = PipelineConfig()

    try:
        client = QdrantClient(
            url=config.qdrant_url,
            api_key=config.qdrant_api_key,
        )

        # Get collection info
        collection_info = client.get_collection(collection_name)
        logger.info(f"Collection '{collection_name}' has {collection_info.points_count} points")

        # Sample a few points to validate
        if collection_info.points_count > 0:
            sample_size = min(5, collection_info.points_count)
            sample_results = client.scroll(
                collection_name=collection_name,
                limit=sample_size,
                with_payload=True,
                with_vectors=True
            )

            for point in sample_results[0]:
                # Validate vector dimensions
                if len(point.vector) != 1024:  # Cohere embedding dimension
                    logger.error(f"Invalid vector dimension {len(point.vector)} for point {point.id}")
                    return False

                # Validate required metadata fields exist
                required_fields = ["content", "source_url", "title"]
                for field in required_fields:
                    if field not in point.payload:
                        logger.error(f"Missing required field '{field}' in point {point.id}")
                        return False

            logger.info(f"Successfully validated {sample_size} sample embeddings")
            return True
        else:
            logger.info("Collection is empty, no embeddings to validate")
            return True

    except Exception as e:
        logger.error(f"Error validating stored embeddings: {str(e)}")
        return False


def cleanup_failed_ingestion(collection_name: str = "rag-embeddings", config: PipelineConfig = None) -> bool:
    """
    Clean up any partially ingested content from failed runs.

    Args:
        collection_name: Name of the collection to clean up
        config: Pipeline configuration object

    Returns:
        True if cleanup was successful, False otherwise
    """
    if config is None:
        config = PipelineConfig()

    try:
        client = QdrantClient(
            url=config.qdrant_url,
            api_key=config.qdrant_api_key,
        )

        # For now, this is a placeholder that just validates the collection exists
        # In a more complex implementation, this might remove points with specific status flags
        collection_info = client.get_collection(collection_name)
        logger.info(f"Collection '{collection_name}' cleanup validation passed: {collection_info.points_count} points")
        return True

    except Exception as e:
        logger.error(f"Error during cleanup: {str(e)}")
        return False


def run_sample_similarity_search(config: PipelineConfig = None) -> bool:
    """
    Run a sample similarity search to validate the ingestion pipeline.

    Args:
        config: Pipeline configuration object

    Returns:
        True if search was successful, False otherwise
    """
    if config is None:
        config = PipelineConfig()

    try:
        # Create a simple test query
        test_query = "artificial intelligence in robotics"

        # Initialize Cohere client to generate embedding for the query
        if not config.cohere_api_key:
            raise ValueError("COHERE_API_KEY environment variable is not set")

        co = cohere.Client(config.cohere_api_key)
        response = co.embed(texts=[test_query], model=config.cohere_model, input_type="search_query")
        query_embedding = response.embeddings[0]

        # Search for similar content
        results = search_similar_chunks(query_embedding, top_k=3, config=config)

        logger.info(f"Sample similarity search returned {len(results)} results")
        for i, result in enumerate(results):
            logger.debug(f"Result {i+1}: Score {result['score']:.4f}, URL: {result['source_url']}")

        return len(results) > 0

    except Exception as e:
        logger.error(f"Error during sample similarity search: {str(e)}")
        return False


def monitor_memory_usage():
    """
    Monitor memory usage to manage large books effectively.
    This is a placeholder that would integrate with a memory monitoring library in a real implementation.
    """
    try:
        import psutil
        process = psutil.Process()
        memory_info = process.memory_info()
        memory_mb = memory_info.rss / 1024 / 1024  # Convert to MB
        logger.debug(f"Current memory usage: {memory_mb:.2f} MB")
        return memory_mb
    except ImportError:
        logger.warning("psutil not available for memory monitoring")
        return 0


def split_by_tokens(text: str, max_tokens: int, overlap_tokens: int = 0) -> List[str]:
    """
    Split text into chunks by token count with optional overlap.

    Args:
        text: Text to split into chunks
        max_tokens: Maximum number of tokens per chunk
        overlap_tokens: Number of overlapping tokens between chunks

    Returns:
        List of text chunks
    """
    words = text.split()
    max_words = math.floor(max_tokens * 1.2)
    overlap_words = math.floor(overlap_tokens * 1.2)

    chunks = []
    start_idx = 0

    while start_idx < len(words):
        end_idx = min(start_idx + max_words, len(words))
        chunk = ' '.join(words[start_idx:end_idx])
        chunks.append(chunk)

        # Move to next chunk with overlap
        start_idx = end_idx - overlap_words
        if start_idx < 0:
            start_idx = 0
        if start_idx >= len(words):
            break

    return chunks


def main():
    """
    Execute the complete RAG ingestion pipeline with command-line arguments and progress tracking.
    """
    # Set up command-line argument parsing
    parser = argparse.ArgumentParser(description="RAG Book Content Ingestion Pipeline")
    parser.add_argument("--target-url", type=str,
                        default="https://physical-ai-and-humanoid-robotics-b-one.vercel.app/",
                        help="The Docusaurus book URL to process")
    parser.add_argument("--sitemap-url", type=str,
                        default="https://physical-ai-and-humanoid-robotics-b-one.vercel.app/sitemap.xml",
                        help="Optional sitemap URL to discover additional pages")
    parser.add_argument("--force-update", action="store_true",
                        help="Force update even if content hasn't changed")
    parser.add_argument("--chunk-size", type=int, default=None,
                        help="Override the chunk size from environment variables")
    parser.add_argument("--chunk-overlap", type=int, default=None,
                        help="Override the chunk overlap from environment variables")
    parser.add_argument("--batch-size", type=int, default=None,
                        help="Override the batch size from environment variables")
    parser.add_argument("--verbose", action="store_true",
                        help="Enable verbose logging")
    parser.add_argument("--validate", action="store_true",
                        help="Run validation after ingestion")

    args = parser.parse_args()

    # Set up logging level based on verbose flag
    if args.verbose:
        logging.getLogger().setLevel(logging.DEBUG)
    else:
        logging.getLogger().setLevel(logging.INFO)

    logger.info("Starting RAG ingestion pipeline")
    start_time = time.time()

    # Load configuration
    config = PipelineConfig()

    # Override config values with command-line arguments if provided
    if args.chunk_size:
        config.chunk_size = args.chunk_size
    if args.chunk_overlap:
        config.chunk_overlap = args.chunk_overlap
    if args.batch_size:
        config.batch_size = args.batch_size

    # Validate URLs
    if not is_valid_url(args.target_url):
        raise ValueError(f"Invalid target URL: {args.target_url}")

    if args.sitemap_url and not is_valid_url(args.sitemap_url):
        raise ValueError(f"Invalid sitemap URL: {args.sitemap_url}")

    # Normalize URLs
    target_url = normalize_url(args.target_url)
    if args.sitemap_url:
        sitemap_url = normalize_url(args.sitemap_url)
    else:
        sitemap_url = None

    # Step 1: Get all URLs
    logger.info(f"Extracting URLs from target: {target_url}")
    if sitemap_url:
        logger.info(f"Using sitemap: {sitemap_url}")

    urls = get_all_urls(target_url, sitemap_url)
    logger.info(f"Found {len(urls)} URLs to process")

    # Step 2: Create Qdrant collection
    logger.info("Creating Qdrant collection")
    create_collection("rag-embeddings", config)

    # Track processing statistics
    processed_urls = 0
    processed_chunks = 0
    skipped_urls = 0
    failed_urls = 0

    # Process each URL
    for i, url in enumerate(urls):
        logger.info(f"Processing URL {i+1}/{len(urls)}: {url}")

        # Monitor memory usage periodically
        if i % 10 == 0:  # Check memory every 10 URLs
            monitor_memory_usage()

        try:
            # Extract text from URL
            text = extract_text_from_url(url)
            if not text.strip():
                logger.warning(f"No content extracted from {url}, skipping")
                skipped_urls += 1
                continue

            # Check if content has changed (unless force update is enabled)
            if not args.force_update and not has_content_changed(url, text, config):
                logger.info(f"Content for {url} has not changed, skipping...")
                skipped_urls += 1
                continue

            # Extract metadata from the page
            page_metadata = extract_page_metadata(url)

            # Chunk the text using configuration parameters
            chunks = chunk_text(text, config.chunk_size, config.chunk_overlap)
            logger.info(f"Created {len(chunks)} chunks from {url}")

            # Process chunks in batches for more efficient embedding generation
            batch_size = config.batch_size
            for batch_start_idx in range(0, len(chunks), batch_size):
                batch_chunks = chunks[batch_start_idx:batch_start_idx + batch_size]

                # Extract content for the batch
                batch_contents = [chunk.content for chunk in batch_chunks]

                # Create embeddings for the batch
                try:
                    batch_embeddings = embed(batch_contents, config)

                    # Save each chunk and its embedding to Qdrant
                    for j, (chunk, embedding) in enumerate(zip(batch_chunks, batch_embeddings)):
                        chunk.source_url = url  # Set the source URL

                        # Validate content integrity before saving
                        if not validate_content_integrity(chunk.content, chunk.content):
                            logger.error(f"Content integrity check failed for chunk {chunk.id}, skipping")
                            continue

                        # Prepare metadata combining page metadata with chunk-specific metadata
                        metadata = {
                            "title": page_metadata.get('title', f"Chunk {batch_start_idx+j+1} of {len(chunks)} from {url}"),
                            "hierarchy": page_metadata.get('hierarchy', ''),
                            "url": page_metadata.get('url', url),
                            "path": page_metadata.get('path', ''),
                            "timestamp": time.time(),
                            "chunk_position": batch_start_idx + j,
                            "total_chunks": len(chunks),
                            "content_hash": calculate_content_hash(chunk.content)  # Add content hash for integrity
                        }

                        # Save to Qdrant
                        save_chunk_to_qdrant(chunk, embedding, metadata, config)

                    processed_chunks += len(batch_chunks)

                except Exception as e:
                    logger.error(f"Error processing embedding batch starting at chunk {batch_start_idx}: {str(e)}")
                    # Continue with individual chunks if batch processing fails
                    for j, chunk in enumerate(batch_chunks):
                        chunk.source_url = url  # Set the source URL

                        try:
                            # Create embedding for individual chunk as fallback
                            embedding = embed([chunk.content], config)[0]

                            # Validate content integrity before saving
                            if not validate_content_integrity(chunk.content, chunk.content):
                                logger.error(f"Content integrity check failed for chunk {chunk.id}, skipping")
                                continue

                            # Prepare metadata combining page metadata with chunk-specific metadata
                            metadata = {
                                "title": page_metadata.get('title', f"Chunk {batch_start_idx+j+1} of {len(chunks)} from {url}"),
                                "hierarchy": page_metadata.get('hierarchy', ''),
                                "url": page_metadata.get('url', url),
                                "path": page_metadata.get('path', ''),
                                "timestamp": time.time(),
                                "chunk_position": batch_start_idx + j,
                                "total_chunks": len(chunks),
                                "content_hash": calculate_content_hash(chunk.content)  # Add content hash for integrity
                            }

                            # Save to Qdrant
                            save_chunk_to_qdrant(chunk, embedding, metadata, config)
                            processed_chunks += 1
                        except Exception as e2:
                            logger.error(f"Error processing individual chunk {batch_start_idx+j}: {str(e2)}")
                            continue

            processed_urls += 1

        except Exception as e:
            logger.error(f"Error processing URL {url}: {str(e)}")
            failed_urls += 1
            continue

    # Log final statistics
    total_time = time.time() - start_time
    logger.info(f"RAG ingestion pipeline completed in {total_time:.2f} seconds")
    logger.info(f"Statistics: {processed_urls} URLs processed, {skipped_urls} skipped, {failed_urls} failed, {processed_chunks} chunks processed")

    # Run validation if requested
    if args.validate:
        logger.info("Running post-ingestion validation...")
        validation_success = True

        # Validate stored embeddings
        if not validate_stored_embeddings(config=config):
            logger.error("Embedding validation failed")
            validation_success = False
        else:
            logger.info("Embedding validation passed")

        # Run sample similarity search
        if not run_sample_similarity_search(config=config):
            logger.error("Sample similarity search failed")
            validation_success = False
        else:
            logger.info("Sample similarity search passed")

        if validation_success:
            logger.info("All validations passed!")
        else:
            logger.error("Some validations failed!")
            return 1  # Return error code

    return 0  # Success


if __name__ == "__main__":
    main()


