# Function Contracts: RAG Book Content Ingestion Pipeline

**Feature**: RAG Book Content Ingestion Pipeline
**Created**: 2025-12-18
**Status**: Draft

## Core Functions

### get_all_urls(target_url: str, sitemap_url: str = None) -> List[str]

**Purpose**: Extract all accessible page URLs from a Docusaurus book site, optionally using a sitemap for additional URL discovery

**Input**:
- target_url: The root URL of the Docusaurus book (e.g., "https://physical-ai-and-humanoid-robotics-b-one.vercel.app/")
- sitemap_url: Optional sitemap URL to discover additional pages (e.g., "https://physical-ai-and-humanoid-robotics-b-one.vercel.app/sitemap.xml")

**Output**:
- List of absolute URLs for all accessible pages in the book

**Preconditions**:
- target_url is a valid, accessible Docusaurus site
- If sitemap_url is provided, it must be a valid sitemap
- Network connectivity is available

**Postconditions**:
- Returns a non-empty list of valid URLs if the site has content
- Returns an empty list if no pages are found
- If sitemap_url is provided, the list includes URLs from both the target site and sitemap

**Error Conditions**:
- Raises exception if target_url is inaccessible
- Raises exception if sitemap_url is provided but inaccessible
- Raises exception if network error occurs

**Performance**: Should complete within 30 seconds for sites with up to 1000 pages

### extract_text_from_url(url: str) -> str

**Purpose**: Extract clean text content from a single Docusaurus page

**Input**:
- url: A valid URL pointing to a Docusaurus page

**Output**:
- Clean text content extracted from the page, excluding navigation and UI elements

**Preconditions**:
- URL is accessible and returns valid HTML
- Page follows Docusaurus content structure

**Postconditions**:
- Returns non-empty string if content exists on the page
- Returns empty string if no content found

**Error Conditions**:
- Raises exception if URL is inaccessible
- Raises exception if content extraction fails

**Performance**: Should complete within 5 seconds per page

### chunk_text(text: str, chunk_size: int = 512, overlap: int = 50) -> List[dict]

**Purpose**: Split text content into smaller chunks suitable for embedding generation

**Input**:
- text: The text content to be chunked
- chunk_size: Maximum size of each chunk in tokens (default: 512)
- overlap: Number of overlapping tokens between chunks (default: 50)

**Output**:
- List of dictionaries containing:
  - 'id': unique identifier for the chunk
  - 'content': the text content of the chunk
  - 'position': position of the chunk in the original text

**Preconditions**:
- text is a non-empty string
- chunk_size is a positive integer
- overlap is less than chunk_size

**Postconditions**:
- Returns a list of chunks that preserve the original text content
- Each chunk has content size within specified limits

**Error Conditions**:
- Raises exception if text is empty
- Raises exception if chunk_size is invalid

### embed(texts: List[str]) -> List[List[float]]

**Purpose**: Generate vector embeddings for a list of text chunks using Cohere API

**Input**:
- texts: List of text strings to generate embeddings for

**Output**:
- List of embedding vectors (each vector is a list of floats)

**Preconditions**:
- Cohere API key is properly configured
- Input texts are within API limits (size and count)

**Postconditions**:
- Returns a list of embeddings with the same length as input texts
- Each embedding has consistent dimensions (1024 for multilingual model)

**Error Conditions**:
- Raises exception if Cohere API is unavailable
- Raises exception if API request fails
- Raises exception if input texts exceed API limits

**Performance**: Should process up to 100 texts within 30 seconds

### create_collection(collection_name: str) -> None

**Purpose**: Create a Qdrant collection for storing embeddings

**Input**:
- collection_name: Name of the collection to create (e.g., "rag-embeddings")

**Output**:
- None

**Preconditions**:
- Qdrant connection is properly configured
- Collection does not already exist (or can be recreated)

**Postconditions**:
- Collection exists in Qdrant with appropriate settings
- Collection is ready to receive vector data

**Error Conditions**:
- Raises exception if Qdrant connection fails
- Raises exception if collection creation fails

### save_chunk_to_qdrant(chunk: dict, embedding: List[float], metadata: dict) -> None

**Purpose**: Store a text chunk and its embedding in Qdrant Cloud

**Input**:
- chunk: Dictionary containing chunk information (id, content, position)
- embedding: Vector embedding as a list of floats
- metadata: Additional metadata to store with the vector

**Output**:
- None

**Preconditions**:
- Qdrant collection exists and is accessible
- embedding has correct dimensions
- metadata contains required fields

**Postconditions**:
- Vector is stored in Qdrant with associated metadata
- Vector can be retrieved using similarity search

**Error Conditions**:
- Raises exception if Qdrant connection fails
- Raises exception if vector storage fails
- Raises exception if embedding dimensions are incorrect

## Main Execution Function

### main(target_url: str = "https://physical-ai-and-humanoid-robotics-b-one.vercel.app/", sitemap_url: str = "https://physical-ai-and-humanoid-robotics-b-one.vercel.app/sitemap.xml") -> None

**Purpose**: Execute the complete RAG ingestion pipeline

**Input**:
- target_url: The Docusaurus book URL to process (default: project-specific URL)
- sitemap_url: Optional sitemap URL to discover additional pages (default: project-specific sitemap URL)

**Output**:
- None (results stored in Qdrant)

**Preconditions**:
- All required environment variables are set (API keys, etc.)
- Network connectivity is available
- External services (Cohere, Qdrant) are accessible

**Postconditions**:
- All accessible pages from the target site and sitemap are processed
- Embeddings are stored in Qdrant with proper metadata
- Processed content can be retrieved via similarity search

**Error Conditions**:
- Raises exception if any step of the pipeline fails
- Logs errors for debugging and monitoring