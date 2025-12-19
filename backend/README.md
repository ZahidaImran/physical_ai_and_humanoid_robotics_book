# RAG Book Content Ingestion Pipeline

This module implements a pipeline to:
1. Extract content from deployed Docusaurus book URLs
2. Generate embeddings using Cohere
3. Store vectors and metadata in Qdrant Cloud

## Prerequisites

- Python 3.9 or higher
- UV package manager
- Access to Cohere API (API key)
- Access to Qdrant Cloud (API key and URL)

## Installation

1. Create a virtual environment:
   ```bash
   python -m venv .venv
   source .venv/bin/activate  # On Windows: .venv\Scripts\activate
   ```

2. Install dependencies using UV:
   ```bash
   uv pip install -r requirements.txt
   ```

3. Set up environment variables in `.env`:
   ```env
   COHERE_API_KEY=your_cohere_api_key_here
   QDRANT_API_KEY=your_qdrant_api_key_here
   QDRANT_URL=your_qdrant_cluster_url_here
   CHUNK_SIZE=512
   CHUNK_OVERLAP=50
   COHERE_MODEL=embed-multilingual-v3.0
   BATCH_SIZE=10
   MAX_RETRIES=3
   DELAY_BASE=1.0
   ```

## Usage

### Basic Usage
```bash
python main.py
```

### With Custom Parameters
```bash
python main.py --target-url "https://your-doc-site.com" --sitemap-url "https://your-doc-site.com/sitemap.xml" --chunk-size 1024 --verbose
```

### Available Options
- `--target-url`: The Docusaurus book URL to process (default: https://physical-ai-and-humanoid-robotics-b-one.vercel.app/)
- `--sitemap-url`: Optional sitemap URL to discover additional pages (default: https://physical-ai-and-humanoid-robotics-b-one.vercel.app/sitemap.xml)
- `--force-update`: Force update even if content hasn't changed
- `--chunk-size`: Override the chunk size from environment variables
- `--chunk-overlap`: Override the chunk overlap from environment variables
- `--batch-size`: Override the batch size from environment variables
- `--validate`: Run validation after ingestion
- `--verbose`: Enable verbose logging

## Features

- **URL Discovery**: Extracts all accessible pages from Docusaurus sites with optional sitemap support
- **Content Extraction**: Cleanly extracts text content while preserving document hierarchy and metadata
- **Text Chunking**: Splits content into configurable chunks with overlap
- **Embedding Generation**: Creates high-quality embeddings using Cohere API
- **Vector Storage**: Stores embeddings with metadata in Qdrant Cloud
- **Configuration**: Flexible configuration via environment variables and command-line arguments
- **Monitoring**: Comprehensive logging and progress tracking
- **Error Handling**: Robust error handling with retry mechanisms
- **Content Validation**: Integrity checks and change detection
- **Memory Management**: Monitors memory usage during processing
- **Post-Processing Validation**: Validates stored embeddings and runs sample searches

## Architecture

The pipeline consists of the following main components:

1. **URL Discovery**: `get_all_urls()` - Discovers all accessible pages
2. **Content Extraction**: `extract_text_from_url()` - Extracts clean text content
3. **Metadata Extraction**: `extract_page_metadata()` - Extracts page metadata
4. **Text Chunking**: `chunk_text()` - Splits content into chunks
5. **Embedding Generation**: `embed()` - Creates vector embeddings
6. **Vector Storage**: `save_chunk_to_qdrant()` - Stores vectors in Qdrant
7. **Validation**: `validate_stored_embeddings()` - Validates stored data
8. **Search**: `search_similar_chunks()` - Performs similarity search

## Configuration

The pipeline can be configured through:

1. Environment variables in `.env`
2. Command-line arguments
3. The `PipelineConfig` class

Key configuration parameters:
- `CHUNK_SIZE`: Size of text chunks in tokens (default: 512)
- `CHUNK_OVERLAP`: Overlap between chunks in tokens (default: 50)
- `BATCH_SIZE`: Number of chunks to process in parallel (default: 10)
- `MAX_RETRIES`: Maximum retry attempts for failed operations (default: 3)
- `COHERE_MODEL`: Cohere embedding model to use (default: embed-multilingual-v3.0)

## Validation and Testing

Run the pipeline with validation:
```bash
python main.py --validate
```

This will:
1. Validate stored embeddings after ingestion
2. Run sample similarity searches
3. Report on the success of validation steps

## Error Recovery

The pipeline includes several error recovery mechanisms:
- Exponential backoff for API calls
- Individual chunk processing when batch processing fails
- Content integrity validation
- Detailed logging for debugging