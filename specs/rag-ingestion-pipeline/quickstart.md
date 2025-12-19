# Quickstart Guide: RAG Book Content Ingestion Pipeline

**Feature**: RAG Book Content Ingestion Pipeline
**Created**: 2025-12-18
**Status**: Draft

## Overview

This guide provides instructions to quickly set up and run the RAG ingestion pipeline that extracts content from Docusaurus book URLs, generates embeddings using Cohere, and stores them in Qdrant Cloud.

## Prerequisites

- Python 3.9 or higher
- UV package manager
- Access to Cohere API (API key)
- Access to Qdrant Cloud (API key and URL)

## Setup Instructions

### 1. Clone or Navigate to Project Directory

```bash
cd /path/to/your/project
```

### 2. Create Backend Directory

```bash
mkdir backend
cd backend
```

### 3. Initialize Python Project with UV

```bash
# Create virtual environment
python -m venv .venv
source .venv/bin/activate  # On Windows: .venv\Scripts\activate

# Install uv if not already installed
pip install uv
```

### 4. Create Requirements File

Create `requirements.txt` with the following content:

```txt
cohere==5.5.2
qdrant-client==1.9.1
requests==2.31.0
beautifulsoup4==4.12.2
lxml==4.9.4
python-dotenv==1.0.0
```

Install dependencies using UV:

```bash
uv pip install -r requirements.txt
```

### 5. Set Up Environment Variables

Create a `.env` file in the backend directory with the following content:

```env
COHERE_API_KEY=your_cohere_api_key_here
QDRANT_API_KEY=your_qdrant_api_key_here
QDRANT_URL=your_qdrant_cluster_url_here
CHUNK_SIZE=512
CHUNK_OVERLAP=50
COHERE_MODEL=embed-multilingual-v3.0
```

**Note**: Replace the placeholder values with your actual API keys and URLs.

### 6. Create the Main Application

Create `main.py` with the implementation of the RAG ingestion pipeline containing the required functions:
- `get_all_urls`
- `extract_text_from_url`
- `chunk_text`
- `embed`
- `create_collection`
- `save_chunk_to_qdrant`
- Main execution function

### 7. Run the Pipeline

```bash
cd backend
python main.py
```

## Configuration Options

### Environment Variables

- `COHERE_API_KEY`: Your Cohere API key for embedding generation
- `QDRANT_API_KEY`: Your Qdrant Cloud API key
- `QDRANT_URL`: Your Qdrant Cloud cluster URL
- `CHUNK_SIZE`: Size of text chunks in tokens (default: 512)
- `CHUNK_OVERLAP`: Overlap between chunks in tokens (default: 50)
- `COHERE_MODEL`: Cohere embedding model to use (default: embed-multilingual-v3.0)

### Custom Target URL and Sitemap

To process a different Docusaurus site, modify the target URL and optional sitemap URL in the main function call or add command-line arguments. By default, the pipeline will process:

- Target URL: https://physical-ai-and-humanoid-robotics-b-one.vercel.app/
- Sitemap URL: https://physical-ai-and-humanoid-robotics-b-one.vercel.app/sitemap.xml (optional)

## Expected Output

When running successfully, the pipeline will:

1. Extract all accessible URLs from the target Docusaurus site and optionally from the sitemap URL
2. Download and extract text content from each page
3. Split content into appropriately sized chunks
4. Generate embeddings for each chunk using Cohere
5. Store embeddings in Qdrant Cloud with metadata
6. Log progress and any errors encountered

## Troubleshooting

### Common Issues

1. **API Key Errors**: Verify that your Cohere and Qdrant API keys are correct and have sufficient permissions
2. **Network Issues**: Check your internet connection and firewall settings
3. **Rate Limits**: If you encounter rate limit errors, the system implements exponential backoff to handle them automatically
4. **SSL Certificate Issues**: Ensure your system has up-to-date certificates

### Logging

The application logs progress and errors to the console. For detailed debugging, check for any error messages during execution.

## Next Steps

After successful setup:
1. Verify embeddings are stored correctly in your Qdrant Cloud collection
2. Test similarity search functionality
3. Set up monitoring for ongoing pipeline runs
4. Consider setting up scheduled runs for content updates