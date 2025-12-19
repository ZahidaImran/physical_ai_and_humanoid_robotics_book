# Implementation Plan: RAG Book Content Ingestion Pipeline

**Feature**: RAG Book Content Ingestion Pipeline
**Created**: 2025-12-18
**Status**: Draft
**Branch**: main

## Technical Context

This implementation will create a backend system to extract content from deployed Docusaurus book URLs, generate embeddings using Cohere, and store them in Qdrant Cloud. The system will support a Target URL for content extraction, with an optional Sitemap URL to enhance URL discovery. The system will be implemented as a single main.py file with specific functions as requested.

**Key technologies:**
- Python backend
- Cohere API for embeddings
- Qdrant Cloud for vector storage
- UV package manager
- Web scraping for content extraction

**Unknowns requiring clarification:**
- Specific Cohere embedding model to use: embed-multilingual-v3.0
- Qdrant Cloud configuration details: cosine distance metric with default settings
- Exact chunk size for text processing: 512-token chunks with 50-token overlap

## Constitution Check

This implementation aligns with the project constitution:

✅ **Clarity & Accessibility**: The code will be well-documented with clear comments and function documentation
✅ **Technical Accuracy**: Using established libraries (Cohere, Qdrant) with proper error handling
✅ **Modularity & Reusability**: Functions will be designed to be reusable and testable
✅ **Version Control & Collaboration**: Code will be committed following project standards

## Gates

### Pre-implementation Gate
- [x] Feature specification approved
- [x] Architecture decisions documented
- [ ] Dependencies and security reviewed
- [ ] Performance requirements validated
- [ ] External service integration confirmed

### Post-implementation Gate
- [ ] All functions tested and working
- [ ] Error handling implemented
- [ ] Security scanning passed
- [ ] Performance benchmarks met

## Phase 0: Research

### Research Tasks

1. **Cohere Embedding Models**: Research the best Cohere embedding model for document content
2. **Qdrant Cloud Setup**: Understand Qdrant Cloud collection creation and configuration
3. **Content Extraction**: Best practices for extracting content from Docusaurus sites
4. **Text Chunking**: Optimal chunk size for embedding generation

### Dependencies to be Resolved

- `cohere` Python package
- `qdrant-client` Python package
- `requests` or `beautifulsoup4` for web scraping
- `python-dotenv` for environment management

## Phase 1: Design & Architecture

### Data Model

**BookContent**:
- url: string (source URL)
- title: string (page title)
- content: string (extracted text content)
- hierarchy: string (document hierarchy path)
- timestamp: datetime (when content was extracted)

**EmbeddingVector**:
- id: string (unique identifier)
- vector: list[float] (embedding values)
- metadata: dict (source URL, title, hierarchy, timestamp)

**Chunk**:
- id: string (chunk identifier)
- content: string (text chunk)
- source_url: string (original URL)
- position: int (position in document)

### API Contracts

The system will be designed as a command-line application with the following main functions:

```
get_all_urls(target_url: str, sitemap_url: str = None) -> List[str]
extract_text_from_url(url: str) -> str
chunk_text(text: str, chunk_size: int) -> List[Chunk]
embed(texts: List[str]) -> List[List[float]]
create_collection(collection_name: str) -> None
save_chunk_to_qdrant(chunk: Chunk, embedding: List[float]) -> None
```

### System Architecture

Single-file Python application (main.py) with the following components:
1. URL discovery from target site and optional sitemap
2. Content extraction and cleaning
3. Text chunking algorithm
4. Cohere embedding generation
5. Qdrant Cloud vector storage
6. Main execution flow orchestrating all components

## Phase 2: Implementation Plan

### File Structure
```
backend/
├── main.py
├── requirements.txt
└── .env
```

### Implementation Steps

1. **Setup Backend Folder**
   - Create backend directory
   - Initialize with UV package management
   - Set up virtual environment

2. **Install Dependencies**
   - Cohere client
   - Qdrant client
   - Web scraping libraries
   - Sitemap parsing libraries
   - Environment variable management

3. **Implement Core Functions**
   - URL discovery function (supporting target URL and optional sitemap)
   - Text extraction function
   - Text chunking function
   - Embedding generation function
   - Qdrant collection creation
   - Vector storage function

4. **Main Execution Flow**
   - Initialize clients
   - Execute pipeline: get URLs (from target and optional sitemap) → extract text → chunk → embed → store
   - Add error handling and logging

### Configuration Requirements

- Environment variables for API keys (COHERE_API_KEY, QDRANT_API_KEY, QDRANT_URL)
- Configuration parameters for chunk size, retry policies, and rate limiting

## Phase 3: Deployment & Operations

### Environment Setup
- API keys for Cohere and Qdrant Cloud
- Proper error handling for network issues
- Logging configuration for monitoring

### Monitoring Requirements
- Success/failure metrics for each pipeline stage
- Performance tracking for embedding generation
- Error logging for debugging

## Risks & Mitigation

### Technical Risks
- **API Rate Limits**: Implement exponential backoff for Cohere and Qdrant API calls
- **Memory Usage**: Process content in batches to handle large books
- **Network Issues**: Implement retry logic with appropriate delays

### Operational Risks
- **Service Availability**: Monitor external service status
- **Data Integrity**: Validate embeddings before storage
- **Cost Management**: Monitor API usage to control costs

## Success Criteria

- [ ] Successfully extract content from target URL: https://physical-ai-and-humanoid-robotics-b-one.vercel.app/
- [ ] Optionally process sitemap URL: https://physical-ai-and-humanoid-robotics-b-one.vercel.app/sitemap.xml for additional URL discovery
- [ ] Generate embeddings with 95%+ success rate
- [ ] Store vectors in Qdrant Cloud with proper metadata
- [ ] Complete pipeline execution within performance targets
- [ ] Handle errors gracefully with appropriate logging