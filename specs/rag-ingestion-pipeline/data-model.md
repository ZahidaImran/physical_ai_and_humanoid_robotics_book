# Data Model: RAG Book Content Ingestion Pipeline

**Feature**: RAG Book Content Ingestion Pipeline
**Created**: 2025-12-18
**Status**: Draft

## Entities

### BookContent
Represents extracted text content from Docusaurus book pages

**Fields**:
- id: string (auto-generated unique identifier)
- url: string (source URL of the page)
- title: string (page title extracted from HTML)
- content: string (cleaned text content from the page)
- hierarchy: string (document hierarchy path, e.g., "section/subsection/page")
- created_at: datetime (timestamp when content was extracted)
- updated_at: datetime (timestamp when content was last updated)

**Validation Rules**:
- url must be a valid URL format
- content must not be empty
- created_at must be in the past

### Chunk
Represents a segment of text content suitable for embedding generation

**Fields**:
- id: string (auto-generated unique identifier)
- content: string (text chunk, max 512 tokens)
- source_url: string (original URL this chunk came from)
- source_id: string (reference to BookContent.id)
- position: integer (position of this chunk in the original document)
- created_at: datetime (timestamp when chunk was created)

**Validation Rules**:
- content must be between 10 and 512 tokens
- position must be non-negative
- source_url must reference an existing BookContent

### EmbeddingVector
Represents the vector embedding of text content

**Fields**:
- id: string (auto-generated unique identifier)
- vector: list[float] (numerical embedding values, dimension: 1024 for Cohere multilingual model)
- chunk_id: string (reference to Chunk.id)
- metadata: dict (additional information stored with the vector)
  - source_url: string (original URL)
  - title: string (page title)
  - hierarchy: string (document hierarchy)
  - position: integer (position in document)
- created_at: datetime (timestamp when embedding was generated)

**Validation Rules**:
- vector must have consistent dimensions (1024 for Cohere model)
- chunk_id must reference an existing Chunk
- metadata must include required fields

### PipelineConfiguration
Parameters controlling the ingestion pipeline

**Fields**:
- id: string (configuration identifier)
- chunk_size: integer (size of text chunks in tokens, default: 512)
- chunk_overlap: integer (overlap between chunks in tokens, default: 50)
- cohere_model: string (embedding model name, default: "embed-multilingual-v3.0")
- batch_size: integer (number of chunks to process in parallel, default: 10)
- max_retries: integer (maximum retry attempts for failed operations, default: 3)
- delay_base: float (base delay for exponential backoff, default: 1.0)
- created_at: datetime (timestamp when configuration was created)
- updated_at: datetime (timestamp when configuration was last updated)

**Validation Rules**:
- chunk_size must be between 100 and 1024
- chunk_overlap must be less than chunk_size
- max_retries must be non-negative
- delay_base must be positive

## Relationships

```
BookContent (1) → (N) Chunk
Chunk (1) → (1) EmbeddingVector
PipelineConfiguration (1) → (N) EmbeddingVector (via processing)
```

## State Transitions

### BookContent State Transitions
- EXTRACTED → CHUNKED (when text is split into chunks)
- CHUNKED → EMBEDDED (when embeddings are generated)
- EMBEDDED → STORED (when vectors are saved to Qdrant)

### Processing Status
Each entity can have an associated processing status:
- PENDING: Awaiting processing
- PROCESSING: Currently being processed
- COMPLETED: Successfully processed
- FAILED: Processing failed
- RETRYING: Processing will be retried