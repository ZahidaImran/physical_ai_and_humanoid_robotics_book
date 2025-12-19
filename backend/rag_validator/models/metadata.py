"""
Chunk metadata model for RAG retrieval validation system.
"""
from dataclasses import dataclass
from typing import Optional
import re


@dataclass
class ChunkMetadata:
    """
    Information associated with each content chunk including URL, section, chapter, and chunk ID.
    """
    url: str
    section: str
    chapter: str
    chunk_id: str
    source_file: Optional[str] = None
    page_number: Optional[int] = None

    def __post_init__(self):
        """
        Validate the chunk metadata after initialization.
        """
        if not self.url.strip():
            raise ValueError("URL must not be empty")

        # Basic URL validation
        url_pattern = re.compile(
            r'^https?://'  # http:// or https://
            r'(?:(?:[A-Z0-9](?:[A-Z0-9-]{0,61}[A-Z0-9])?\.)+[A-Z]{2,6}\.?|'  # domain...
            r'localhost|'  # localhost...
            r'\d{1,3}\.\d{1,3}\.\d{1,3}\.\d{1,3})'  # ...or ip
            r'(?::\d+)?'  # optional port
            r'(?:/?|[/?]\S+)$', re.IGNORECASE)

        if not url_pattern.match(self.url):
            raise ValueError(f"Invalid URL format: {self.url}")

        if not self.section.strip():
            raise ValueError("Section must not be empty")

        if not self.chapter.strip():
            raise ValueError("Chapter must not be empty")

        if not self.chunk_id.strip():
            raise ValueError("Chunk ID must not be empty")

        if self.page_number is not None and self.page_number <= 0:
            raise ValueError("Page number must be positive if provided")