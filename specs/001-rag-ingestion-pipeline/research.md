# Research: RAG Ingestion Pipeline

## Overview
Research for implementing a backend service that extracts, chunks, and embeds content from the deployed book website using Cohere embeddings and stores in Qdrant vector database. The implementation will be in a single main.py file with specific functions as required.

## Technology Decisions

### Python Package Management: UV
**Decision**: Use UV package manager for Python dependency management
**Rationale**: UV is a fast, modern Python package manager written in Rust that offers superior performance compared to pip. It's designed to be a drop-in replacement for pip/pip-tools/virtualenv workflows with significantly faster operations.
**Alternatives considered**:
- pip + venv: Standard but slower
- Poetry: Feature-rich but potentially overkill for this project
- Conda: Good for data science but heavier than needed

### Cohere for Embeddings
**Decision**: Use Cohere's embedding models for text vectorization
**Rationale**: Cohere provides high-quality embeddings specifically designed for retrieval tasks. Their models are well-documented, have good performance, and offer reliable APIs with reasonable rate limits for development.
**Model choice**: `embed-english-v3.0` - Cohere's latest English embedding model optimized for retrieval tasks
**Alternatives considered**:
- OpenAI embeddings: Good quality but higher cost
- Sentence Transformers (local): Free but requires more computational resources
- Google's embedding models: Good alternative but Cohere is more established for RAG

### Qdrant for Vector Storage
**Decision**: Use Qdrant Cloud as the vector database with collection named "rag_embedding"
**Rationale**: Qdrant is a high-performance vector database with excellent Python client support. It offers semantic search capabilities, efficient storage of embeddings with metadata, and scalable cloud hosting.
**Configuration**:
- Collection name: `rag_embedding` (as specified in requirements)
- Vector size: 1024 (for Cohere multilingual v3.0 model)
- Payload: Store text content with metadata (module, page, heading, URL)
**Alternatives considered**:
- Pinecone: Good alternative but more expensive
- Weaviate: Good functionality but Qdrant has simpler setup
- Chroma: Open source but less scalable than cloud solutions

### Web Scraping Approach
**Decision**: Use requests + BeautifulSoup for content extraction with sitemap.xml for URL discovery
**Rationale**: This combination provides reliable HTML parsing capabilities and handles various HTML structures well. It's lightweight and perfect for extracting text content from static sites like the deployed Docusaurus book.
**URL Discovery Strategy**: Use sitemap.xml at https://sigma-hackathon-4-specify-plus.vercel.app/sitemap.xml to discover all book pages
**Considerations**:
- Need to respect robots.txt and implement rate limiting
- Handle relative vs absolute URLs appropriately
- Extract headings and metadata alongside main content

## Text Chunking Strategy
**Decision**: Use a token-based chunking approach with semantic awareness
**Rationale**: To maintain context while ensuring chunks fit within embedding model limits, we'll split content into chunks with overlap to preserve context across boundaries.
**Parameters**:
- Chunk size: 800 tokens (approximately 600-700 words)
- Overlap: 160 tokens (20% of chunk size)
- Minimum chunk size: 100 tokens to ensure meaningful content
- Semantic boundaries: Try to chunk at paragraph or section breaks when possible

## Metadata Preservation
**Decision**: Store module, page, heading, and URL as payload in Qdrant
**Rationale**: This preserves the original context needed for downstream RAG applications to properly attribute and reference sources.
**Metadata fields**:
- `module`: The book module (e.g., "Module 1: The Robotic Nervous System")
- `page`: The specific page title
- `heading`: The section heading where the chunk originated
- `url`: The full URL to the source content
- `text`: The actual content chunk

## Function Design Specifications
Based on the requirements, the main.py file will contain these specific functions:

1. **get_all_urls()**: Fetch all URLs from the book website using sitemap.xml
2. **extract_text_from_url()**: Extract clean text content from a given URL, preserving headings and metadata
3. **chunk_text()**: Split text into appropriately sized chunks with metadata preservation
4. **embed()**: Generate embeddings using Cohere for text chunks
5. **create_collection(name="rag_embedding")**: Create Qdrant collection with proper configuration
6. **save_chunk_to_qdrant()**: Save a chunk with embedding to Qdrant with metadata
7. **main()**: Orchestrate the complete pipeline execution

## Error Handling Strategy
**Decision**: Implement comprehensive error handling for API limits, connection issues, and content extraction failures
**Approach**:
- Implement exponential backoff for Cohere API rate limits
- Add connection retry logic for web scraping
- Include content validation to ensure quality before embedding
- Log errors with sufficient context for debugging