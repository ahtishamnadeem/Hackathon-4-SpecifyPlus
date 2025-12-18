# API Contract: RAG Ingestion Pipeline

## Overview
API contract for the RAG ingestion pipeline functions. These represent the internal function interfaces that will be implemented in main.py.

## Function: get_all_urls
**Description**: Discovers and returns all valid URLs from the book website using sitemap.xml

### Parameters
- `base_url` (string): The root URL to start crawling from (default: "https://sigma-hackathon-4-specify-plus.vercel.app/")

### Returns
- `urls` (array of strings): List of discovered URLs from the book website

### Errors
- `ConnectionError`: If the base URL is not accessible
- `SitemapParseError`: If the sitemap.xml cannot be parsed

## Function: extract_text_from_url
**Description**: Extracts clean text content from a given URL, preserving metadata

### Parameters
- `url` (string): The URL to extract content from

### Returns
- `result` (object): Object containing:
  - `text` (string): Clean text content from the page
  - `title` (string): Page title
  - `module` (string): Module name extracted from URL or metadata
  - `page` (string): Page name extracted from URL or metadata
  - `headings` (array of strings): List of headings in the content

### Errors
- `ConnectionError`: If the URL is not accessible
- `ContentExtractionError`: If content cannot be extracted from the page

## Function: chunk_text
**Description**: Splits text into appropriately sized chunks while preserving context and metadata

### Parameters
- `text` (string): The text to be chunked
- `url` (string): The source URL for metadata
- `title` (string): The page title for metadata
- `module` (string): The module name for metadata
- `page` (string): The page name for metadata
- `headings` (array of strings): List of headings for context preservation
- `chunk_size` (integer, optional): Target size for chunks in tokens (default: 800)
- `chunk_overlap` (integer, optional): Overlap between chunks in tokens (default: 160)

### Returns
- `chunks` (array of objects): Array of chunk objects containing:
  - `text` (string): The chunked text
  - `metadata` (object): Metadata including URL, title, module, page, and relevant heading

### Errors
- `InvalidChunkSizeError`: If chunk size parameters are invalid

## Function: embed
**Description**: Generates embeddings for text chunks using Cohere

### Parameters
- `texts` (array of strings): Array of text chunks to embed
- `model` (string, optional): Cohere embedding model to use (default: "embed-english-v3.0")

### Returns
- `embeddings` (array of arrays): Array of embedding vectors

### Errors
- `APIError`: If Cohere API call fails
- `RateLimitError`: If API rate limits are exceeded

## Function: create_collection
**Description**: Creates a Qdrant collection named "rag_embedding" for storing embeddings

### Parameters
- `collection_name` (string): Name of the collection to create (default: "rag_embedding")
- `vector_size` (integer): Size of the embedding vectors (default: 1024 for Cohere multilingual v3.0)

### Returns
- `success` (boolean): True if collection was created or already exists

### Errors
- `ConnectionError`: If unable to connect to Qdrant
- `ConfigurationError`: If collection parameters are invalid

## Function: save_chunk_to_qdrant
**Description**: Saves a text chunk with embedding to Qdrant with metadata

### Parameters
- `chunk` (object): Chunk object with text and metadata
- `embedding` (array of floats): The embedding vector
- `collection_name` (string): Name of the collection to save to (default: "rag_embedding")

### Returns
- `record_id` (string): ID of the saved record in Qdrant

### Errors
- `ConnectionError`: If unable to connect to Qdrant
- `StorageError`: If the record could not be saved

## Function: main
**Description**: Main function that orchestrates the complete ingestion pipeline

### Parameters
- `base_url` (string, optional): The root URL to process (default: "https://sigma-hackathon-4-specify-plus.vercel.app/")
- `chunk_size` (integer, optional): Target size for chunks in tokens (default: 800)
- `chunk_overlap` (integer, optional): Overlap between chunks in tokens (default: 160)

### Returns
- `result` (object): Object containing:
  - `total_chunks_processed` (integer): Number of chunks successfully processed
  - `total_embeddings_stored` (integer): Number of embeddings successfully stored in Qdrant

### Errors
- `PipelineError`: If any step in the pipeline fails