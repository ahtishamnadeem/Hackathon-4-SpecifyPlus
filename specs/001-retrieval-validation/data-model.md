# Data Model: RAG Retrieval Pipeline Validation

## Overview
Data models for the validation system that verifies semantic retrieval of book content from the Qdrant vector store.

## Entity: Query Request
**Description**: A semantic search query sent to the vector store for validation
**Fields**:
- `query_text` (string): The semantic query text to search for
- `top_k` (integer): Number of results to retrieve (default: 5)
- `query_metadata` (object): Additional query parameters (filters, weights, etc.)

**Validation rules**:
- `query_text` must not be empty
- `top_k` must be between 1 and 100

## Entity: Retrieved Chunk
**Description**: A content chunk returned from the vector store during validation
**Fields**:
- `text` (string): The retrieved text content
- `similarity_score` (float): The semantic similarity score (0.0 to 1.0)
- `metadata` (object): Metadata object containing:
  - `module` (string): The book module
  - `page` (string): The page title
  - `heading` (string): The section heading
  - `url` (string): The source URL
- `vector_id` (string): The unique identifier in the vector store

**Validation rules**:
- `text` must not be empty
- `similarity_score` must be between 0.0 and 1.0
- `vector_id` must match expected format

## Entity: Validation Result
**Description**: Result of a validation operation
**Fields**:
- `query_request` (object): The original query that was validated
- `retrieved_chunks` (array of objects): Array of retrieved chunks
- `connection_validated` (boolean): Whether Qdrant connection succeeded
- `semantic_accuracy` (float): Overall accuracy of semantic retrieval (0.0 to 1.0)
- `metadata_completeness` (float): Percentage of metadata fields correctly preserved
- `validation_timestamp` (timestamp): When validation was performed
- `errors` (array of strings): Any errors encountered during validation

**Validation rules**:
- `semantic_accuracy` must be between 0.0 and 1.0
- `metadata_completeness` must be between 0.0 and 1.0
- `retrieved_chunks` array must contain at least one valid chunk for successful validation

## Entity: Pipeline Status
**Description**: Overall status of the retrieval pipeline validation
**Fields**:
- `is_ready_for_agent_integration` (boolean): Whether pipeline is ready for agent usage
- `last_validation_time` (timestamp): Time of last validation run
- `validation_summary` (object): Summary of validation results including:
  - `total_queries_tested` (integer): Number of queries executed
  - `successful_retrievals` (integer): Number of successful retrievals
  - `average_similarity_score` (float): Average similarity score across all queries
  - `metadata_accuracy_rate` (float): Percentage of metadata correctly preserved
- `issues_found` (array of strings): Any issues discovered during validation

**Validation rules**:
- `average_similarity_score` must be between 0.0 and 1.0
- `metadata_accuracy_rate` must be between 0.0 and 1.0
- `total_queries_tested` must be non-negative
- `successful_retrievals` must be non-negative and not exceed `total_queries_tested`