# API Contract: RAG Retrieval Pipeline Validation

## Overview
API contract for the validation functions that verify the RAG retrieval pipeline. These represent the internal function interfaces that will be implemented in validation.py.

## Function: validate_qdrant_connection
**Description**: Validates connection to Qdrant Cloud and verifies access to the specified collection

### Parameters
- `qdrant_url` (string): The Qdrant Cloud URL
- `qdrant_api_key` (string): The Qdrant API key
- `collection_name` (string): Name of the collection to validate (default: "rag_embedding")

### Returns
- `result` (object): Object containing:
  - `connected` (boolean): Whether connection was successful
  - `collection_exists` (boolean): Whether the collection exists
  - `vector_count` (integer): Number of vectors in the collection
  - `collection_config` (object): Configuration details of the collection

### Errors
- `ConnectionError`: If unable to connect to Qdrant
- `AuthenticationError`: If API credentials are invalid
- `NotFoundError`: If the specified collection doesn't exist

## Function: execute_semantic_query
**Description**: Executes a semantic similarity search against the Qdrant collection

### Parameters
- `query_text` (string): The semantic query text
- `collection_name` (string): Name of the collection to search in
- `top_k` (integer, optional): Number of results to return (default: 5)
- `query_filters` (object, optional): Additional filters for the query

### Returns
- `results` (object): Object containing:
  - `query_text` (string): The original query text
  - `retrieved_chunks` (array of objects): Array of retrieved content chunks with:
    - `text` (string): The retrieved text content
    - `similarity_score` (float): Similarity score (0.0 to 1.0)
    - `metadata` (object): Metadata including module, page, heading, URL
    - `vector_id` (string): The vector ID in the store
  - `execution_time_ms` (float): Time taken to execute the query

### Errors
- `QueryError`: If the query format is invalid
- `ConnectionError`: If unable to connect to Qdrant during query
- `QdrantIndexError`: If the collection index is corrupted

## Function: validate_retrieved_results
**Description**: Validates that retrieved results contain correct text and properly preserved metadata

### Parameters
- `retrieved_chunks` (array of objects): Array of chunks retrieved from the vector store
- `query_text` (string): The original query text for context
- `expected_content_types` (array of strings, optional): Expected types of content to be retrieved

### Returns
- `validation_report` (object): Object containing:
  - `content_accuracy` (float): Accuracy of text content preservation (0.0 to 1.0)
  - `metadata_completeness` (float): Completeness of metadata preservation (0.0 to 1.0)
  - `relevance_score` (float): How relevant the results are to the query (0.0 to 1.0)
  - `issues_found` (array of objects): Array of validation issues found:
    - `type` (string): Type of issue (e.g., "missing_metadata", "content_corruption")
    - `severity` (string): Severity level ("critical", "warning", "info")
    - `details` (string): Detailed description of the issue

### Errors
- `ValidationError`: If validation parameters are invalid
- `IntegrityError`: If retrieved content doesn't match expected format

## Function: confirm_pipeline_readiness
**Description**: Confirms the entire retrieval pipeline is ready for agent integration

### Parameters
- `validation_results` (array of objects): Array of validation results from multiple test queries
- `minimum_accuracy_threshold` (float, optional): Minimum accuracy required (default: 0.9)
- `minimum_metadata_completeness` (float, optional): Minimum metadata completeness (default: 0.95)

### Returns
- `readiness_report` (object): Object containing:
  - `is_ready_for_agent_integration` (boolean): Whether pipeline meets readiness criteria
  - `overall_accuracy` (float): Overall accuracy across all validation tests
  - `metadata_preservation_rate` (float): Overall metadata preservation rate
  - `recommendations` (array of strings): Recommendations for improvements if not ready
  - `confidence_level` (string): Confidence level ("high", "medium", "low")

### Errors
- `ReadinessError`: If readiness assessment cannot be completed
- `InsufficientTestDataError`: If not enough validation results to assess readiness

## Function: main
**Description**: Main validation function that orchestrates the complete pipeline validation

### Parameters
- `qdrant_url` (string, optional): Qdrant Cloud URL (defaults to environment variable)
- `qdrant_api_key` (string, optional): Qdrant API key (defaults to environment variable)
- `cohere_api_key` (string, optional): Cohere API key (defaults to environment variable)
- `collection_name` (string, optional): Collection name to validate (default: "rag_embedding")
- `test_queries` (array of strings, optional): Custom test queries to use for validation

### Returns
- `final_report` (object): Object containing:
  - `connection_status` (object): Status of Qdrant connection validation
  - `query_results` (array of objects): Results from semantic query validation
  - `validation_findings` (object): Summary of validation results
  - `readiness_assessment` (object): Final readiness assessment for agent integration
  - `execution_summary` (object): Summary of execution including timing and errors

### Errors
- `PipelineValidationError`: If any step in the validation pipeline fails