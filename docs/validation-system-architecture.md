# RAG Retrieval Pipeline Validation System

This system validates the RAG (Retrieval-Augmented Generation) retrieval pipeline by connecting to Qdrant Cloud and executing semantic queries against the existing `rag_embedding` collection. The system verifies that semantic queries return relevant content chunks with correct text and metadata, confirming the pipeline is ready for agent integration.

## Architecture Overview

The validation system consists of several key components:

### 1. Connection Management
- **Qdrant Client**: Connects to Qdrant Cloud and validates access to the `rag_embedding` collection
- **Cohere Client**: Generates embeddings for semantic queries using the same model as the original ingestion

### 2. Core Validation Functions
- **`validate_qdrant_connection`**: Verifies connection to Qdrant Cloud and collection accessibility
- **`execute_semantic_query`**: Executes semantic similarity search against the vector store
- **`validate_retrieved_results`**: Validates content accuracy and metadata completeness
- **`confirm_pipeline_readiness`**: Assesses overall pipeline readiness for agent integration

### 3. Supporting Components
- Environment variable management with validation
- Comprehensive error handling with custom exception classes
- Logging system for monitoring and debugging
- Command-line interface for flexible execution

## Data Flow

1. **Initialization**: Load environment variables and initialize Qdrant and Cohere clients
2. **Connection Validation**: Verify access to Qdrant Cloud and the `rag_embedding` collection
3. **Query Execution**: For each test query:
   - Generate embedding using Cohere
   - Execute semantic search in Qdrant
   - Retrieve top-k most similar content chunks
4. **Result Validation**: Validate content accuracy and metadata completeness
5. **Readiness Assessment**: Generate final pipeline readiness report

## Configuration

### Environment Variables

Create a `.env` file in the root directory with the following variables:

```env
QDRANT_URL=your_qdrant_cloud_url_here
QDRANT_API_KEY=your_qdrant_api_key_here
COHERE_API_KEY=your_cohere_api_key_here
CHUNK_SIZE=800
CHUNK_OVERLAP=160
RATE_LIMIT=1
COHERE_MODEL=embed-english-v3.0
VALIDATION_TOP_K=5
SIMILARITY_THRESHOLD=0.7
```

### Constants

- `CHUNK_SIZE`: Size of text chunks for processing (default: 800)
- `CHUNK_OVERLAP`: Overlap between chunks (default: 160)
- `RATE_LIMIT`: Delay between API calls in seconds (default: 1)
- `COHERE_MODEL`: Cohere model for embeddings (default: embed-english-v3.0)
- `VALIDATION_TOP_K`: Number of results to retrieve (default: 5)
- `SIMILARITY_THRESHOLD`: Minimum similarity score (default: 0.7)

## Usage

### Command Line Interface

```bash
# Run with default settings
python validation.py

# Run with custom parameters
python validation.py --collection-name "rag_embedding" --top-k 5

# Run with specific queries
python validation.py --query "What is ROS 2?" --query "How to create a node?"
```

### Programmatic Usage

```python
from validation import main

# Run validation with default settings
result = main()

# Run validation with custom parameters
result = main(
    qdrant_url="https://your-qdrant-url",
    qdrant_api_key="your-api-key",
    cohere_api_key="your-cohere-key",
    collection_name="rag_embedding",
    test_queries=["What is ROS 2?", "How to create a node?"],
    top_k=5
)
```

## Output Format

The validation system produces a comprehensive report with:

- Connection status to Qdrant Cloud
- Number of vectors in the collection
- Query execution results with timing
- Content accuracy metrics
- Metadata completeness metrics
- Pipeline readiness assessment
- Confidence level and recommendations

## Error Handling

The system includes comprehensive error handling with custom exception classes:

- `ConnectionError`: Network or connection issues
- `ConfigurationError`: Missing or invalid configuration
- `PipelineError`: General pipeline execution failures
- `InsufficientTestDataError`: Not enough data for validation

## Validation Criteria

The system assesses pipeline readiness based on:

- **Content Accuracy**: Minimum 90% of retrieved chunks have valid content
- **Metadata Completeness**: Minimum 95% of chunks have complete metadata
- **Relevance Score**: Average similarity score above 0.5

## Integration Readiness

The system provides a clear readiness assessment with:
- Boolean flag for agent integration readiness
- Confidence level (high, medium, low)
- Specific recommendations for improvement
- Overall performance metrics