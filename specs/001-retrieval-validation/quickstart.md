# Quickstart: RAG Retrieval Pipeline Validation

## Overview
Quick setup guide to run validation on the RAG retrieval pipeline that connects to Qdrant Cloud and verifies semantic retrieval of book content.

## Prerequisites
- Python 3.11 or higher
- Qdrant Cloud API key and URL
- Cohere API key
- Access to the `rag_embedding` collection in Qdrant

## Setup

### 1. Navigate to backend directory
```bash
cd backend
```

### 2. Install dependencies
```bash
pip install -r requirements.txt
```

### 3. Configure environment variables
Copy the example environment file and add your credentials:
```bash
cp .env.example .env
```

Edit `.env` and add:
```env
QDRANT_URL=your_qdrant_cloud_url_here
QDRANT_API_KEY=your_qdrant_api_key_here
COHERE_API_KEY=your_cohere_api_key_here
```

## Usage

### 1. Run the validation pipeline
```bash
python validation.py
```

The script will:
1. Connect to Qdrant Cloud and verify access to `rag_embedding` collection
2. Execute sample semantic queries to test retrieval
3. Validate retrieved chunks for content accuracy and metadata preservation
4. Generate a validation report with readiness status

### 2. Run with custom parameters
```bash
python validation.py --collection-name "rag_embedding" --top-k 5 --queries-file "test_queries.txt"
```

### 3. Run validation tests
```bash
python -m pytest tests/test_validation.py
python -m pytest tests/test_integration.py
```

## Configuration
The validation can be configured by modifying environment variables in `.env`:
- `QDRANT_URL`: Qdrant Cloud instance URL
- `QDRANT_API_KEY`: Qdrant Cloud API key
- `COHERE_API_KEY`: Cohere API key for query embeddings
- `VALIDATION_TOP_K`: Number of results to retrieve for validation (default: 5)
- `SIMILARITY_THRESHOLD`: Minimum similarity score for relevant results (default: 0.7)

## Sample Output
On successful validation, you'll see:
```
✅ Qdrant connection established successfully
✅ Collection 'rag_embedding' verified with 150 vectors
✅ Semantic queries returned relevant results
✅ Metadata preserved correctly: 100% accuracy
✅ Pipeline ready for agent integration: YES
```

## Troubleshooting

### Connection Issues
If you encounter connection issues:
- Verify your QDRANT_URL and QDRANT_API_KEY are correct
- Check that your network allows outbound connections to Qdrant Cloud
- Ensure the `rag_embedding` collection exists and is accessible

### Query Issues
If semantic queries return unexpected results:
- Verify the Cohere API key is valid and has sufficient quota
- Check that the query embedding model matches the one used for ingestion
- Ensure the Qdrant collection contains properly indexed content