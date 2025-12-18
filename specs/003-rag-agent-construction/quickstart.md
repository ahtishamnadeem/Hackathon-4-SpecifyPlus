# Quickstart: RAG Agent Construction

## Overview
Quick setup guide for the RAG (Retrieval-Augmented Generation) agent that connects to Qdrant Cloud and processes user queries using the existing `rag_embedding` collection.

## Prerequisites
- Python 3.11 or higher
- Access to Qdrant Cloud with the `rag_embedding` collection
- OpenAI API key for the language model
- Git for version control

## Setup

### 1. Clone and Navigate to Backend Directory
```bash
cd backend/
```

### 2. Install Dependencies
```bash
pip install fastapi uvicorn openai qdrant-client python-dotenv pydantic pytest
```

### 3. Create Environment Configuration
Create a `.env` file in the backend directory with the following variables:
```env
# Qdrant Configuration
QDRANT_URL=your_qdrant_cloud_url_here
QDRANT_API_KEY=your_qdrant_api_key_here

# OpenAI Configuration
OPENAI_API_KEY=your_openai_api_key_here

# Agent Configuration
MODEL_NAME=gpt-4-turbo
COLLECTION_NAME=rag_embedding
MAX_RETRIEVALS=5
TEMPERATURE=0.7
MAX_TOKENS=500

# Server Configuration
HOST=0.0.0.0
PORT=8000
```

### 4. Run the Agent Service
```bash
uvicorn main:app --host 0.0.0.0 --port 8000 --reload
```

## Usage

### Query the Agent
Make a POST request to the `/query` endpoint:

```bash
curl -X POST "http://localhost:8000/query" \
  -H "Content-Type: application/json" \
  -H "X-API-Key: your-api-key" \
  -d '{
    "query": "What are the key concepts in ROS 2?",
    "max_tokens": 300,
    "temperature": 0.5,
    "include_sources": true
  }'
```

### Check Service Health
```bash
curl -X GET "http://localhost:8000/health"
```

### Get Agent Information
```bash
curl -X GET "http://localhost:8000/info"
```

## Configuration Options

### Environment Variables
- `QDRANT_URL`: URL for your Qdrant Cloud instance
- `QDRANT_API_KEY`: API key for Qdrant access
- `OPENAI_API_KEY`: API key for OpenAI services
- `MODEL_NAME`: OpenAI model to use (default: gpt-4-turbo)
- `COLLECTION_NAME`: Qdrant collection name (default: rag_embedding)
- `MAX_RETRIEVALS`: Maximum number of chunks to retrieve (default: 5)
- `TEMPERATURE`: Response creativity (default: 0.7)
- `MAX_TOKENS`: Maximum response length (default: 500)
- `HOST`: Server host (default: 0.0.0.0)
- `PORT`: Server port (default: 8000)

## Testing

### Run Unit Tests
```bash
python -m pytest tests/test_agent.py -v
```

### Run Integration Tests
```bash
python -m pytest tests/test_api.py -v
```

### Run All Tests
```bash
python -m pytest tests/ -v
```

## Troubleshooting

### Common Issues

1. **Qdrant Connection Error**
   - Verify `QDRANT_URL` and `QDRANT_API_KEY` are correct
   - Check that the `rag_embedding` collection exists
   - Ensure network connectivity to Qdrant Cloud

2. **OpenAI API Error**
   - Verify `OPENAI_API_KEY` is valid and has sufficient quota
   - Check that the specified model is available

3. **Slow Response Times**
   - Verify Qdrant Cloud performance settings
   - Check network connectivity between services
   - Consider adjusting `MAX_RETRIEVALS` to reduce retrieval time

### Health Check
If experiencing issues, run the health check:
```bash
curl -X GET "http://localhost:8000/health"
```

## Next Steps
1. Integrate the RAG agent into your application using the API endpoints
2. Monitor response quality and adjust temperature/settings as needed
3. Scale the service based on your usage requirements
4. Implement client-side caching to reduce API calls