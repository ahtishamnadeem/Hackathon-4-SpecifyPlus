# API Contract: RAG Agent Construction

## Overview
API contract for the RAG agent endpoints that process user queries and generate answers based on content retrieved from Qdrant.

## Endpoint: POST /query
**Description**: Process a user query and return a generated answer based on retrieved content

### Request
**Content-Type**: `application/json`

**Body Parameters**:
- `query` (string, required): The user's question or request text
- `max_tokens` (integer, optional): Maximum tokens for the response (default: 500, min: 1, max: 2000)
- `temperature` (float, optional): Temperature for response generation (default: 0.7, min: 0.0, max: 1.0)
- `include_sources` (boolean, optional): Whether to include source references (default: true)
- `session_id` (string, optional): Session identifier for conversation history

**Example Request**:
```json
{
  "query": "What are the key concepts in ROS 2?",
  "max_tokens": 300,
  "temperature": 0.5,
  "include_sources": true
}
```

### Response
**Success Response (200 OK)**:
```json
{
  "query": "What are the key concepts in ROS 2?",
  "answer": "ROS 2 (Robot Operating System 2) is a flexible framework for writing robot software. Key concepts include...",
  "sources": [
    {
      "text": "ROS 2 is designed to be a flexible framework for writing robot software...",
      "url": "https://example.com/ros2-intro",
      "page": "Introduction to ROS 2"
    }
  ],
  "confidence": 0.85,
  "retrieval_stats": {
    "chunks_retrieved": 3,
    "processing_time_ms": 450
  },
  "query_id": "query-12345"
}
```

**Error Response (400 Bad Request)**:
```json
{
  "error": "Invalid request parameters",
  "message": "Query text is required and cannot be empty",
  "code": "INVALID_QUERY"
}
```

**Error Response (503 Service Unavailable)**:
```json
{
  "error": "Service unavailable",
  "message": "Unable to connect to Qdrant or OpenAI services",
  "code": "SERVICE_UNAVAILABLE"
}
```

## Endpoint: GET /health
**Description**: Health check endpoint to verify service status

### Request
**Query Parameters**: None

### Response
**Success Response (200 OK)**:
```json
{
  "status": "healthy",
  "timestamp": "2025-12-18T10:30:00Z",
  "services": {
    "qdrant": "connected",
    "openai": "connected"
  },
  "response_time_ms": 15.2
}
```

**Error Response (503 Service Unavailable)**:
```json
{
  "status": "unavailable",
  "timestamp": "2025-12-18T10:30:00Z",
  "services": {
    "qdrant": "disconnected",
    "openai": "connected"
  },
  "response_time_ms": 15.2
}
```

## Endpoint: GET /info
**Description**: Information about the agent and its capabilities

### Request
**Query Parameters**: None

### Response
**Success Response (200 OK)**:
```json
{
  "agent_name": "RAG Book Assistant",
  "version": "1.0.0",
  "description": "RAG agent for answering questions about the book content",
  "capabilities": [
    "Semantic search in book content",
    "Accurate answers grounded in source material",
    "Source attribution for all answers"
  ],
  "supported_models": ["gpt-4-turbo", "gpt-3.5-turbo"],
  "max_query_length": 1000,
  "max_response_tokens": 2000,
  "collection_name": "rag_embedding"
}
```

## Error Codes

- `INVALID_QUERY`: Query text is missing or invalid
- `RETRIEVAL_FAILED`: Failed to retrieve relevant content from Qdrant
- `GENERATION_FAILED`: Failed to generate response from LLM
- `SERVICE_UNAVAILABLE`: Dependent service (Qdrant/OpenAI) is unavailable
- `VALIDATION_FAILED`: Response validation failed (not properly grounded)
- `RATE_LIMIT_EXCEEDED`: Request rate limit exceeded

## Rate Limiting
- **Per IP**: 100 requests per minute
- **Per API key**: 1000 requests per minute
- **Burst limit**: 10 requests allowed in 1 second window

## Authentication
- **Method**: API Key in header
- **Header**: `X-API-Key: your-api-key-here`
- **Required**: Yes for all endpoints