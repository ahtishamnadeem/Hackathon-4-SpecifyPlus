# API Contract: RAG Spec-4 Frontend-Backend Integration

## Overview
API contract for the endpoints that facilitate communication between the Docusaurus frontend chat component and the FastAPI backend for RAG agent integration.

## Endpoint: POST /chat/query
**Description**: Process user queries with optional selected text context and return generated answers based on retrieved content

### Request
**Content-Type**: `application/json`

**Body Parameters**:
- `query` (string, required): The user's query text
- `selected_text` (string, optional): Text selected from book pages for context
- `context_metadata` (object, optional): Additional context information:
  - `page_url` (string): URL of the page where text was selected
  - `page_title` (string): Title of the page where text was selected
  - `module` (string): Module or section where text was selected
- `user_preferences` (object, optional): User preferences for the response:
  - `temperature` (float): Temperature setting for response generation (0.0 to 1.0, default: 0.7)
  - `max_tokens` (integer): Maximum tokens for the response (default: 500)

**Example Request**:
```json
{
  "query": "What are the key concepts in ROS 2?",
  "selected_text": "Robot Operating System 2 is a flexible framework for writing robot software",
  "context_metadata": {
    "page_url": "https://book.example.com/ros2-intro",
    "page_title": "Introduction to ROS 2",
    "module": "Chapter 1"
  },
  "user_preferences": {
    "temperature": 0.7,
    "max_tokens": 300
  }
}
```

### Response
**Success Response (200 OK)**:
```json
{
  "success": true,
  "answer": "ROS 2 (Robot Operating System 2) is a flexible framework for writing robot software...",
  "sources": [
    {
      "text": "ROS 2 is designed to be a flexible framework for writing robot software...",
      "url": "https://book.example.com/ros2-intro",
      "page": "Introduction to ROS 2",
      "similarity_score": 0.85
    }
  ],
  "confidence": 0.85,
  "retrieval_stats": {
    "chunks_retrieved": 3,
    "execution_time_ms": 450
  },
  "query_id": "query-12345"
}
```

**Error Response (400 Bad Request)**:
```json
{
  "success": false,
  "error": {
    "type": "ValidationError",
    "message": "Query text is required and cannot be empty",
    "code": "MISSING_QUERY"
  },
  "timestamp": "2025-12-19T10:30:00Z"
}
```

**Error Response (503 Service Unavailable)**:
```json
{
  "success": false,
  "error": {
    "type": "ServiceUnavailable",
    "message": "Unable to connect to RAG agent services",
    "code": "AGENT_UNAVAILABLE"
  },
  "timestamp": "2025-12-19T10:30:00Z"
}
```

## Endpoint: POST /chat/text-selection
**Description**: Handle text selection data from frontend for context capture

### Request
**Content-Type**: `application/json`

**Body Parameters**:
- `selected_text` (string, required): The selected text content
- `page_url` (string, required): URL of the page where text was selected
- `page_title` (string, required): Title of the page where text was selected
- `position_start` (integer, required): Starting character position of selection
- `position_end` (integer, required): Ending character position of selection
- `timestamp` (string, required): ISO 8601 timestamp of selection

**Example Request**:
```json
{
  "selected_text": "This is the selected text from the book",
  "page_url": "https://book.example.com/chapter-2",
  "page_title": "Advanced Concepts",
  "position_start": 150,
  "position_end": 190,
  "timestamp": "2025-12-19T10:30:00Z"
}
```

### Response
**Success Response (200 OK)**:
```json
{
  "success": true,
  "selection_id": "sel-67890",
  "message": "Text selection captured successfully"
}
```

**Error Response (400 Bad Request)**:
```json
{
  "success": false,
  "error": {
    "type": "ValidationError",
    "message": "Selected text and page URL are required",
    "code": "MISSING_SELECTION_DATA"
  },
  "timestamp": "2025-12-19T10:30:00Z"
}
```

## Endpoint: GET /chat/health
**Description**: Health check endpoint to verify service status

### Request
**Query Parameters**: None

### Response
**Success Response (200 OK)**:
```json
{
  "status": "healthy",
  "timestamp": "2025-12-19T10:30:00Z",
  "services": {
    "frontend": "connected",
    "backend": "connected",
    "rag_agent": "connected"
  },
  "response_time_ms": 15.2
}
```

**Error Response (503 Service Unavailable)**:
```json
{
  "status": "unavailable",
  "timestamp": "2025-12-19T10:30:00Z",
  "services": {
    "frontend": "connected",
    "backend": "disconnected",
    "rag_agent": "disconnected"
  },
  "response_time_ms": 15.2
}
```

## Error Codes

- `MISSING_QUERY`: Query text is missing or invalid
- `MISSING_SELECTION_DATA`: Required text selection data is missing
- `AGENT_UNAVAILABLE`: RAG agent services are unavailable
- `VALIDATION_ERROR`: Request validation failed
- `SERVICE_UNAVAILABLE`: Dependent service is unavailable
- `RATE_LIMIT_EXCEEDED`: Request rate limit exceeded

## Rate Limiting
- **Per IP**: 10 requests per minute
- **Per API key**: 100 requests per minute
- **Burst limit**: 5 requests allowed in 1 second window

## Authentication
- **Method**: Optional API Key in header (if required)
- **Header**: `X-API-Key: your-api-key-here` (optional)
- **Required**: No by default, configurable per deployment