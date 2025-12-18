# Data Model: RAG Spec-4 Frontend-Backend Integration

## Overview
Data models for the frontend-backend integration system that connects the Docusaurus chat component with the FastAPI backend for RAG agent communication.

## Entity: Chat Message
**Description**: A message in the conversation between user and RAG agent
**Fields**:
- `id` (string): Unique identifier for the message
- `text` (string): The message content (user query or agent response)
- `sender` (string): Who sent the message ('user' or 'agent')
- `timestamp` (datetime): When the message was created
- `selected_text` (string, optional): Text selected by user from book pages (for user messages)
- `confidence` (float, optional): Confidence score in agent response (for agent messages)
- `sources` (array of objects, optional): Source references for agent response

**Validation rules**:
- `text` must not be empty
- `sender` must be either 'user' or 'agent'
- `timestamp` must be in ISO 8601 format
- `confidence` must be between 0.0 and 1.0 if provided

## Entity: Text Selection
**Description**: Text selected by the user from book pages for context
**Fields**:
- `id` (string): Unique identifier for the selection
- `text` (string): The selected text content
- `page_url` (string): URL of the page where text was selected
- `page_title` (string): Title of the page where text was selected
- `position_start` (integer): Starting character position of selection in the page
- `position_end` (integer): Ending character position of selection in the page
- `timestamp` (datetime): When the selection was made

**Validation rules**:
- `text` must not be empty
- `page_url` must be a valid URL
- `position_start` must be less than `position_end`
- `position_start` and `position_end` must be non-negative
- `timestamp` must be in ISO 8601 format

## Entity: API Request
**Description**: Request object for API communication between frontend and backend
**Fields**:
- `query` (string): The user's query text
- `selected_text` (string, optional): Text selected from book pages for context
- `context_metadata` (object, optional): Additional context information:
  - `page_url` (string): URL of the page where text was selected
  - `page_title` (string): Title of the page where text was selected
  - `module` (string): Module or section where text was selected
- `user_preferences` (object, optional): User preferences for the response:
  - `temperature` (float): Temperature setting for response generation (0.0 to 1.0)
  - `max_tokens` (integer): Maximum tokens for the response

**Validation rules**:
- `query` must not be empty
- `temperature` must be between 0.0 and 1.0 if provided
- `max_tokens` must be positive if provided
- `page_url` in context_metadata must be a valid URL if provided

## Entity: API Response
**Description**: Response object from backend to frontend API communication
**Fields**:
- `success` (boolean): Whether the request was processed successfully
- `answer` (string): The agent's response to the query
- `sources` (array of objects): Array of source references used in the response:
  - `text` (string): Snippet of the source text
  - `url` (string): URL to the source
  - `page` (string): Page/module reference
  - `similarity_score` (float): Similarity score for the source (0.0 to 1.0)
- `confidence` (float): Confidence score for the response (0.0 to 1.0)
- `retrieval_stats` (object): Statistics about the retrieval process:
  - `chunks_retrieved` (integer): Number of content chunks retrieved
  - `execution_time_ms` (float): Time taken for retrieval in milliseconds
- `query_id` (string): Unique identifier for the query

**Validation rules**:
- `success` must be boolean
- If `success` is true, `answer` must not be empty
- `confidence` must be between 0.0 and 1.0
- `retrieval_stats.execution_time_ms` must be positive if provided
- `retrieval_stats.chunks_retrieved` must be non-negative

## Entity: Chat History
**Description**: Complete conversation history between user and agent
**Fields**:
- `conversation_id` (string): Unique identifier for the conversation
- `messages` (array of objects): Array of message objects (Chat Message entities)
- `created_at` (datetime): When the conversation was started
- `updated_at` (datetime): When the conversation was last updated
- `metadata` (object): Additional conversation metadata:
  - `user_session_id` (string): Session identifier for the user
  - `source_pages` (array of strings): URLs of pages where text was selected during conversation

**Validation rules**:
- `conversation_id` must be unique
- `messages` array must contain valid message objects
- `updated_at` must be equal to or later than `created_at`
- `source_pages` must contain valid URLs

## Entity: Error Response
**Description**: Standardized error response format for API communication
**Fields**:
- `success` (boolean): Always false for error responses
- `error` (object): Error information:
  - `type` (string): Type of error (e.g., "ValidationError", "ConnectionError")
  - `message` (string): Human-readable error message
  - `code` (string): Machine-readable error code
- `timestamp` (datetime): When the error occurred

**Validation rules**:
- `success` must be false
- `error` object must be present and contain type, message, and code
- `timestamp` must be in ISO 8601 format