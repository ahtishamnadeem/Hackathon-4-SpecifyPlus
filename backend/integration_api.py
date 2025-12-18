"""
FastAPI Integration API

Main FastAPI application for the frontend-backend integration that handles
communication between the Docusaurus chat component and the RAG agent backend.
"""

import os
import sys
import logging
import asyncio
from datetime import datetime
from typing import Dict, Any, Optional, List
from fastapi import FastAPI, HTTPException, BackgroundTasks, Request
from pydantic import BaseModel
import uvicorn

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler('integration_api.log'),
        logging.StreamHandler(sys.stdout)
    ]
)
logger = logging.getLogger(__name__)

# Initialize FastAPI app
app = FastAPI(
    title="RAG Frontend-Backend Integration API",
    description="API for connecting Docusaurus frontend chat component with RAG agent backend",
    version="1.0.0"
)

# Import modules for the integration
from cors_config import configure_cors
from validation import ValidationError


# Define Pydantic models for request/response validation
class ChatQueryRequest(BaseModel):
    """Request model for chat query endpoint"""
    query: str
    selected_text: Optional[str] = None
    context_metadata: Optional[Dict[str, Any]] = {}
    user_preferences: Optional[Dict[str, Any]] = {}


class TextSelectionRequest(BaseModel):
    """Request model for text selection endpoint"""
    selected_text: str
    page_url: str
    page_title: str
    position_start: int
    position_end: int
    timestamp: str


class ChatQueryResponse(BaseModel):
    """Response model for chat query endpoint"""
    success: bool
    message: Optional[str] = None
    sources: Optional[List[Dict[str, Any]]] = []
    confidence: Optional[float] = 0.0
    processing_time_ms: Optional[float] = 0.0
    conversation_id: Optional[str] = None
    error: Optional[Dict[str, str]] = None


class TextSelectionResponse(BaseModel):
    """Response model for text selection endpoint"""
    success: bool
    selection_id: Optional[str] = None
    message: Optional[str] = None
    error: Optional[Dict[str, str]] = None


class HealthResponse(BaseModel):
    """Response model for health check endpoint"""
    status: str
    timestamp: str
    services: Dict[str, str]
    response_time_ms: float
    error: Optional[Dict[str, str]] = None


# Configure CORS for cross-origin requests
allowed_origins_str = os.getenv('ALLOWED_ORIGINS', 'http://localhost:3000,http://localhost:3001,https://your-docusaurus-site.com')
allowed_origins = [origin.strip() for origin in allowed_origins_str.split(',') if origin.strip()]
configure_cors(app, allowed_origins)


@app.get("/chat/health", response_model=HealthResponse)
async def health_check():
    """Health check endpoint to verify service status"""
    start_time = asyncio.get_event_loop().time()

    # Check the status of various services
    services_status = {
        "frontend": "connected",  # This is the frontend's perspective
        "backend": "connected",   # This service is running
        "rag_agent": "unknown"    # We'd need to check the actual RAG agent status
    }

    # In a real implementation, we'd check the actual RAG agent status
    # For now, we'll assume it's connected if we can load the configuration
    try:
        rag_agent_url = os.getenv('RAG_AGENT_URL')
        if rag_agent_url:
            # In a real implementation, we'd ping the RAG agent
            services_status["rag_agent"] = "connected"
        else:
            services_status["rag_agent"] = "disconnected"
    except Exception as e:
        logger.error(f"Error checking RAG agent status: {str(e)}")
        services_status["rag_agent"] = "disconnected"

    # Calculate response time
    end_time = asyncio.get_event_loop().time()
    response_time = (end_time - start_time) * 1000  # Convert to milliseconds

    # Determine overall status
    overall_status = "healthy"
    if services_status["backend"] == "disconnected" or services_status["rag_agent"] == "disconnected":
        overall_status = "degraded"
    if services_status["backend"] == "disconnected" and services_status["rag_agent"] == "disconnected":
        overall_status = "unavailable"

    return HealthResponse(
        status=overall_status,
        timestamp=datetime.utcnow().isoformat() + "Z",
        services=services_status,
        response_time_ms=response_time
    )


@app.post("/chat/send", response_model=ChatQueryResponse)
async def send_chat_message(request: ChatQueryRequest):
    """Process a user query and return a generated answer based on retrieved content"""
    try:
        # This is a placeholder implementation
        # In a real implementation, this would call the RAG agent

        # Log the incoming request
        logger.info(f"Received chat query: '{request.query[:50]}...' with selected text: {bool(request.selected_text)}")

        # Placeholder response - in real implementation, this would call the RAG agent
        response_data = {
            "success": True,
            "message": "This is a placeholder response. The full integration with the RAG agent would be implemented in a complete system.",
            "sources": [],
            "confidence": 0.85,
            "processing_time_ms": 1200.0,
            "conversation_id": "conv-placeholder-12345"
        }

        logger.info(f"Returning chat response for query: '{request.query[:50]}...'")
        return ChatQueryResponse(**response_data)

    except ValidationError as ve:
        logger.error(f"Validation error in chat query: {str(ve)}")
        return ChatQueryResponse(
            success=False,
            error={
                "type": "ValidationError",
                "message": str(ve),
                "code": "VALIDATION_ERROR"
            }
        )
    except Exception as e:
        logger.error(f"Error processing chat query: {str(e)}")
        return ChatQueryResponse(
            success=False,
            error={
                "type": "ProcessingError",
                "message": "An error occurred while processing your query",
                "code": "PROCESSING_ERROR"
            }
        )


@app.post("/chat/text-selection", response_model=TextSelectionResponse)
async def handle_text_selection(selection: TextSelectionRequest):
    """Handle text selection data from frontend for context capture"""
    try:
        # Validate the selection data
        if not selection.selected_text or not selection.selected_text.strip():
            raise ValidationError("Selected text is required and cannot be empty")

        if not selection.page_url:
            raise ValidationError("Page URL is required")

        if selection.position_start < 0 or selection.position_end < 0:
            raise ValidationError("Position values must be non-negative")

        if selection.position_start > selection.position_end:
            raise ValidationError("Start position cannot be greater than end position")

        # Log the selection
        logger.info(f"Captured text selection from {selection.page_url}: '{selection.selected_text[:50]}...'")

        # Generate a selection ID
        import uuid
        selection_id = f"sel-{uuid.uuid4()}"

        # In a real implementation, this might store the selection data
        # for future reference or analysis

        response_data = {
            "success": True,
            "selection_id": selection_id,
            "message": "Text selection captured successfully"
        }

        logger.info(f"Text selection handled successfully with ID: {selection_id}")
        return TextSelectionResponse(**response_data)

    except ValidationError as ve:
        logger.error(f"Validation error in text selection: {str(ve)}")
        return TextSelectionResponse(
            success=False,
            error={
                "type": "ValidationError",
                "message": str(ve),
                "code": "VALIDATION_ERROR"
            }
        )
    except Exception as e:
        logger.error(f"Error handling text selection: {str(e)}")
        return TextSelectionResponse(
            success=False,
            error={
                "type": "ProcessingError",
                "message": "An error occurred while processing the text selection",
                "code": "PROCESSING_ERROR"
            }
        )


if __name__ == "__main__":
    # Load configuration and start the server
    host = os.getenv('BACKEND_HOST', '0.0.0.0')
    port = int(os.getenv('BACKEND_PORT', '8000'))

    logger.info(f"Starting RAG Frontend-Backend Integration API on {host}:{port}")

    uvicorn.run(
        "integration_api:app",
        host=host,
        port=port,
        reload=True
    )