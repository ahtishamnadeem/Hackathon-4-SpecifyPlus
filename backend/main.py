"""
RAG Agent Construction

Backend service using FastAPI to implement a RAG (Retrieval-Augmented Generation) agent
that connects to Qdrant Cloud and processes user queries using the existing `rag_embedding`
collection. The system retrieves relevant content chunks and generates accurate answers
grounded in the book content, exposing API endpoints for external consumption.
"""

import os
import sys
import logging
from typing import Dict, Any, Optional, List
from fastapi import FastAPI, HTTPException, BackgroundTasks
from pydantic import BaseModel
import uvicorn
from slowapi import Limiter, _rate_limit_exceeded_handler
from slowapi.util import get_remote_address
from slowapi.errors import RateLimitExceeded


# Custom exception classes
class ConnectionError(Exception):
    """Raised when a connection error occurs"""
    pass


class ConfigurationError(Exception):
    """Raised when configuration parameters are invalid"""
    pass


class AgentError(Exception):
    """Raised when any step in the agent processing fails"""
    pass


class ValidationError(Exception):
    """Raised when validation parameters are invalid"""
    pass


class AuthenticationError(Exception):
    """Raised when authentication fails"""
    pass


class NotFoundError(Exception):
    """Raised when a requested resource is not found"""
    pass


class RateLimitError(Exception):
    """Raised when API rate limits are exceeded"""
    pass


class IntegrityError(Exception):
    """Raised when retrieved content doesn't match expected format"""
    pass

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler('rag_agent.log'),
        logging.StreamHandler(sys.stdout)
    ]
)
logger = logging.getLogger(__name__)

# Initialize rate limiter
limiter = Limiter(key_func=get_remote_address)
app = FastAPI(
    title="RAG Agent API",
    description="Retrieval-Augmented Generation agent for answering questions about book content",
    version="1.0.0"
)
app.state.limiter = limiter
app.add_exception_handler(RateLimitExceeded, _rate_limit_exceeded_handler)

# Import modules (will be implemented in other files)
from config import load_config, validate_config
from agent import process_query, initialize_openai_client
from retrieval import retrieve_content, validate_qdrant_connection


class QueryRequest(BaseModel):
    """Request model for query endpoint"""
    query: str
    max_tokens: Optional[int] = 500
    temperature: Optional[float] = 0.7
    include_sources: Optional[bool] = True


class QueryResponse(BaseModel):
    """Response model for query endpoint"""
    query: str
    answer: str
    sources: Optional[List[Dict[str, Any]]]
    confidence: float
    retrieval_stats: Dict[str, Any]
    query_id: str


from fastapi import Request

@app.get("/health")
@limiter.limit("30/minute")  # 30 requests per minute per IP (health checks are more frequent)
async def health_check(request: Request):
    """Health check endpoint to verify service status"""
    import time
    from datetime import datetime

    start_time = time.time()

    # Check Qdrant connection
    qdrant_status = "disconnected"
    try:
        qdrant_conn = validate_qdrant_connection()
        qdrant_status = "connected" if qdrant_conn['connected'] else "disconnected"
    except:
        qdrant_status = "disconnected"

    # Check OpenAI connection
    openai_status = "disconnected"
    try:
        initialize_openai_client()
        openai_status = "connected"
    except:
        openai_status = "disconnected"

    # Determine overall status
    overall_status = "healthy" if qdrant_status == "connected" and openai_status == "connected" else "degraded"

    response_time = (time.time() - start_time) * 1000  # Convert to milliseconds

    return {
        "status": overall_status,
        "timestamp": datetime.utcnow().isoformat() + "Z",
        "services": {
            "qdrant": qdrant_status,
            "openai": openai_status
        },
        "response_time_ms": response_time
    }


@app.get("/info")
@limiter.limit("20/minute")  # 20 requests per minute per IP
async def get_agent_info(request: Request):
    """Information about the agent and its capabilities"""
    return {
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



@app.post("/query", response_model=QueryResponse)
@limiter.limit("10/minute")  # 10 requests per minute per IP
async def query_endpoint(request: Request, query_request: QueryRequest):
    """Process a user query and return a generated answer based on retrieved content"""
    try:
        # Process the query using the agent
        result = process_query(
            query_text=query_request.query,
            max_tokens=query_request.max_tokens,
            temperature=query_request.temperature
        )

        # Generate a query ID
        import uuid
        query_id = str(uuid.uuid4())

        # Prepare the response
        response = {
            "query": request.query,
            "answer": result['answer'],
            "sources": result['sources'] if request.include_sources else [],
            "confidence": result['confidence_score'],
            "retrieval_stats": result['retrieval_details']['retrieval_stats'],
            "query_id": query_id
        }

        return response

    except Exception as e:
        logger.error(f"Error processing query: {str(e)}")
        raise HTTPException(status_code=500, detail=str(e))


import argparse


def start_server(host: str = None, port: int = None):
    """
    Start the RAG Agent server with specified host and port
    """
    config = load_config()

    if host is None:
        host = config['host']
    if port is None:
        port = config['port']

    logger.info(f"Starting RAG Agent server on {host}:{port}")

    import uvicorn
    uvicorn.run(app, host=host, port=port, reload=False)


def main():
    """Main function to start the server with command-line arguments"""
    try:
        parser = argparse.ArgumentParser(description='RAG Agent Server')
        parser.add_argument('--host', default=None, help='Host to bind to (default: from config)')
        parser.add_argument('--port', type=int, default=None, help='Port to bind to (default: from config)')
        parser.add_argument('--reload', action='store_true', help='Enable auto-reload (development)')

        args = parser.parse_args()

        config = load_config()

        # Use command-line args if provided, otherwise use config values
        host = args.host if args.host is not None else config['host']
        port = args.port if args.port is not None else config['port']
        reload = args.reload

        logger.info(f"Starting RAG Agent server on {host}:{port}")

        import uvicorn
        uvicorn.run(
            app,
            host=host,
            port=port,
            reload=reload
        )
    except KeyboardInterrupt:
        logger.info("Server shutdown requested by user")
        sys.exit(0)
    except Exception as e:
        logger.error(f"Failed to start server: {str(e)}")
        sys.exit(1)


if __name__ == "__main__":
    main()