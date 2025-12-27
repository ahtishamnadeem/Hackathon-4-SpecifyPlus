"""
RAG Agent Construction - Final Version
Includes: Async RAG, CORS, Rate limiting, /chat/send endpoint
"""

import os
import sys
import logging
from typing import Dict, Any, Optional, List
from fastapi import FastAPI, HTTPException, Body, Request
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
import uvicorn
from slowapi import Limiter, _rate_limit_exceeded_handler
from slowapi.util import get_remote_address
from slowapi.errors import RateLimitExceeded
import uuid
import asyncio

# -------------------------
# Logging
# -------------------------
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[logging.StreamHandler(sys.stdout)]
)
logger = logging.getLogger(__name__)

# -------------------------
# Rate Limiter
# -------------------------
limiter = Limiter(key_func=get_remote_address)
app = FastAPI(title="RAG Agent API", version="1.0.0")
app.state.limiter = limiter
app.add_exception_handler(RateLimitExceeded, _rate_limit_exceeded_handler)

# -------------------------
# CORS Configuration
# -------------------------
origins = [
    "http://localhost:3000",  # Frontend dev
    "http://localhost:8000",
    "*",  # allow all origins (for testing)
]

app.add_middleware(
    CORSMiddleware,
    allow_origins=origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# -------------------------
# Import RAG modules
# -------------------------
from config import load_config
from agent import process_query, initialize_openai_client
from retrieval import validate_qdrant_connection

# -------------------------
# Pydantic models
# -------------------------
class QueryRequest(BaseModel):
    query: str
    selected_text: Optional[str] = None
    context_metadata: Optional[Dict[str, Any]] = None
    max_tokens: Optional[int] = 500
    temperature: Optional[float] = 0.7
    include_sources: Optional[bool] = True
    user_preferences: Optional[Dict[str, Any]] = None

class QueryResponse(BaseModel):
    query: str
    answer: str
    sources: Optional[List[Dict[str, Any]]]
    confidence: float
    retrieval_stats: Dict[str, Any]
    query_id: str

# -------------------------
# Health Endpoint
# -------------------------
@app.get("/health")
@limiter.limit("30/minute")
async def health_check(request: Request):
    import time
    from datetime import datetime
    start_time = time.time()
    qdrant_status = "disconnected"
    openai_status = "disconnected"

    try:
        qdrant_conn = validate_qdrant_connection()
        qdrant_status = "connected" if qdrant_conn['connected'] else "disconnected"
    except:
        pass

    try:
        initialize_openai_client()
        openai_status = "connected"
    except:
        pass

    overall_status = "healthy" if qdrant_status == "connected" and openai_status == "connected" else "degraded"
    response_time = (time.time() - start_time) * 1000

    return {
        "status": overall_status,
        "timestamp": datetime.utcnow().isoformat() + "Z",
        "services": {"qdrant": qdrant_status, "openai": openai_status},
        "response_time_ms": response_time
    }

# -------------------------
# Agent Info Endpoint
# -------------------------
@app.get("/info")
@limiter.limit("20/minute")
async def get_agent_info(request: Request):
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

# -------------------------
# Chat / RAG Endpoint
# -------------------------
@app.post("/chat/send")
@limiter.limit("10/minute")
async def chat_send(request: Request, query_request: QueryRequest = Body(...)):
    """
    Async endpoint for frontend chat queries.
    Runs RAG agent and returns structured response.
    """
    try:
        loop = asyncio.get_running_loop()
        result = await loop.run_in_executor(
            None,
            lambda: process_query(
                query_text=query_request.query,
                selected_text=query_request.selected_text,
                max_tokens=query_request.max_tokens,
                temperature=query_request.temperature
            )
        )

        return {
            "reply": result.get("answer", ""),
            "sources": result.get("sources", []),
            "confidence": result.get("confidence_score", 0),
            "retrieval_stats": result.get("retrieval_details", {}).get("retrieval_stats", {}),
            "query_id": str(uuid.uuid4())
        }

    except Exception as e:
        error_msg = str(e)
        logger.error(f"Error in /chat/send: {error_msg}")

        # Check if this is a configuration error that can be more specifically described
        if "QDRANT_URL" in error_msg or "configuration" in error_msg.lower():
            raise HTTPException(
                status_code=500,
                detail="Configuration error: Required environment variables are missing. Please check that QDRANT_URL and other required configuration variables are set."
            )
        else:
            raise HTTPException(status_code=500, detail=error_msg)

# -------------------------
# Server Start
# -------------------------
def main():
    config = load_config()
    host = config.get("host", "0.0.0.0")  # Use 0.0.0.0 for Railway
    port = int(os.environ.get("PORT", config.get("port", 8000)))  # Use PORT from environment first
    uvicorn.run(app, host=host, port=port, reload=False)  # Disable reload for production

if __name__ == "__main__":
    main()
