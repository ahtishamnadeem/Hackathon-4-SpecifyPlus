"""
CORS Configuration Module

Handles CORS (Cross-Origin Resource Sharing) configuration for the FastAPI backend
to allow secure communication between the frontend and backend.
"""

from fastapi.middleware.cors import CORSMiddleware
from typing import List


def configure_cors(app, allowed_origins: List[str] = None):
    """
    Configure CORS middleware for the FastAPI application

    Args:
        app: FastAPI application instance
        allowed_origins: List of allowed origins for CORS requests
    """
    if allowed_origins is None:
        # Default to common localhost addresses for development
        allowed_origins = [
            "http://localhost:3000",
            "http://localhost:3001",
            "http://127.0.0.1:3000",
            "http://127.0.0.1:3001",
            "https://localhost:3000",
            "https://127.0.0.1:3000"
        ]

    app.add_middleware(
        CORSMiddleware,
        allow_origins=allowed_origins,
        allow_credentials=True,
        allow_methods=["GET", "POST", "PUT", "DELETE", "OPTIONS"],
        allow_headers=["*"],
        # Expose headers that frontend might need to access
        expose_headers=["Access-Control-Allow-Origin", "Access-Control-Allow-Credentials"]
    )


# Example usage:
if __name__ == "__main__":
    from fastapi import FastAPI

    app = FastAPI(title="CORS Test App")

    # Configure CORS with default settings
    configure_cors(app)

    @app.get("/")
    async def root():
        return {"message": "CORS is configured!"}

    print("CORS configuration module created successfully!")
    print(f"Default allowed origins: {['http://localhost:3000', 'http://localhost:3001', 'http://127.0.0.1:3000', 'http://127.0.0.1:3001', 'https://localhost:3000', 'https://127.0.0.1:3000']}")