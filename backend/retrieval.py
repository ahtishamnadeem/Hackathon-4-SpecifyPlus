"""
Content retrieval module for the RAG Agent

Handles connecting to Qdrant Cloud and retrieving relevant content chunks
based on semantic similarity to user queries.
"""

import os
import logging
import time
from typing import List, Dict, Any, Optional
from qdrant_client import QdrantClient
from qdrant_client.http.models import SearchRequest
from qdrant_client.models import VectorParams, Distance
import cohere

from config import load_config

logger = logging.getLogger(__name__)


def initialize_qdrant_client() -> QdrantClient:
    """
    Initialize Qdrant client with connection validation
    """
    try:
        config = load_config()
    except Exception as e:
        logger.error(f"Failed to load configuration for Qdrant: {str(e)}")
        raise ConnectionError(f"Could not load configuration: {str(e)}")

    # Check if Qdrant is enabled
    if not config.get('qdrant_enabled', False):
        logger.warning("Qdrant not configured - cannot initialize client")
        raise ConnectionError("Qdrant not configured - please set QDRANT_URL and QDRANT_API_KEY in environment variables")

    try:
        client = QdrantClient(
            url=config['qdrant_url'],
            api_key=config['qdrant_api_key'],
            timeout=30
        )

        # Test connection by getting collections
        client.get_collections()
        logger.info("Successfully connected to Qdrant Cloud")
        return client
    except Exception as e:
        logger.error(f"Failed to connect to Qdrant: {str(e)}")
        raise ConnectionError(f"Could not connect to Qdrant: {str(e)}")


def retrieve_content(query_text: str, collection_name: str = None, top_k: int = None) -> Dict[str, Any]:
    """
    Retrieve relevant content chunks from Qdrant based on semantic similarity

    Args:
        query_text: The user's query text
        collection_name: Name of the Qdrant collection (defaults to config value)
        top_k: Number of results to retrieve (defaults to config value)

    Returns:
        Dictionary containing query text, retrieved chunks, and execution stats
    """
    start_time = time.time()

    try:
        # Get configuration
        config = load_config()
        if collection_name is None:
            collection_name = config.get('collection_name', 'rag_embedding')
        if top_k is None:
            top_k = config.get('max_retrievals', 5)
    except Exception as e:
        logger.error(f"Failed to load configuration for content retrieval: {str(e)}")
        execution_time = (time.time() - start_time) * 1000
        raise ConnectionError(f"Could not load configuration: {str(e)}")

    # Check if Qdrant is enabled
    if not config.get('qdrant_enabled', False):
        logger.warning("Qdrant not enabled - returning empty results")
        execution_time = (time.time() - start_time) * 1000  # Convert to milliseconds

        result = {
            'query_text': query_text,
            'retrieved_chunks': [],
            'retrieval_stats': {
                'chunks_retrieved': 0,
                'execution_time_ms': execution_time,
                'collection_name': collection_name,
                'top_k_requested': top_k,
                'qdrant_enabled': False,
                'reason': 'Qdrant not configured'
            }
        }
        logger.info(f"Qdrant not configured, returning empty results for query: '{query_text[:50]}...' in {execution_time:.2f}ms")
        return result

    try:
        # Initialize Qdrant client
        qdrant_client = initialize_qdrant_client()

        # Initialize Cohere client for embeddings
        import cohere
        co = cohere.Client(config['cohere_api_key'])

        # Generate embedding for the query using Cohere
        response = co.embed(
            texts=[query_text],
            model='embed-english-v3.0',  # Using Cohere's latest embedding model
            input_type='search_query'
        )
        query_embedding = response.embeddings[0]

        # Perform semantic search in Qdrant using query_points method (newer API)
        from qdrant_client.http import models
        search_results = qdrant_client.query_points(
            collection_name=collection_name,
            query=query_embedding,
            limit=top_k,
            with_payload=True,
            with_vectors=False
        )

        # Format results - query_points returns PointStruct or similar objects
        retrieved_chunks = []
        for point in search_results.points:
            # Extract payload safely
            payload = point.payload if point.payload else {}
            chunk = {
                'text': payload.get('text', ''),
                'similarity_score': point.score if hasattr(point, 'score') else 0.0,
                'metadata': {
                    'module': payload.get('module', ''),
                    'page': payload.get('page', ''),
                    'heading': payload.get('heading', ''),
                    'url': payload.get('url', ''),
                    'title': payload.get('title', '')
                },
                'vector_id': point.id if hasattr(point, 'id') else None
            }
            retrieved_chunks.append(chunk)

        execution_time = (time.time() - start_time) * 1000  # Convert to milliseconds

        result = {
            'query_text': query_text,
            'retrieved_chunks': retrieved_chunks,
            'retrieval_stats': {
                'chunks_retrieved': len(retrieved_chunks),
                'execution_time_ms': execution_time,
                'collection_name': collection_name,
                'top_k_requested': top_k,
                'qdrant_enabled': True
            }
        }

        logger.info(f"Retrieved {len(retrieved_chunks)} chunks for query: '{query_text[:50]}...' in {execution_time:.2f}ms")
        return result

    except Exception as e:
        logger.error(f"Failed to retrieve content: {str(e)}")
        execution_time = (time.time() - start_time) * 1000  # Convert to milliseconds

        # Return empty results if Qdrant fails but don't raise an exception that breaks the flow
        result = {
            'query_text': query_text,
            'retrieved_chunks': [],
            'retrieval_stats': {
                'chunks_retrieved': 0,
                'execution_time_ms': execution_time,
                'collection_name': collection_name,
                'top_k_requested': top_k,
                'qdrant_enabled': True,
                'error': str(e)
            }
        }
        logger.warning(f"Qdrant retrieval failed, returning empty results for query: '{query_text[:50]}...' in {execution_time:.2f}ms. Error: {str(e)}")
        return result


def validate_qdrant_connection(collection_name: str = None) -> Dict[str, Any]:
    """
    Validate connection to Qdrant and check collection accessibility

    Args:
        collection_name: Name of the collection to validate (defaults to config value)

    Returns:
        Dictionary containing connection status and collection info
    """
    try:
        config = load_config()
        if collection_name is None:
            collection_name = config.get('collection_name', 'rag_embedding')
    except Exception as e:
        logger.error(f"Failed to load configuration for Qdrant validation: {str(e)}")
        return {
            'connected': False,
            'collection_exists': False,
            'vector_count': 0,
            'collection_config': None,
            'qdrant_enabled': False,
            'error': f"Configuration error: {str(e)}"
        }

    # Check if Qdrant is enabled
    if not config.get('qdrant_enabled', False):
        logger.info("Qdrant not configured - validation skipped")
        return {
            'connected': False,
            'collection_exists': False,
            'vector_count': 0,
            'collection_config': None,
            'qdrant_enabled': False,
            'error': "Qdrant not configured - please set QDRANT_URL and QDRANT_API_KEY in environment variables"
        }

    try:
        client = initialize_qdrant_client()

        # Check if collection exists
        collections = client.get_collections()
        collection_names = [coll.name for coll in collections.collections]

        collection_exists = collection_name in collection_names

        # Get collection info if it exists
        collection_info = None
        vector_count = 0
        if collection_exists:
            collection_info = client.get_collection(collection_name)
            vector_count = getattr(collection_info, 'points_count', 0)

        result = {
            'connected': True,
            'collection_exists': collection_exists,
            'vector_count': vector_count,
            'collection_config': {
                'vectors_count': vector_count,
                'indexed_vectors_count': getattr(collection_info, 'indexed_vectors_count', 0) if collection_info else 0,
                'config': getattr(collection_info, 'config', {}) if collection_info else {}
            } if collection_info else None,
            'qdrant_enabled': True
        }

        logger.info(f"Qdrant connection validated: collection '{collection_name}' exists with {vector_count} vectors")
        return result

    except Exception as e:
        logger.error(f"Failed to validate Qdrant connection: {str(e)}")
        return {
            'connected': False,
            'collection_exists': False,
            'vector_count': 0,
            'collection_config': None,
            'qdrant_enabled': True,
            'error': f"Connection error: {str(e)}"
        }


if __name__ == "__main__":
    # Test the retrieval functionality
    try:
        config = load_config()
        print("Testing Qdrant connection...")
        conn_result = validate_qdrant_connection()
        print(f"Connected: {conn_result['connected']}")
        print(f"Collection exists: {conn_result['collection_exists']}")
        print(f"Vector count: {conn_result['vector_count']}")

        print("\nTesting content retrieval...")
        result = retrieve_content("What is ROS 2?", top_k=2)
        print(f"Retrieved {len(result['retrieved_chunks'])} chunks")
        if result['retrieved_chunks']:
            print(f"First chunk text: {result['retrieved_chunks'][0]['text'][:100]}...")

    except Exception as e:
        print(f"Error during testing: {e}")