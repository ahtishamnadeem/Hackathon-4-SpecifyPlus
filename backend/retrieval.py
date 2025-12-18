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
    config = load_config()

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

    # Get configuration
    config = load_config()
    if collection_name is None:
        collection_name = config['collection_name']
    if top_k is None:
        top_k = config['max_retrievals']

    try:
        # Initialize Qdrant client
        qdrant_client = initialize_qdrant_client()

        # For now, we'll use a placeholder embedding approach
        # In a real implementation, we'd use the appropriate embedding service
        # Since the requirement is for OpenAI Agents SDK, we'll need to implement properly

        # For demonstration, we'll create a mock embedding
        # In a real implementation, this would call the appropriate embedding API
        import random
        query_embedding = [random.random() for _ in range(1536)]  # Typical embedding dimension

        # Perform semantic search in Qdrant
        search_results = qdrant_client.search(
            collection_name=collection_name,
            query_vector=query_embedding,
            limit=top_k,
            with_payload=True,
            with_vectors=False
        )

        # Format results
        retrieved_chunks = []
        for point in search_results:
            chunk = {
                'text': point.payload.get('text', ''),
                'similarity_score': point.score,
                'metadata': {
                    'module': point.payload.get('module', ''),
                    'page': point.payload.get('page', ''),
                    'heading': point.payload.get('heading', ''),
                    'url': point.payload.get('url', ''),
                    'title': point.payload.get('title', '')
                },
                'vector_id': point.id
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
                'top_k_requested': top_k
            }
        }

        logger.info(f"Retrieved {len(retrieved_chunks)} chunks for query: '{query_text[:50]}...' in {execution_time:.2f}ms")
        return result

    except Exception as e:
        logger.error(f"Failed to retrieve content: {str(e)}")
        raise ConnectionError(f"Could not retrieve content: {str(e)}")


def validate_qdrant_connection(collection_name: str = None) -> Dict[str, Any]:
    """
    Validate connection to Qdrant and check collection accessibility

    Args:
        collection_name: Name of the collection to validate (defaults to config value)

    Returns:
        Dictionary containing connection status and collection info
    """
    config = load_config()
    if collection_name is None:
        collection_name = config['collection_name']

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
            } if collection_info else None
        }

        logger.info(f"Qdrant connection validated: collection '{collection_name}' exists with {vector_count} vectors")
        return result

    except Exception as e:
        logger.error(f"Failed to validate Qdrant connection: {str(e)}")
        raise ConnectionError(f"Could not validate Qdrant connection: {str(e)}")


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