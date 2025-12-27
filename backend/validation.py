"""
RAG Retrieval Pipeline Validation

Backend service to validate the RAG retrieval pipeline by connecting to Qdrant Cloud
and executing semantic queries against the existing `rag_embedding` collection.
The system verifies that semantic queries return relevant content chunks with correct
text and metadata, confirming the pipeline is ready for agent integration.
"""

import os
import sys
import logging
import time
from datetime import datetime
from typing import List, Dict, Any, Optional
from qdrant_client import QdrantClient
from qdrant_client.http.models import SearchRequest
from pydantic import BaseModel
import cohere
import requests
from bs4 import BeautifulSoup
import argparse
from statistics import mean

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler('validation_pipeline.log'),
        logging.StreamHandler(sys.stdout)
    ]
)
logger = logging.getLogger(__name__)

# Configuration constants (using environment variables or defaults)
CHUNK_SIZE = int(os.getenv('CHUNK_SIZE', '800'))
CHUNK_OVERLAP = int(os.getenv('CHUNK_OVERLAP', '160'))
RATE_LIMIT = float(os.getenv('RATE_LIMIT', '1'))  # seconds between requests
COHERE_MODEL = os.getenv('COHERE_MODEL', 'embed-english-v3.0')
VALIDATION_TOP_K = int(os.getenv('VALIDATION_TOP_K', '5'))
SIMILARITY_THRESHOLD = float(os.getenv('SIMILARITY_THRESHOLD', '0.7'))


# Error classes
class QdrantConnectionError(Exception):
    """Raised when a connection error occurs with Qdrant"""
    pass


class ConfigurationError(Exception):
    """Raised when configuration parameters are invalid"""
    pass


class AuthenticationError(Exception):
    """Raised when authentication fails"""
    pass


class NotFoundError(Exception):
    """Raised when a requested resource is not found"""
    pass


class QueryError(Exception):
    """Raised when a query format is invalid"""
    pass


class QdrantIndexError(Exception):
    """Raised when the collection index is corrupted"""
    pass


class ValidationError(Exception):
    """Raised when validation parameters are invalid"""
    pass


class IntegrityError(Exception):
    """Raised when retrieved content doesn't match expected format"""
    pass


class ReadinessError(Exception):
    """Raised when readiness assessment cannot be completed"""
    pass


class PipelineValidationError(Exception):
    """Raised when any step in the validation pipeline fails"""
    pass


class InsufficientTestDataError(Exception):
    """Raised when there is insufficient data for validation"""
    pass


def load_environment_variables():
    """
    Load environment variables with validation
    """
    required_vars = ['COHERE_API_KEY', 'QDRANT_URL', 'QDRANT_API_KEY']
    missing_vars = [var for var in required_vars if not os.getenv(var)]

    if missing_vars:
        raise ConfigurationError(f"Missing required environment variables: {', '.join(missing_vars)}")

    logger.info("All required environment variables are present")
    return {
        'cohere_api_key': os.getenv('COHERE_API_KEY'),
        'qdrant_url': os.getenv('QDRANT_URL'),
        'qdrant_api_key': os.getenv('QDRANT_API_KEY'),
        'book_url': os.getenv('BOOK_URL', 'https://sigma-hackathon-4-specify-plus.vercel.app/')
    }


def initialize_qdrant_client():
    """
    Initialize Qdrant client with connection validation
    """
    qdrant_url = os.getenv('QDRANT_URL')
    qdrant_api_key = os.getenv('QDRANT_API_KEY')

    if not qdrant_url or not qdrant_api_key:
        raise ConfigurationError("QDRANT_URL and QDRANT_API_KEY must be set in environment variables")

    try:
        client = QdrantClient(
            url=qdrant_url,
            api_key=qdrant_api_key,
            timeout=30
        )

        # Test connection by getting collections
        client.get_collections()
        logger.info("Successfully connected to Qdrant Cloud")
        return client
    except Exception as e:
        logger.error(f"Failed to connect to Qdrant: {str(e)}")
        raise QdrantConnectionError(f"Could not connect to Qdrant: {str(e)}")


def initialize_cohere_client():
    """
    Initialize Cohere client with connection validation
    """
    cohere_api_key = os.getenv('COHERE_API_KEY')

    if not cohere_api_key:
        raise ConfigurationError("COHERE_API_KEY must be set in environment variables")

    try:
        client = cohere.Client(cohere_api_key)

        # Test connection by making a simple API call
        response = client.embed(texts=["test"], model="embed-english-v3.0")
        logger.info("Successfully connected to Cohere API")
        return client
    except Exception as e:
        logger.error(f"Failed to connect to Cohere: {str(e)}")
        raise QdrantConnectionError(f"Could not connect to Cohere: {str(e)}")


def validate_qdrant_connection(qdrant_url: str = None, qdrant_api_key: str = None, collection_name: str = None) -> Dict[str, Any]:
    """
    Validates connection to Qdrant Cloud and verifies access to the specified collection
    """
    if collection_name is None:
        from config import load_config
        config = load_config()
        collection_name = config.get('collection_name', 'rag_embedding')

    if not qdrant_url:
        qdrant_url = os.getenv('QDRANT_URL')
    if not qdrant_api_key:
        qdrant_api_key = os.getenv('QDRANT_API_KEY')

    if not qdrant_url or not qdrant_api_key:
        raise ConfigurationError("QDRANT_URL and QDRANT_API_KEY must be provided")

    try:
        client = QdrantClient(url=qdrant_url, api_key=qdrant_api_key, timeout=30)

        # Check if collection exists
        collections = client.get_collections()
        collection_names = [coll.name for coll in collections.collections]

        collection_exists = collection_name in collection_names

        # Get collection info if it exists
        collection_info = None
        vector_count = 0
        if collection_exists:
            collection_info = client.get_collection(collection_name)
            vector_count = collection_info.points_count
        # Note: If collection doesn't exist, we don't raise an error here anymore
        # The function will return the result with collection_exists=False

        result = {
            'connected': True,
            'collection_exists': collection_exists,
            'vector_count': vector_count,
            'collection_config': {
                'vectors_count': vector_count,
                'indexed_vectors_count': getattr(collection_info, 'indexed_vectors_count', 0) if collection_info else 0,
                'config': getattr(collection_info, 'config', {}).dict() if collection_info and hasattr(getattr(collection_info, 'config', {}), 'dict') else {}
            } if collection_info else None
        }

        logger.info(f"Qdrant connection validated: collection '{collection_name}' {'exists' if collection_exists else 'does not exist'} with {vector_count} vectors")
        return result

    except AuthenticationError:
        # Re-raise AuthenticationError if it was already the correct type
        raise
    except NotFoundError:
        # Re-raise NotFoundError if it was already the correct type
        raise
    except Exception as e:
        logger.error(f"Failed to validate Qdrant connection: {str(e)}")
        if "unauthorized" in str(e).lower() or "authentication" in str(e).lower() or "401" in str(e):
            raise AuthenticationError(f"Authentication failed for Qdrant: {str(e)}")
        elif "not found" in str(e).lower() or "404" in str(e):
            raise NotFoundError(f"Qdrant collection not found: {str(e)}")
        else:
            raise QdrantConnectionError(f"Could not validate Qdrant connection: {str(e)}")


def execute_semantic_query(query_text: str, collection_name: str = None, top_k: int = 5, query_filters: Dict = None) -> Dict[str, Any]:
    """
    Executes a semantic similarity search against the Qdrant collection
    """
    if collection_name is None:
        from config import load_config
        config = load_config()
        collection_name = config.get('collection_name', 'rag_embedding')

    # Initialize Cohere client to generate query embedding
    cohere_client = initialize_cohere_client()

    # Initialize Qdrant client
    qdrant_client = initialize_qdrant_client()

    try:
        # Generate embedding for the query
        response = cohere_client.embed(
            texts=[query_text],
            model=COHERE_MODEL,
            input_type="search_query"  # Appropriate for search queries
        )

        query_embedding = response.embeddings[0]  # Get the first (and only) embedding

        # Execute similarity search in Qdrant using query_points method (newer API)
        start_time = time.time()

        from qdrant_client.http import models
        search_result = qdrant_client.query_points(
            collection_name=collection_name,
            query=query_embedding,
            limit=top_k,
            with_payload=True,
            with_vectors=False
        )

        execution_time = (time.time() - start_time) * 1000  # Convert to milliseconds

        # Format results - query_points returns PointStruct or similar objects
        retrieved_chunks = []
        for point in search_result.points:
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

        result = {
            'query_text': query_text,
            'retrieved_chunks': retrieved_chunks,
            'execution_time_ms': execution_time
        }

        logger.info(f"Semantic query executed successfully: '{query_text[:50]}...' returned {len(retrieved_chunks)} results in {execution_time:.2f}ms")
        return result

    except Exception as e:
        logger.error(f"Failed to execute semantic query: {str(e)}")
        raise QdrantConnectionError(f"Could not execute semantic query: {str(e)}")


def validate_retrieved_results(retrieved_chunks: List[Dict[str, Any]], query_text: str = "", expected_content_types: List[str] = None) -> Dict[str, Any]:
    """
    Validates that retrieved results contain correct text and properly preserved metadata
    """
    if not retrieved_chunks:
        return {
            'content_accuracy': 0.0,
            'metadata_completeness': 0.0,
            'relevance_score': 0.0,
            'issues_found': [{'type': 'empty_results', 'severity': 'critical', 'details': 'No chunks were retrieved'}]
        }

    issues = []
    total_chunks = len(retrieved_chunks)

    # Validate content accuracy
    valid_content_chunks = 0
    for chunk in retrieved_chunks:
        if chunk.get('text') and len(chunk['text'].strip()) > 0:
            valid_content_chunks += 1
        else:
            issues.append({
                'type': 'missing_content',
                'severity': 'critical',
                'details': f'Chunk with ID {chunk.get("vector_id")} has empty or missing text content'
            })

    content_accuracy = valid_content_chunks / total_chunks if total_chunks > 0 else 0.0

    # Validate metadata completeness
    valid_metadata_chunks = 0
    for chunk in retrieved_chunks:
        metadata = chunk.get('metadata', {})
        required_fields = ['module', 'page', 'heading', 'url', 'title']
        missing_fields = [field for field in required_fields if not metadata.get(field)]

        if not missing_fields:
            valid_metadata_chunks += 1
        else:
            issues.append({
                'type': 'missing_metadata',
                'severity': 'warning',
                'details': f'Chunk with ID {chunk.get("vector_id")} missing metadata fields: {missing_fields}'
            })

    metadata_completeness = valid_metadata_chunks / total_chunks if total_chunks > 0 else 0.0

    # Validate relevance (based on similarity scores)
    if retrieved_chunks:
        avg_similarity = sum([chunk['similarity_score'] for chunk in retrieved_chunks]) / len(retrieved_chunks)
        relevance_score = avg_similarity
    else:
        relevance_score = 0.0

    # Additional validation checks
    for i, chunk in enumerate(retrieved_chunks):
        # Check similarity score range
        score = chunk.get('similarity_score', 0)
        if score < 0.0 or score > 1.0:
            issues.append({
                'type': 'invalid_similarity_score',
                'severity': 'error',
                'details': f'Chunk {i} has invalid similarity score: {score}'
            })

        # Check text content quality
        text = chunk.get('text', '')
        if len(text) < 10:  # Too short to be meaningful
            issues.append({
                'type': 'short_content',
                'severity': 'warning',
                'details': f'Chunk {i} has very short content ({len(text)} chars)'
            })

    validation_report = {
        'content_accuracy': content_accuracy,
        'metadata_completeness': metadata_completeness,
        'relevance_score': relevance_score,
        'issues_found': issues
    }

    logger.info(f"Validation completed for {total_chunks} chunks: content_accuracy={content_accuracy:.2f}, metadata_completeness={metadata_completeness:.2f}, relevance_score={relevance_score:.2f}")
    return validation_report


def confirm_pipeline_readiness(validation_results: List[Dict[str, Any]], minimum_accuracy_threshold: float = 0.9, minimum_metadata_completeness: float = 0.95) -> Dict[str, Any]:
    """
    Confirms the entire retrieval pipeline is ready for agent integration
    """
    if not validation_results:
        raise InsufficientTestDataError("No validation results provided for readiness assessment")

    # Calculate overall metrics
    content_accuracies = [result.get('content_accuracy', 0) for result in validation_results]
    metadata_completeness_values = [result.get('metadata_completeness', 0) for result in validation_results]
    relevance_scores = [result.get('relevance_score', 0) for result in validation_results]

    overall_accuracy = mean(content_accuracies) if content_accuracies else 0.0
    metadata_preservation_rate = mean(metadata_completeness_values) if metadata_completeness_values else 0.0
    avg_relevance = mean(relevance_scores) if relevance_scores else 0.0

    # Check readiness criteria
    accuracy_meets_threshold = overall_accuracy >= minimum_accuracy_threshold
    metadata_meets_threshold = metadata_preservation_rate >= minimum_metadata_completeness
    relevance_meets_minimum = avg_relevance >= 0.5  # Reasonable minimum for relevance

    is_ready = accuracy_meets_threshold and metadata_meets_threshold and relevance_meets_minimum

    # Determine confidence level
    if is_ready and overall_accuracy >= 0.95 and metadata_preservation_rate >= 0.98 and avg_relevance >= 0.7:
        confidence_level = "high"
    elif is_ready and overall_accuracy >= 0.9 and metadata_preservation_rate >= 0.95 and avg_relevance >= 0.6:
        confidence_level = "medium"
    else:
        confidence_level = "low"

    # Generate recommendations if not ready
    recommendations = []
    if not accuracy_meets_threshold:
        recommendations.append(f"Increase content accuracy from {overall_accuracy:.2f} to at least {minimum_accuracy_threshold}")
    if not metadata_meets_threshold:
        recommendations.append(f"Increase metadata completeness from {metadata_preservation_rate:.2f} to at least {minimum_metadata_completeness}")
    if not relevance_meets_minimum:
        recommendations.append(f"Improve query relevance from {avg_relevance:.2f} to at least 0.5")

    if not recommendations and is_ready:
        recommendations.append("Pipeline is performing well and ready for agent integration")

    readiness_report = {
        'is_ready_for_agent_integration': is_ready,
        'overall_accuracy': overall_accuracy,
        'metadata_preservation_rate': metadata_preservation_rate,
        'recommendations': recommendations,
        'confidence_level': confidence_level
    }

    logger.info(f"Pipeline readiness assessment: ready={is_ready}, confidence={confidence_level}, accuracy={overall_accuracy:.2f}, metadata_rate={metadata_preservation_rate:.2f}")
    return readiness_report


def main(
    qdrant_url: str = None,
    qdrant_api_key: str = None,
    cohere_api_key: str = None,
    collection_name: str = None,
    test_queries: List[str] = None,
    top_k: int = VALIDATION_TOP_K
):
    """
    Main validation function that orchestrates the complete pipeline validation
    """
    # Use config value if collection_name is not provided
    if collection_name is None:
        from config import load_config
        config = load_config()
        collection_name = config.get('collection_name', 'rag_embedding')

    logger.info("Starting RAG Retrieval Pipeline Validation")

    try:
        # Load environment variables if not provided
        if not qdrant_url or not qdrant_api_key or not cohere_api_key:
            env_vars = load_environment_variables()
            if not qdrant_url:
                qdrant_url = env_vars['qdrant_url']
            if not qdrant_api_key:
                qdrant_api_key = env_vars['qdrant_api_key']
            if not cohere_api_key:
                cohere_api_key = env_vars['cohere_api_key']

        # Default test queries if none provided
        if not test_queries:
            test_queries = [
                "What is ROS 2 and its role in robotics?",
                "How to create a ROS 2 node in Python?",
                "Explain message passing in ROS 2",
                "What are the differences between ROS 1 and ROS 2?",
                "How to use launch files in ROS 2?"
            ]

        # Step 1: Validate Qdrant connection
        logger.info("Step 1: Validating Qdrant connection...")
        connection_result = validate_qdrant_connection(qdrant_url, qdrant_api_key, collection_name)

        if not connection_result['connected'] or not connection_result['collection_exists']:
            raise PipelineValidationError(f"Qdrant connection or collection validation failed")

        # Step 2: Execute semantic queries and collect results
        logger.info(f"Step 2: Executing {len(test_queries)} semantic queries...")
        query_results = []
        validation_findings = []

        for i, query in enumerate(test_queries):
            logger.info(f"Executing query {i+1}/{len(test_queries)}: {query}")

            try:
                query_result = execute_semantic_query(query, collection_name, top_k)
                query_results.append(query_result)

                # Validate the retrieved results
                validation_result = validate_retrieved_results(query_result['retrieved_chunks'], query)
                validation_findings.append(validation_result)

            except Exception as e:
                logger.error(f"Failed to process query '{query}': {str(e)}")
                # Continue with other queries
                continue

        # Step 3: Assess pipeline readiness
        logger.info("Step 3: Assessing pipeline readiness...")
        readiness_assessment = confirm_pipeline_readiness(validation_findings)

        # Step 4: Generate final report
        import time
        start_time = time.time()

        execution_summary = {
            'total_queries_executed': len(query_results),
            'successful_validations': len(validation_findings),
            'total_chunks_validated': sum(len(result['retrieved_chunks']) for result in query_results if 'retrieved_chunks' in result),
            'start_time': datetime.utcnow().isoformat() + "Z",
            'end_time': datetime.utcnow().isoformat() + "Z",
            'duration_seconds': time.time() - start_time
        }

        final_report = {
            'connection_status': connection_result,
            'query_results': query_results,
            'validation_findings': validation_findings,
            'readiness_assessment': readiness_assessment,
            'execution_summary': execution_summary
        }

        # Print summary results
        print("\n" + "="*60)
        print("RAG RETRIEVAL PIPELINE VALIDATION RESULTS")
        print("="*60)
        print(f"✅ Qdrant connection: {'SUCCESS' if connection_result['connected'] and connection_result['collection_exists'] else 'FAILED'}")
        print(f"📊 Collection vectors: {connection_result['vector_count']}")
        print(f"🔍 Queries executed: {len(query_results)}/{len(test_queries)}")
        print(f"📈 Content accuracy: {readiness_assessment['overall_accuracy']:.2f}")
        print(f"🏷️  Metadata completeness: {readiness_assessment['metadata_preservation_rate']:.2f}")
        print(f"🎯 Ready for agent integration: {'YES' if readiness_assessment['is_ready_for_agent_integration'] else 'NO'}")
        print(f"💪 Confidence level: {readiness_assessment['confidence_level'].upper()}")
        print("="*60)

        if readiness_assessment['recommendations']:
            print("\n💡 RECOMMENDATIONS:")
            for rec in readiness_assessment['recommendations']:
                print(f"  • {rec}")

        print("\n")

        return final_report

    except Exception as e:
        logger.error(f"Pipeline validation failed: {str(e)}")
        raise PipelineValidationError(f"RAG retrieval pipeline validation failed: {str(e)}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='RAG Retrieval Pipeline Validation')
    parser.add_argument('--qdrant-url', default=os.getenv('QDRANT_URL'),
                       help='Qdrant Cloud URL (defaults to QDRANT_URL env var)')
    parser.add_argument('--qdrant-api-key', default=os.getenv('QDRANT_API_KEY'),
                       help='Qdrant API key (defaults to QDRANT_API_KEY env var)')
    parser.add_argument('--cohere-api-key', default=os.getenv('COHERE_API_KEY'),
                       help='Cohere API key (defaults to COHERE_API_KEY env var)')
    parser.add_argument('--collection-name', default=None,
                       help='Collection name to validate (defaults to COLLECTION_NAME env var or rag_embedding)')
    parser.add_argument('--top-k', type=int, default=VALIDATION_TOP_K,
                       help=f'Number of results to retrieve (default: {VALIDATION_TOP_K})')
    parser.add_argument('--query', action='append', dest='queries',
                       help='Test query to execute (can be specified multiple times)')

    args = parser.parse_args()

    # Use default queries if none provided via command line
    test_queries = args.queries if args.queries else None

    try:
        # Use config value if collection_name is not provided via command line
        collection_name = args.collection_name
        if collection_name is None:
            from config import load_config
            config = load_config()
            collection_name = config.get('collection_name', 'rag_embedding')

        result = main(
            qdrant_url=args.qdrant_url,
            qdrant_api_key=args.qdrant_api_key,
            cohere_api_key=args.cohere_api_key,
            collection_name=collection_name,
            test_queries=test_queries,
            top_k=args.top_k  # Pass the top_k parameter
        )
        print("Validation completed successfully!")
    except Exception as e:
        logger.error(f"Validation failed: {str(e)}")
        print(f"❌ Validation failed: {str(e)}")
        sys.exit(1)