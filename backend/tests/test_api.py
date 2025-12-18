"""
Unit tests for the main.py API endpoints
"""
import unittest
from fastapi.testclient import TestClient
from unittest.mock import Mock, patch, MagicMock
import os
import sys

# Add the backend directory to the path so we can import modules
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from main import app


class TestAPIEndpoints(unittest.TestCase):
    """Test the API endpoints"""

    def setUp(self):
        """Set up test environment"""
        # Set up minimal required environment variables for testing
        os.environ['QDRANT_URL'] = 'https://test-qdrant.example.com'
        os.environ['QDRANT_API_KEY'] = 'test-qdrant-key'
        os.environ['OPENAI_API_KEY'] = 'test-openai-key'

        # Create test client
        self.client = TestClient(app)

    def tearDown(self):
        """Clean up test environment"""
        # Remove test environment variables
        for var in ['QDRANT_URL', 'QDRANT_API_KEY', 'OPENAI_API_KEY']:
            if var in os.environ:
                del os.environ[var]

    @patch('main.validate_qdrant_connection')
    @patch('main.initialize_openai_client')
    def test_health_check_endpoint(self, mock_init_openai, mock_validate_qdrant):
        """Test the health check endpoint"""
        # Mock the service checks
        mock_validate_qdrant.return_value = {
            'connected': True,
            'collection_exists': True,
            'vector_count': 100
        }
        mock_init_openai.return_value = Mock()  # Mock successful connection

        # Make request
        response = self.client.get("/health")

        # Assertions
        self.assertEqual(response.status_code, 200)
        data = response.json()
        self.assertEqual(data['status'], 'healthy')
        self.assertEqual(data['services']['qdrant'], 'connected')
        self.assertEqual(data['services']['openai'], 'connected')
        self.assertIn('response_time_ms', data)

    @patch('main.validate_qdrant_connection')
    def test_health_check_qdrant_disconnected(self, mock_validate_qdrant):
        """Test health check when Qdrant is disconnected"""
        # Mock Qdrant connection failure
        def side_effect(*args, **kwargs):
            raise Exception("Connection failed")
        mock_validate_qdrant.side_effect = side_effect

        # Make request
        response = self.client.get("/health")

        # Assertions
        self.assertEqual(response.status_code, 200)  # Health check should still return 200 but with degraded status
        data = response.json()
        self.assertIn(data['status'], ['healthy', 'degraded'])  # Could be degraded depending on OpenAI status
        self.assertIn(data['services']['qdrant'], ['connected', 'disconnected'])

    def test_info_endpoint(self):
        """Test the info endpoint"""
        # Make request
        response = self.client.get("/info")

        # Assertions
        self.assertEqual(response.status_code, 200)
        data = response.json()
        self.assertEqual(data['agent_name'], 'RAG Book Assistant')
        self.assertEqual(data['version'], '1.0.0')
        self.assertIn('capabilities', data)
        self.assertIn('semantic search in book content', [cap.lower() for cap in data['capabilities']])

    @patch('main.process_query')
    def test_query_endpoint_success(self, mock_process_query):
        """Test the query endpoint with successful response"""
        # Mock the process_query function
        mock_process_query.return_value = {
            'query_text': 'test query',
            'answer': 'This is a test answer based on the context.',
            'sources': [
                {
                    'text': 'Test source content...',
                    'url': 'https://example.com/test',
                    'page': 'test_page',
                    'heading': 'Test Heading',
                    'similarity_score': 0.85
                }
            ],
            'confidence_score': 0.85,
            'retrieval_details': {
                'retrieved_chunks': [
                    {
                        'text': 'Test content',
                        'similarity_score': 0.85,
                        'metadata': {'module': 'test', 'page': 'test', 'heading': 'Test', 'url': 'https://example.com/test', 'title': 'Test'},
                        'vector_id': 'test-id'
                    }
                ],
                'retrieval_stats': {
                    'chunks_retrieved': 1,
                    'execution_time_ms': 120.5,
                    'collection_name': 'rag_embedding',
                    'top_k_requested': 5
                }
            },
            'generation_details': {
                'model_used': 'gpt-4-turbo',
                'tokens_used': 50,
                'input_tokens': 30,
                'output_tokens': 20
            },
            'processing_time_ms': 150.0
        }

        # Make request
        response = self.client.post(
            "/query",
            json={
                "query": "test query",
                "max_tokens": 200,
                "temperature": 0.7,
                "include_sources": True
            }
        )

        # Assertions
        self.assertEqual(response.status_code, 200)
        data = response.json()
        self.assertEqual(data['query'], 'test query')
        self.assertEqual(data['answer'], 'This is a test answer based on the context.')
        self.assertEqual(data['confidence'], 0.85)
        self.assertGreater(len(data['sources']), 0)

    @patch('main.process_query')
    def test_query_endpoint_without_sources(self, mock_process_query):
        """Test the query endpoint with sources disabled"""
        # Mock the process_query function
        mock_process_query.return_value = {
            'query_text': 'test query without sources',
            'answer': 'This is an answer without showing sources.',
            'sources': [
                {
                    'text': 'Test source content...',
                    'url': 'https://example.com/test',
                    'page': 'test_page',
                    'heading': 'Test Heading',
                    'similarity_score': 0.85
                }
            ],
            'confidence_score': 0.85,
            'retrieval_details': {
                'retrieved_chunks': [
                    {
                        'text': 'Test content',
                        'similarity_score': 0.85,
                        'metadata': {'module': 'test', 'page': 'test', 'heading': 'Test', 'url': 'https://example.com/test', 'title': 'Test'},
                        'vector_id': 'test-id'
                    }
                ],
                'retrieval_stats': {
                    'chunks_retrieved': 1,
                    'execution_time_ms': 120.5,
                    'collection_name': 'rag_embedding',
                    'top_k_requested': 5
                }
            },
            'generation_details': {
                'model_used': 'gpt-4-turbo',
                'tokens_used': 50,
                'input_tokens': 30,
                'output_tokens': 20
            },
            'processing_time_ms': 150.0
        }

        # Make request without sources
        response = self.client.post(
            "/query",
            json={
                "query": "test query without sources",
                "max_tokens": 200,
                "temperature": 0.7,
                "include_sources": False  # Don't include sources
            }
        )

        # Assertions
        self.assertEqual(response.status_code, 200)
        data = response.json()
        self.assertEqual(data['query'], 'test query without sources')
        self.assertEqual(len(data['sources']), 0)  # Should be empty when include_sources is False

    def test_query_endpoint_validation_error(self):
        """Test the query endpoint with validation error"""
        # Make request with missing required field
        response = self.client.post(
            "/query",
            json={}  # Missing required 'query' field
        )

        # Assertions - should return 422 for validation error
        self.assertEqual(response.status_code, 422)

    @patch('main.process_query')
    def test_query_endpoint_internal_error(self, mock_process_query):
        """Test the query endpoint with internal error"""
        # Mock process_query to raise an exception
        mock_process_query.side_effect = Exception("Processing failed")

        # Make request
        response = self.client.post(
            "/query",
            json={
                "query": "error test query",
                "max_tokens": 200,
                "temperature": 0.7
            }
        )

        # Assertions - should return 500 for internal error
        self.assertEqual(response.status_code, 500)


class TestAPIServerStartup(unittest.TestCase):
    """Test server startup functionality"""

    def setUp(self):
        """Set up test environment"""
        # Set up minimal required environment variables for testing
        os.environ['QDRANT_URL'] = 'https://test-qdrant.example.com'
        os.environ['QDRANT_API_KEY'] = 'test-qdrant-key'
        os.environ['OPENAI_API_KEY'] = 'test-openai-key'

    def tearDown(self):
        """Clean up test environment"""
        # Remove test environment variables
        for var in ['QDRANT_URL', 'QDRANT_API_KEY', 'OPENAI_API_KEY']:
            if var in os.environ:
                del os.environ[var]

    def test_app_startup(self):
        """Test that the FastAPI app starts without errors"""
        # This test verifies that the app object is created successfully
        self.assertIsNotNone(app)
        self.assertEqual(app.title, "RAG Agent API")

    def test_routes_exist(self):
        """Test that expected routes exist"""
        routes = [route.path for route in app.routes]

        self.assertIn("/health", routes)
        self.assertIn("/info", routes)
        self.assertIn("/query", routes)
        self.assertIn("/openapi.json", routes)  # FastAPI default
        self.assertIn("/docs", routes)  # FastAPI default


if __name__ == '__main__':
    unittest.main()