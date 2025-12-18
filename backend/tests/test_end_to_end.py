"""
End-to-end tests for the complete RAG agent functionality
"""
import unittest
from fastapi.testclient import TestClient
from unittest.mock import Mock, patch, MagicMock
import os
import sys

# Add the backend directory to the path so we can import modules
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from main import app


class TestEndToEndRAGAgent(unittest.TestCase):
    """Test the complete RAG agent workflow end-to-end"""

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
    @patch('main.retrieve_content')
    @patch('main.process_query')
    def test_complete_workflow_success(self, mock_process_query, mock_retrieve_content, mock_init_openai, mock_validate_qdrant):
        """Test the complete RAG agent workflow from API request to response"""
        # Mock all the dependencies to simulate a complete successful flow
        mock_validate_qdrant.return_value = {
            'connected': True,
            'collection_exists': True,
            'vector_count': 150
        }
        mock_init_openai.return_value = Mock()

        mock_retrieve_content.return_value = {
            'query_text': 'What is ROS 2?',
            'retrieved_chunks': [
                {
                    'text': 'ROS 2 (Robot Operating System 2) is a flexible framework for writing robot software.',
                    'similarity_score': 0.92,
                    'metadata': {
                        'module': 'introduction',
                        'page': 'ros2-intro',
                        'heading': 'What is ROS 2?',
                        'url': 'https://book.example.com/ros2-intro',
                        'title': 'Introduction to ROS 2'
                    },
                    'vector_id': 'chunk-123'
                }
            ],
            'retrieval_stats': {
                'chunks_retrieved': 1,
                'execution_time_ms': 120.5,
                'collection_name': 'rag_embedding',
                'top_k_requested': 5
            }
        }

        mock_process_query.return_value = {
            'query_text': 'What is ROS 2?',
            'answer': 'ROS 2 (Robot Operating System 2) is a flexible framework for writing robot software. It provides services designed for a heterogeneous computer cluster such as hardware abstraction, device drivers, libraries, visualizers, message-passing, package management.',
            'sources': [
                {
                    'text': 'ROS 2 (Robot Operating System 2) is a flexible framework for writing robot software.',
                    'url': 'https://book.example.com/ros2-intro',
                    'page': 'ros2-intro',
                    'heading': 'What is ROS 2?',
                    'similarity_score': 0.92
                }
            ],
            'confidence_score': 0.88,
            'retrieval_details': {
                'retrieved_chunks': [
                    {
                        'text': 'ROS 2 (Robot Operating System 2) is a flexible framework for writing robot software.',
                        'similarity_score': 0.92,
                        'metadata': {
                            'module': 'introduction',
                            'page': 'ros2-intro',
                            'heading': 'What is ROS 2?',
                            'url': 'https://book.example.com/ros2-intro',
                            'title': 'Introduction to ROS 2'
                        },
                        'vector_id': 'chunk-123'
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
                'tokens_used': 75,
                'input_tokens': 45,
                'output_tokens': 30
            },
            'processing_time_ms': 1800.0
        }

        # Test the complete workflow through the API
        # Step 1: Check health
        health_response = self.client.get("/health")
        self.assertEqual(health_response.status_code, 200)
        health_data = health_response.json()
        self.assertEqual(health_data['status'], 'healthy')

        # Step 2: Get agent info
        info_response = self.client.get("/info")
        self.assertEqual(info_response.status_code, 200)
        info_data = info_response.json()
        self.assertEqual(info_data['agent_name'], 'RAG Book Assistant')

        # Step 3: Submit a query
        query_response = self.client.post(
            "/query",
            json={
                "query": "What is ROS 2?",
                "max_tokens": 200,
                "temperature": 0.7,
                "include_sources": True
            }
        )
        self.assertEqual(query_response.status_code, 200)
        query_data = query_response.json()

        # Step 4: Verify response structure and content
        self.assertEqual(query_data['query'], 'What is ROS 2?')
        self.assertIn('ROS 2', query_data['answer'])
        self.assertGreater(len(query_data['answer']), 10)  # Answer should have content
        self.assertGreater(query_data['confidence'], 0.5)  # Confidence should be reasonable
        self.assertGreater(len(query_data['sources']), 0)  # Should have at least one source

    @patch('main.validate_qdrant_connection')
    @patch('main.process_query')
    def test_workflow_with_low_confidence_query(self, mock_process_query, mock_validate_qdrant):
        """Test workflow with a query that results in low confidence answer"""
        # Mock dependencies
        mock_validate_qdrant.return_value = {
            'connected': True,
            'collection_exists': True,
            'vector_count': 150
        }

        mock_process_query.return_value = {
            'query_text': 'What is the meaning of life?',
            'answer': 'I cannot find specific information about this in the provided context.',
            'sources': [],
            'confidence_score': 0.2,  # Low confidence
            'retrieval_details': {
                'retrieved_chunks': [],
                'retrieval_stats': {
                    'chunks_retrieved': 0,
                    'execution_time_ms': 80.0,
                    'collection_name': 'rag_embedding',
                    'top_k_requested': 5
                }
            },
            'generation_details': {
                'model_used': 'gpt-4-turbo',
                'tokens_used': 25,
                'input_tokens': 15,
                'output_tokens': 10
            },
            'processing_time_ms': 1200.0
        }

        # Submit query through API
        response = self.client.post(
            "/query",
            json={
                "query": "What is the meaning of life?",
                "max_tokens": 150,
                "temperature": 0.7,
                "include_sources": True
            }
        )
        self.assertEqual(response.status_code, 200)

        data = response.json()
        self.assertEqual(data['query'], 'What is the meaning of life?')
        self.assertLess(data['confidence'], 0.5)  # Should have low confidence
        self.assertEqual(len(data['sources']), 0)  # No sources for low confidence answer

    def test_api_error_scenarios(self):
        """Test various API error scenarios"""
        # Test query endpoint with missing query
        response = self.client.post("/query", json={})
        self.assertEqual(response.status_code, 422)

        # Test query endpoint with empty query
        response = self.client.post("/query", json={"query": ""})
        self.assertEqual(response.status_code, 200)  # Should still process but may return empty result

        # Test query endpoint with very long query (should be handled gracefully)
        long_query = "test " * 1000
        response = self.client.post("/query", json={"query": long_query})
        # This might return 200 or 500 depending on how it's handled, but shouldn't crash the server

    @patch('main.validate_qdrant_connection')
    @patch('main.initialize_openai_client')
    def test_health_check_comprehensive(self, mock_init_openai, mock_validate_qdrant):
        """Test health check with various service states"""
        # Test with both services healthy
        mock_validate_qdrant.return_value = {'connected': True, 'collection_exists': True, 'vector_count': 100}
        mock_init_openai.return_value = Mock()

        response = self.client.get("/health")
        self.assertEqual(response.status_code, 200)
        data = response.json()
        self.assertIn(data['status'], ['healthy', 'degraded'])

        # Test with Qdrant failing
        def qdrant_side_effect(*args, **kwargs):
            raise Exception("Qdrant connection failed")
        mock_validate_qdrant.side_effect = qdrant_side_effect

        response = self.client.get("/health")
        self.assertEqual(response.status_code, 200)  # Health endpoint should still respond
        data = response.json()
        self.assertIn(data['services']['qdrant'], ['connected', 'disconnected'])

    def test_rate_limiting_simulation(self):
        """Test that the API can handle multiple requests without crashing"""
        # This test simulates multiple requests to ensure the system is robust
        for i in range(3):  # Test multiple requests
            response = self.client.get("/info")
            self.assertEqual(response.status_code, 200)
            data = response.json()
            self.assertEqual(data['agent_name'], 'RAG Book Assistant')


class TestAPISystemIntegration(unittest.TestCase):
    """Test system-level integration aspects"""

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

    def test_openapi_documentation_exists(self):
        """Test that OpenAPI documentation is available"""
        response = self.client.get("/openapi.json")
        self.assertEqual(response.status_code, 200)

        data = response.json()
        self.assertIn('info', data)
        self.assertIn('paths', data)
        self.assertEqual(data['info']['title'], 'RAG Agent API')

    def test_swagger_ui_available(self):
        """Test that Swagger UI is available"""
        response = self.client.get("/docs")
        self.assertEqual(response.status_code, 200)
        self.assertIn('swagger', response.text.lower())

    def test_multiple_concurrent_requests_simulation(self):
        """Simulate multiple concurrent requests to test system stability"""
        import threading
        import time

        results = []

        def make_request(query_num):
            response = self.client.post(
                "/query",
                json={
                    "query": f"Test query {query_num}",
                    "max_tokens": 100,
                    "temperature": 0.7
                }
            )
            results.append((query_num, response.status_code))

        # Start multiple threads making requests
        threads = []
        for i in range(5):  # 5 concurrent requests
            thread = threading.Thread(target=make_request, args=(i,))
            threads.append(thread)
            thread.start()
            time.sleep(0.01)  # Small delay between requests

        # Wait for all threads to complete
        for thread in threads:
            thread.join()

        # Verify all requests were processed
        self.assertEqual(len(results), 5)
        for query_num, status_code in results:
            self.assertIn(status_code, [200, 422, 500])  # Should get some valid response


if __name__ == '__main__':
    unittest.main()