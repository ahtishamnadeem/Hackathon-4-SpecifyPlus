"""
Integration tests for the validation pipeline
"""
import unittest
from unittest.mock import Mock, patch, MagicMock
import os
import sys
import tempfile
import json

# Add the backend directory to the path so we can import validation
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import validation


class TestValidationPipelineIntegration(unittest.TestCase):
    """Test the complete validation pipeline integration"""

    def setUp(self):
        """Set up environment variables for testing"""
        # Set up minimal required environment variables for validation
        os.environ['COHERE_API_KEY'] = 'test-cohere-key'
        os.environ['QDRANT_URL'] = 'https://test-qdrant.example.com'
        os.environ['QDRANT_API_KEY'] = 'test-qdrant-key'

    def tearDown(self):
        """Clean up environment variables"""
        for var in ['COHERE_API_KEY', 'QDRANT_URL', 'QDRANT_API_KEY']:
            if var in os.environ:
                del os.environ[var]

    @patch('validation.validate_qdrant_connection')
    @patch('validation.execute_semantic_query')
    @patch('validation.validate_retrieved_results')
    @patch('validation.confirm_pipeline_readiness')
    @patch('validation.initialize_qdrant_client')
    @patch('validation.initialize_cohere_client')
    def test_validation_pipeline_success(
        self,
        mock_init_cohere,
        mock_init_qdrant,
        mock_confirm_readiness,
        mock_validate_results,
        mock_execute_query,
        mock_validate_connection
    ):
        """Test successful execution of the complete validation pipeline"""
        # Mock the functions to return test data
        mock_validate_connection.return_value = {
            'connected': True,
            'collection_exists': True,
            'vector_count': 150,
            'collection_config': {'vectors_count': 150}
        }

        mock_execute_query.return_value = {
            'query_text': 'test query',
            'retrieved_chunks': [
                {
                    'text': 'test content',
                    'similarity_score': 0.8,
                    'metadata': {
                        'module': 'test',
                        'page': 'page1',
                        'heading': 'heading',
                        'url': 'https://example.com'
                    },
                    'vector_id': 'test-id'
                }
            ],
            'execution_time_ms': 100.0
        }

        mock_validate_results.return_value = {
            'content_accuracy': 0.95,
            'metadata_completeness': 0.98,
            'relevance_score': 0.8,
            'issues_found': []
        }

        mock_confirm_readiness.return_value = {
            'is_ready_for_agent_integration': True,
            'overall_accuracy': 0.95,
            'metadata_preservation_rate': 0.98,
            'recommendations': ['Pipeline is performing well and ready for agent integration'],
            'confidence_level': 'high'
        }

        # Run the main validation function
        result = validation.main(
            qdrant_url='https://test-qdrant.example.com',
            qdrant_api_key='test-key',
            cohere_api_key='test-cohere-key',
            collection_name='rag_embedding',
            test_queries=['test query']
        )

        # Verify the result structure
        self.assertIn('connection_status', result)
        self.assertIn('query_results', result)
        self.assertIn('validation_findings', result)
        self.assertIn('readiness_assessment', result)
        self.assertIn('execution_summary', result)

        # Verify that the mocked functions were called
        mock_validate_connection.assert_called_once()
        mock_execute_query.assert_called_once()
        mock_validate_results.assert_called_once()
        mock_confirm_readiness.assert_called_once()

    @patch('validation.validate_qdrant_connection')
    @patch('validation.logger')
    def test_validation_pipeline_with_error_handling(
        self,
        mock_logger,
        mock_validate_connection
    ):
        """Test validation pipeline error handling"""
        # Mock validate_qdrant_connection to raise an exception
        mock_validate_connection.side_effect = validation.QdrantConnectionError("Failed to connect to Qdrant")

        # Test that the error is propagated
        with self.assertRaises(validation.PipelineValidationError):
            validation.main(
                qdrant_url='https://test-qdrant.example.com',
                qdrant_api_key='test-key',
                cohere_api_key='test-cohere-key'
            )

        # Verify that error was logged
        mock_logger.error.assert_called()


class TestValidationFunctionsIntegration(unittest.TestCase):
    """Test integration between individual validation functions"""

    def setUp(self):
        """Set up environment variables for testing"""
        os.environ['COHERE_API_KEY'] = 'test-cohere-key'
        os.environ['QDRANT_URL'] = 'https://test-qdrant.example.com'
        os.environ['QDRANT_API_KEY'] = 'test-qdrant-key'

    def tearDown(self):
        """Clean up environment variables"""
        for var in ['COHERE_API_KEY', 'QDRANT_URL', 'QDRANT_API_KEY']:
            if var in os.environ:
                del os.environ[var]

    @patch('validation.execute_semantic_query')
    def test_query_and_validation_integration(self, mock_execute_query):
        """Test the integration between execute_semantic_query and validate_retrieved_results"""
        # Mock the semantic query result
        mock_execute_query.return_value = {
            'query_text': 'test query',
            'retrieved_chunks': [
                {
                    'text': 'This is relevant content for the test query',
                    'similarity_score': 0.85,
                    'metadata': {
                        'module': 'test_module',
                        'page': 'test_page',
                        'heading': 'Test Heading',
                        'url': 'https://example.com/test'
                    },
                    'vector_id': 'test-vector-id'
                },
                {
                    'text': 'Another relevant piece of content',
                    'similarity_score': 0.75,
                    'metadata': {
                        'module': 'test_module',
                        'page': 'test_page',
                        'heading': 'Another Heading',
                        'url': 'https://example.com/test2'
                    },
                    'vector_id': 'test-vector-id2'
                }
            ],
            'execution_time_ms': 120.5
        }

        # Test the integration between query execution and result validation
        query_result = validation.execute_semantic_query(
            query_text='test query',
            collection_name='rag_embedding'
        )

        # Validate the retrieved results
        validation_result = validation.validate_retrieved_results(
            query_result['retrieved_chunks'],
            query_result['query_text']
        )

        # Verify that validation was performed
        self.assertGreaterEqual(validation_result['content_accuracy'], 0)
        self.assertGreaterEqual(validation_result['metadata_completeness'], 0)
        self.assertGreaterEqual(validation_result['relevance_score'], 0)

        # Check that no critical issues were found
        critical_issues = [issue for issue in validation_result['issues_found'] if issue['severity'] == 'critical']
        self.assertEqual(len(critical_issues), 0, f"Found critical issues: {critical_issues}")


if __name__ == '__main__':
    unittest.main()