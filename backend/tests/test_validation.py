"""
Unit tests for validation.py functions
"""
import unittest
from unittest.mock import Mock, patch, MagicMock
import os
import sys

# Add the backend directory to the path so we can import validation
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import validation


class TestValidateQdrantConnection(unittest.TestCase):
    """Test the validate_qdrant_connection function"""

    @patch('validation.QdrantClient')
    def test_validate_qdrant_connection_success(self, mock_qdrant_client):
        """Test successful Qdrant connection validation"""
        # Mock the Qdrant client and its methods
        mock_client = Mock()
        mock_collection = Mock()
        mock_collection.name = 'rag_embedding'
        mock_collections_response = Mock()
        mock_collections_response.collections = [mock_collection]

        mock_client.get_collections.return_value = mock_collections_response

        # Mock the get_collection method to return collection info
        mock_collection_info = Mock()
        mock_collection_info.points_count = 150
        mock_client.get_collection.return_value = mock_collection_info

        mock_qdrant_client.return_value = mock_client

        # Call the function
        result = validation.validate_qdrant_connection(
            qdrant_url="https://test.qdrant.example.com",
            qdrant_api_key="test-key",
            collection_name="rag_embedding"
        )

        # Assertions
        self.assertTrue(result['connected'])
        self.assertTrue(result['collection_exists'])
        self.assertEqual(result['vector_count'], 150)
        self.assertIsNotNone(result['collection_config'])

    @patch('validation.QdrantClient')
    def test_validate_qdrant_connection_collection_not_found(self, mock_qdrant_client):
        """Test Qdrant connection validation when collection doesn't exist"""
        # Mock the Qdrant client and its methods
        mock_client = Mock()
        mock_collection = Mock()
        mock_collection.name = 'other_collection'  # Different collection name
        mock_collections_response = Mock()
        mock_collections_response.collections = [mock_collection]

        mock_client.get_collections.return_value = mock_collections_response

        mock_qdrant_client.return_value = mock_client

        # Call the function - should raise NotFoundError
        with self.assertRaises(validation.NotFoundError):
            validation.validate_qdrant_connection(
                qdrant_url="https://test.qdrant.example.com",
                qdrant_api_key="test-key",
                collection_name="rag_embedding"
            )

    def test_validate_qdrant_connection_missing_credentials(self):
        """Test Qdrant connection validation with missing credentials"""
        # Temporarily clear environment variables to test missing credentials
        original_qdrant_url = os.environ.get('QDRANT_URL')
        original_qdrant_api_key = os.environ.get('QDRANT_API_KEY')

        # Remove environment variables if they exist
        if 'QDRANT_URL' in os.environ:
            del os.environ['QDRANT_URL']
        if 'QDRANT_API_KEY' in os.environ:
            del os.environ['QDRANT_API_KEY']

        try:
            # Call the function with missing credentials
            with self.assertRaises(validation.ConfigurationError):
                validation.validate_qdrant_connection(
                    qdrant_url=None,
                    qdrant_api_key=None,
                    collection_name="rag_embedding"
                )
        finally:
            # Restore original environment variables
            if original_qdrant_url is not None:
                os.environ['QDRANT_URL'] = original_qdrant_url
            if original_qdrant_api_key is not None:
                os.environ['QDRANT_API_KEY'] = original_qdrant_api_key


class TestExecuteSemanticQuery(unittest.TestCase):
    """Test the execute_semantic_query function"""

    @patch('validation.initialize_cohere_client')
    @patch('validation.initialize_qdrant_client')
    def test_execute_semantic_query_success(self, mock_qdrant_client, mock_cohere_client):
        """Test successful semantic query execution"""
        # Mock Cohere client
        mock_cohere = Mock()
        mock_cohere.embed.return_value = Mock(embeddings=[[0.1, 0.2, 0.3]])
        mock_cohere_client.return_value = mock_cohere

        # Mock Qdrant client
        mock_qdrant = Mock()
        mock_point = Mock()
        mock_point.payload = {'text': 'test content', 'module': 'test', 'page': 'page1', 'heading': 'heading', 'url': 'https://example.com', 'title': 'title'}
        mock_point.score = 0.8
        mock_point.id = 'test-id'
        mock_qdrant.search.return_value = [mock_point]
        mock_qdrant_client.return_value = mock_qdrant

        # Call the function
        result = validation.execute_semantic_query(
            query_text="test query",
            collection_name="rag_embedding",
            top_k=5
        )

        # Assertions
        self.assertEqual(result['query_text'], "test query")
        self.assertEqual(len(result['retrieved_chunks']), 1)
        self.assertGreater(result['execution_time_ms'], 0)  # Execution time should be measured


class TestValidateRetrievedResults(unittest.TestCase):
    """Test the validate_retrieved_results function"""

    def test_validate_retrieved_results_success(self):
        """Test successful validation of retrieved results"""
        retrieved_chunks = [
            {
                'text': 'This is valid content',
                'similarity_score': 0.8,
                'metadata': {
                    'module': 'test',
                    'page': 'page1',
                    'heading': 'heading',
                    'url': 'https://example.com'
                },
                'vector_id': 'test-id'
            }
        ]

        result = validation.validate_retrieved_results(retrieved_chunks, "test query")

        # Should have high content accuracy and metadata completeness
        self.assertGreaterEqual(result['content_accuracy'], 0.9)
        self.assertGreaterEqual(result['metadata_completeness'], 0.9)
        self.assertGreaterEqual(result['relevance_score'], 0.7)
        self.assertEqual(len(result['issues_found']), 0)

    def test_validate_retrieved_results_empty_chunks(self):
        """Test validation with empty chunks"""
        result = validation.validate_retrieved_results([], "test query")

        self.assertEqual(result['content_accuracy'], 0.0)
        self.assertEqual(result['metadata_completeness'], 0.0)
        self.assertEqual(result['relevance_score'], 0.0)
        self.assertTrue(len(result['issues_found']) > 0)
        self.assertTrue(any(issue['type'] == 'empty_results' for issue in result['issues_found']))

    def test_validate_retrieved_results_missing_metadata(self):
        """Test validation with missing metadata"""
        retrieved_chunks = [
            {
                'text': 'This is valid content',
                'similarity_score': 0.8,
                'metadata': {
                    'module': 'test',
                    # Missing some required fields
                },
                'vector_id': 'test-id'
            }
        ]

        result = validation.validate_retrieved_results(retrieved_chunks, "test query")

        # Should have some metadata issues
        self.assertLess(result['metadata_completeness'], 1.0)
        self.assertTrue(len(result['issues_found']) > 0)


class TestConfirmPipelineReadiness(unittest.TestCase):
    """Test the confirm_pipeline_readiness function"""

    def test_confirm_pipeline_readiness_ready(self):
        """Test pipeline readiness when all criteria are met"""
        validation_results = [
            {
                'content_accuracy': 0.95,
                'metadata_completeness': 0.98,
                'relevance_score': 0.75
            },
            {
                'content_accuracy': 0.92,
                'metadata_completeness': 0.96,
                'relevance_score': 0.72
            }
        ]

        result = validation.confirm_pipeline_readiness(
            validation_results,
            minimum_accuracy_threshold=0.9,
            minimum_metadata_completeness=0.95
        )

        self.assertTrue(result['is_ready_for_agent_integration'])
        self.assertEqual(result['confidence_level'], 'medium')
        self.assertEqual(len(result['recommendations']), 1)  # Should have positive recommendation

    def test_confirm_pipeline_readiness_not_ready(self):
        """Test pipeline readiness when criteria are not met"""
        validation_results = [
            {
                'content_accuracy': 0.7,
                'metadata_completeness': 0.8,
                'relevance_score': 0.5
            }
        ]

        result = validation.confirm_pipeline_readiness(
            validation_results,
            minimum_accuracy_threshold=0.9,
            minimum_metadata_completeness=0.95
        )

        self.assertFalse(result['is_ready_for_agent_integration'])
        self.assertIn('increase content accuracy', result['recommendations'][0].lower())
        self.assertIn('increase metadata completeness', result['recommendations'][1].lower())


class TestMainFunction(unittest.TestCase):
    """Test the main validation function"""

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

    @patch('validation.validate_qdrant_connection')
    @patch('validation.execute_semantic_query')
    @patch('validation.validate_retrieved_results')
    @patch('validation.confirm_pipeline_readiness')
    def test_main_function_success(
        self,
        mock_confirm_readiness,
        mock_validate_results,
        mock_execute_query,
        mock_validate_connection
    ):
        """Test successful execution of the main validation function"""
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
            test_queries=['test query'],
            top_k=5
        )

        # Verify the result structure
        self.assertIn('connection_status', result)
        self.assertIn('query_results', result)
        self.assertIn('validation_findings', result)
        self.assertIn('readiness_assessment', result)
        self.assertIn('execution_summary', result)

        # Verify that the mocked functions were called
        mock_validate_connection.assert_called_once()
        mock_execute_query.assert_called()
        mock_validate_results.assert_called()
        mock_confirm_readiness.assert_called_once()

    @patch('validation.validate_qdrant_connection')
    def test_main_function_with_error(self, mock_validate_connection):
        """Test main validation function with error handling"""
        # Mock validate_qdrant_connection to raise an exception
        mock_validate_connection.side_effect = validation.QdrantConnectionError("Failed to connect to Qdrant")

        # Test that the error is propagated
        with self.assertRaises(validation.PipelineValidationError):
            validation.main(
                qdrant_url='https://test-qdrant.example.com',
                qdrant_api_key='test-key',
                cohere_api_key='test-cohere-key'
            )


if __name__ == '__main__':
    unittest.main()