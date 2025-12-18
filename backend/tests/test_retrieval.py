"""
Unit tests for the retrieval.py module
"""
import unittest
from unittest.mock import Mock, patch, MagicMock
import os
import sys

# Add the backend directory to the path so we can import modules
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import retrieval
from retrieval import retrieve_content, validate_qdrant_connection


class TestRetrieveContent(unittest.TestCase):
    """Test the retrieve_content function"""

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

    @patch('retrieval.initialize_qdrant_client')
    def test_retrieve_content_success(self, mock_init_qdrant_client):
        """Test successful content retrieval"""
        # Mock the Qdrant client and its search method
        mock_client = Mock()
        mock_point = Mock()
        mock_point.payload = {
            'text': 'This is test content for the query',
            'module': 'test_module',
            'page': 'test_page',
            'heading': 'Test Heading',
            'url': 'https://example.com/test',
            'title': 'Test Title'
        }
        mock_point.score = 0.85
        mock_point.id = 'test-id'

        mock_search_result = [mock_point]
        mock_client.search.return_value = mock_search_result

        mock_init_qdrant_client.return_value = mock_client

        # Call the function
        result = retrieve_content(
            query_text="test query",
            collection_name="rag_embedding",
            top_k=5
        )

        # Assertions
        self.assertEqual(result['query_text'], "test query")
        self.assertEqual(len(result['retrieved_chunks']), 1)
        self.assertEqual(result['retrieved_chunks'][0]['text'], 'This is test content for the query')
        self.assertEqual(result['retrieved_chunks'][0]['similarity_score'], 0.85)
        self.assertEqual(result['retrieved_chunks'][0]['vector_id'], 'test-id')
        self.assertEqual(result['retrieval_stats']['chunks_retrieved'], 1)

    @patch('retrieval.initialize_qdrant_client')
    def test_retrieve_content_empty_results(self, mock_init_qdrant_client):
        """Test content retrieval when no results are found"""
        # Mock the Qdrant client to return empty results
        mock_client = Mock()
        mock_client.search.return_value = []
        mock_init_qdrant_client.return_value = mock_client

        # Call the function
        result = retrieve_content(
            query_text="nonexistent query",
            collection_name="rag_embedding",
            top_k=5
        )

        # Assertions
        self.assertEqual(result['query_text'], "nonexistent query")
        self.assertEqual(len(result['retrieved_chunks']), 0)
        self.assertEqual(result['retrieval_stats']['chunks_retrieved'], 0)

    @patch('retrieval.initialize_qdrant_client')
    def test_retrieve_content_with_custom_params(self, mock_init_qdrant_client):
        """Test content retrieval with custom parameters"""
        # Mock the Qdrant client
        mock_client = Mock()
        mock_point = Mock()
        mock_point.payload = {
            'text': 'Custom param test content',
            'module': 'custom_module',
            'page': 'custom_page',
            'heading': 'Custom Heading',
            'url': 'https://example.com/custom',
            'title': 'Custom Title'
        }
        mock_point.score = 0.92
        mock_point.id = 'custom-id'

        mock_search_result = [mock_point]
        mock_client.search.return_value = mock_search_result
        mock_init_qdrant_client.return_value = mock_client

        # Call the function with custom parameters
        result = retrieve_content(
            query_text="custom query",
            collection_name="custom_collection",
            top_k=3
        )

        # Verify the search was called with the right parameters
        mock_client.search.assert_called_once_with(
            collection_name="custom_collection",
            query_vector=unittest.mock.ANY,  # We don't care about the specific embedding in this test
            limit=3,
            with_payload=True,
            with_vectors=False
        )

        # Assertions
        self.assertEqual(result['query_text'], "custom query")
        self.assertEqual(len(result['retrieved_chunks']), 1)
        self.assertEqual(result['retrieved_chunks'][0]['text'], 'Custom param test content')

    def test_retrieve_content_missing_params(self):
        """Test retrieve_content with missing required parameters"""
        # This should raise an exception due to missing environment variables
        # that are needed for configuration loading
        with self.assertRaises(Exception):
            # Temporarily remove environment variables to trigger config error
            original_env = os.environ.copy()
            for var in ['QDRANT_URL', 'QDRANT_API_KEY', 'OPENAI_API_KEY']:
                if var in os.environ:
                    del os.environ[var]

            try:
                retrieve_content(query_text="test query")
            finally:
                # Restore environment variables
                os.environ.clear()
                os.environ.update(original_env)


class TestValidateQdrantConnection(unittest.TestCase):
    """Test the validate_qdrant_connection function"""

    def setUp(self):
        """Set up test environment"""
        # Set up minimal required environment variables for testing
        os.environ['QDRANT_URL'] = 'https://test-qdrant.example.com'
        os.environ['QDRANT_API_KEY'] = 'test-qdrant-key'

    def tearDown(self):
        """Clean up test environment"""
        # Remove test environment variables
        for var in ['QDRANT_URL', 'QDRANT_API_KEY']:
            if var in os.environ:
                del os.environ[var]

    @patch('retrieval.initialize_qdrant_client')
    def test_validate_qdrant_connection_success(self, mock_init_qdrant_client):
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

        mock_init_qdrant_client.return_value = mock_client

        # Call the function
        result = validate_qdrant_connection(collection_name="rag_embedding")

        # Assertions
        self.assertTrue(result['connected'])
        self.assertTrue(result['collection_exists'])
        self.assertEqual(result['vector_count'], 150)
        self.assertIsNotNone(result['collection_config'])

    @patch('retrieval.initialize_qdrant_client')
    def test_validate_qdrant_connection_collection_not_found(self, mock_init_qdrant_client):
        """Test Qdrant connection validation when collection doesn't exist"""
        # Mock the Qdrant client and its methods
        mock_client = Mock()
        mock_other_collection = Mock()
        mock_other_collection.name = 'other_collection'  # Different collection name
        mock_collections_response = Mock()
        mock_collections_response.collections = [mock_other_collection]

        mock_client.get_collections.return_value = mock_collections_response

        mock_init_qdrant_client.return_value = mock_client

        # Call the function - should return collection_exists=False
        result = validate_qdrant_connection(collection_name="rag_embedding")

        # Assertions
        self.assertTrue(result['connected'])  # Connected to Qdrant but collection doesn't exist
        self.assertFalse(result['collection_exists'])
        self.assertEqual(result['vector_count'], 0)  # No vectors in non-existent collection


if __name__ == '__main__':
    unittest.main()