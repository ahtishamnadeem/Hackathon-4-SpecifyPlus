"""
Unit tests for the agent.py module
"""
import unittest
from unittest.mock import Mock, patch, MagicMock
import os
import sys

# Add the backend directory to the path so we can import modules
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import agent
from agent import process_query, validate_grounding


class TestProcessQuery(unittest.TestCase):
    """Test the process_query function"""

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

    @patch('agent.initialize_openai_client')
    @patch('agent.retrieve_content')
    def test_process_query_success(self, mock_retrieve_content, mock_init_openai_client):
        """Test successful query processing"""
        # Mock the OpenAI client
        mock_openai_client = Mock()
        mock_choice = Mock()
        mock_choice.message.content = "This is a test answer based on the context."
        mock_usage = Mock()
        mock_usage.total_tokens = 50
        mock_usage.prompt_tokens = 30
        mock_usage.completion_tokens = 20

        mock_response = Mock()
        mock_response.choices = [mock_choice]
        mock_response.usage = mock_usage

        mock_openai_client.chat.completions.create.return_value = mock_response
        mock_init_openai_client.return_value = mock_openai_client

        # Mock the retrieval function
        mock_retrieve_content.return_value = {
            'query_text': 'test query',
            'retrieved_chunks': [
                {
                    'text': 'This is relevant content for the test query',
                    'similarity_score': 0.85,
                    'metadata': {
                        'module': 'test_module',
                        'page': 'test_page',
                        'heading': 'Test Heading',
                        'url': 'https://example.com/test',
                        'title': 'Test Title'
                    },
                    'vector_id': 'test-vector-id'
                }
            ],
            'retrieval_stats': {
                'chunks_retrieved': 1,
                'execution_time_ms': 120.5,
                'collection_name': 'rag_embedding',
                'top_k_requested': 5
            }
        }

        # Call the function
        result = process_query(
            query_text="test query",
            max_tokens=200,
            temperature=0.7
        )

        # Assertions
        self.assertEqual(result['query_text'], "test query")
        self.assertEqual(result['answer'], "This is a test answer based on the context.")
        self.assertEqual(len(result['sources']), 1)
        self.assertEqual(result['confidence_score'], 0.85)  # Average similarity score
        self.assertEqual(result['generation_details']['tokens_used'], 50)

    @patch('agent.initialize_openai_client')
    @patch('agent.retrieve_content')
    def test_process_query_with_custom_params(self, mock_retrieve_content, mock_init_openai_client):
        """Test query processing with custom parameters"""
        # Mock the OpenAI client
        mock_openai_client = Mock()
        mock_choice = Mock()
        mock_choice.message.content = "Custom parameters answer."
        mock_usage = Mock()
        mock_usage.total_tokens = 75
        mock_usage.prompt_tokens = 45
        mock_usage.completion_tokens = 30

        mock_response = Mock()
        mock_response.choices = [mock_choice]
        mock_response.usage = mock_usage

        mock_openai_client.chat.completions.create.return_value = mock_response
        mock_init_openai_client.return_value = mock_openai_client

        # Mock the retrieval function
        mock_retrieve_content.return_value = {
            'query_text': 'custom params query',
            'retrieved_chunks': [
                {
                    'text': 'Custom parameters content',
                    'similarity_score': 0.92,
                    'metadata': {
                        'module': 'custom_module',
                        'page': 'custom_page',
                        'heading': 'Custom Heading',
                        'url': 'https://example.com/custom',
                        'title': 'Custom Title'
                    },
                    'vector_id': 'custom-vector-id'
                }
            ],
            'retrieval_stats': {
                'chunks_retrieved': 1,
                'execution_time_ms': 150.0,
                'collection_name': 'rag_embedding',
                'top_k_requested': 5
            }
        }

        # Call the function with custom parameters
        result = process_query(
            query_text="custom params query",
            max_tokens=300,
            temperature=0.5
        )

        # Verify the OpenAI call was made with the right parameters
        mock_openai_client.chat.completions.create.assert_called_once()

        # Assertions
        self.assertEqual(result['query_text'], "custom params query")
        self.assertEqual(result['answer'], "Custom parameters answer.")
        self.assertEqual(result['confidence_score'], 0.92)  # Average similarity score

    @patch('agent.initialize_openai_client')
    @patch('agent.retrieve_content')
    def test_process_query_empty_retrieval(self, mock_retrieve_content, mock_init_openai_client):
        """Test query processing when retrieval returns no results"""
        # Mock the OpenAI client
        mock_openai_client = Mock()
        mock_choice = Mock()
        mock_choice.message.content = "No context available to answer this question."
        mock_usage = Mock()
        mock_usage.total_tokens = 40
        mock_usage.prompt_tokens = 25
        mock_usage.completion_tokens = 15

        mock_response = Mock()
        mock_response.choices = [mock_choice]
        mock_response.usage = mock_usage

        mock_openai_client.chat.completions.create.return_value = mock_response
        mock_init_openai_client.return_value = mock_openai_client

        # Mock the retrieval function to return empty results
        mock_retrieve_content.return_value = {
            'query_text': 'empty query',
            'retrieved_chunks': [],
            'retrieval_stats': {
                'chunks_retrieved': 0,
                'execution_time_ms': 50.0,
                'collection_name': 'rag_embedding',
                'top_k_requested': 5
            }
        }

        # Call the function
        result = process_query(
            query_text="empty query",
            max_tokens=150,
            temperature=0.7
        )

        # Assertions
        self.assertEqual(result['query_text'], "empty query")
        self.assertEqual(result['answer'], "No context available to answer this question.")
        self.assertEqual(len(result['sources']), 0)
        self.assertEqual(result['confidence_score'], 0.0)  # No similarity score when no content retrieved

    def test_process_query_missing_config(self):
        """Test process_query with missing configuration"""
        # Temporarily remove environment variables to trigger config error
        original_env = os.environ.copy()
        for var in ['QDRANT_URL', 'QDRANT_API_KEY', 'OPENAI_API_KEY']:
            if var in os.environ:
                del os.environ[var]

        try:
            # This should raise an exception due to missing config
            with self.assertRaises(Exception):
                process_query(query_text="test query")
        finally:
            # Restore environment variables
            os.environ.clear()
            os.environ.update(original_env)


class TestValidateGrounding(unittest.TestCase):
    """Test the validate_grounding function"""

    def test_validate_grounding_good_match(self):
        """Test grounding validation with good content match"""
        generated_answer = "The robot operating system is a middleware for robotics applications."
        retrieved_chunks = [
            {
                'text': 'ROS (Robot Operating System) is a flexible framework for writing robot software.',
                'similarity_score': 0.85,
                'metadata': {'module': 'intro', 'page': 'overview', 'heading': 'Introduction', 'url': 'https://example.com/intro', 'title': 'Intro'}
            },
            {
                'text': 'It provides services designed for a heterogeneous computer cluster such as hardware abstraction, device drivers, libraries, visualizers, message-passing, package management.',
                'similarity_score': 0.75,
                'metadata': {'module': 'features', 'page': 'features', 'heading': 'Features', 'url': 'https://example.com/features', 'title': 'Features'}
            }
        ]

        result = validate_grounding(generated_answer, retrieved_chunks)

        # Assertions
        self.assertTrue(result['is_valid'])
        self.assertGreater(result['grounding_ratio'], 0.0)  # Should have some match
        self.assertEqual(len(result['issues_found']), 0)  # No issues with good grounding

    def test_validate_grounding_poor_match(self):
        """Test grounding validation with poor content match"""
        generated_answer = "This answer has nothing to do with robotics or ROS."
        retrieved_chunks = [
            {
                'text': 'ROS (Robot Operating System) is a flexible framework for writing robot software.',
                'similarity_score': 0.85,
                'metadata': {'module': 'intro', 'page': 'overview', 'heading': 'Introduction', 'url': 'https://example.com/intro', 'title': 'Intro'}
            }
        ]

        result = validate_grounding(generated_answer, retrieved_chunks)

        # Assertions
        self.assertFalse(result['is_valid'])  # Poor match should not be valid
        self.assertLess(result['grounding_ratio'], 0.3)  # Low ratio due to poor match
        self.assertGreater(len(result['issues_found']), 0)  # Should have issues with poor grounding

    def test_validate_grounding_empty_chunks(self):
        """Test grounding validation with empty retrieved chunks"""
        generated_answer = "Any answer here."
        retrieved_chunks = []  # Empty list

        result = validate_grounding(generated_answer, retrieved_chunks)

        # Assertions
        self.assertFalse(result['is_valid'])  # Cannot ground to empty content
        self.assertEqual(result['grounding_ratio'], 0.0)  # No content to ground to
        self.assertGreater(len(result['issues_found']), 0)  # Should have issues


if __name__ == '__main__':
    unittest.main()