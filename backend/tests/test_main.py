"""
Unit tests for main.py functions
"""
import unittest
from unittest.mock import Mock, patch, MagicMock
import os
import sys

# Add the backend directory to the path so we can import main
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import main


class TestEnvironmentVariables(unittest.TestCase):
    """Test environment variable loading and validation"""

    def setUp(self):
        # Clear environment variables for testing
        for var in ['COHERE_API_KEY', 'QDRANT_URL', 'QDRANT_API_KEY', 'BOOK_URL']:
            if var in os.environ:
                del os.environ[var]

    def test_load_environment_variables_missing_vars(self):
        """Test that missing environment variables raise ConfigurationError"""
        with self.assertRaises(main.ConfigurationError):
            main.load_environment_variables()

    def test_load_environment_variables_with_vars(self):
        """Test that environment variables are loaded correctly"""
        os.environ['COHERE_API_KEY'] = 'test-key'
        os.environ['QDRANT_URL'] = 'https://test.qdrant.example.com'
        os.environ['QDRANT_API_KEY'] = 'test-qdrant-key'

        result = main.load_environment_variables()

        self.assertEqual(result['cohere_api_key'], 'test-key')
        self.assertEqual(result['qdrant_url'], 'https://test.qdrant.example.com')
        self.assertEqual(result['qdrant_api_key'], 'test-qdrant-key')


class TestConfigurationConstants(unittest.TestCase):
    """Test configuration constant creation"""

    def test_create_configuration_constants(self):
        """Test that configuration constants are created with default values"""
        # Ensure default values are used
        if 'CHUNK_SIZE' in os.environ:
            del os.environ['CHUNK_SIZE']
        if 'CHUNK_OVERLAP' in os.environ:
            del os.environ['CHUNK_OVERLAP']
        if 'RATE_LIMIT' in os.environ:
            del os.environ['RATE_LIMIT']

        main.create_configuration_constants()

        # Values should be set to defaults
        self.assertEqual(main.CHUNK_SIZE, 800)
        self.assertEqual(main.CHUNK_OVERLAP, 160)
        self.assertEqual(main.RATE_LIMIT, 1.0)


class TestChunkTextFunction(unittest.TestCase):
    """Test the chunk_text function"""

    def test_chunk_text_basic(self):
        """Test basic text chunking functionality"""
        text = "This is a test sentence. " * 10  # 30 words
        url = "https://example.com/test"
        title = "Test Page"
        module = "module1"
        page = "page1"

        chunks = main.chunk_text(
            text=text,
            url=url,
            title=title,
            module=module,
            page=page,
            chunk_size=5,
            chunk_overlap=1
        )

        # Should create multiple chunks
        self.assertGreater(len(chunks), 1)

        # Each chunk should have proper structure
        for chunk in chunks:
            self.assertIn('text', chunk)
            self.assertIn('metadata', chunk)
            self.assertEqual(chunk['metadata']['url'], url)
            self.assertEqual(chunk['metadata']['title'], title)
            self.assertEqual(chunk['metadata']['module'], module)
            self.assertEqual(chunk['metadata']['page'], page)

    def test_chunk_text_empty_text(self):
        """Test chunking with empty text"""
        chunks = main.chunk_text(
            text="",
            url="https://example.com/test",
            title="Test Page",
            module="module1",
            page="page1"
        )

        self.assertEqual(chunks, [])

    def test_chunk_text_invalid_sizes(self):
        """Test that invalid chunk sizes raise errors"""
        with self.assertRaises(main.InvalidChunkSizeError):
            main.chunk_text(
                text="test",
                url="https://example.com/test",
                title="Test Page",
                module="module1",
                page="page1",
                chunk_size=-1
            )

        with self.assertRaises(main.InvalidChunkSizeError):
            main.chunk_text(
                text="test",
                url="https://example.com/test",
                title="Test Page",
                module="module1",
                page="page1",
                chunk_size=10,
                chunk_overlap=15  # Overlap greater than chunk size
            )


class TestGetAllUrls(unittest.TestCase):
    """Test the get_all_urls function"""

    @patch('main.requests.get')
    @patch('main.BeautifulSoup')
    def test_get_all_urls_success(self, mock_bs, mock_get):
        """Test successful URL retrieval from sitemap"""
        # Mock the response
        mock_response = Mock()
        mock_response.content = b'<urlset><url><loc>https://example.com/page1</loc></url><url><loc>https://example.com/page2</loc></url></urlset>'
        mock_response.raise_for_status.return_value = None
        mock_get.return_value = mock_response

        # Mock BeautifulSoup to return the structure we expect
        mock_soup_instance = Mock()

        # Create proper mock objects for the XML elements
        mock_url_element1 = Mock()
        mock_loc_element1 = Mock()
        mock_loc_element1.text = 'https://example.com/page1'
        mock_url_element1.find.return_value = mock_loc_element1
        mock_url_element1.name = 'url'

        mock_url_element2 = Mock()
        mock_loc_element2 = Mock()
        mock_loc_element2.text = 'https://example.com/page2'
        mock_url_element2.find.return_value = mock_loc_element2
        mock_url_element2.name = 'url'

        # Alternative: mock <loc> elements directly
        mock_loc_direct1 = Mock()
        mock_loc_direct1.text = 'https://example.com/page1'
        mock_loc_direct1.name = 'loc'

        mock_loc_direct2 = Mock()
        mock_loc_direct2.text = 'https://example.com/page2'
        mock_loc_direct2.name = 'loc'

        # The find_all should return both types of elements
        mock_soup_instance.find_all.return_value = [mock_loc_direct1, mock_loc_direct2]

        mock_bs.return_value = mock_soup_instance

        urls = main.get_all_urls("https://example.com")

        self.assertEqual(len(urls), 2)
        self.assertIn('https://example.com/page1', urls)
        self.assertIn('https://example.com/page2', urls)


class TestExtractTextFromUrl(unittest.TestCase):
    """Test the extract_text_from_url function"""

    @patch('main.requests.get')
    def test_extract_text_from_url_success(self, mock_get):
        """Test successful text extraction from URL"""
        # Mock the response
        mock_response = Mock()
        mock_response.content = b'<html><head><title>Test Page</title></head><body><h1>Main Title</h1><p>This is test content.</p></body></html>'
        mock_response.raise_for_status.return_value = None
        mock_get.return_value = mock_response

        result = main.extract_text_from_url("https://example.com/test")

        self.assertEqual(result['title'], 'Test Page')
        self.assertIn('test content', result['text'].lower())
        self.assertEqual(result['module'], 'test')  # From URL path
        self.assertEqual(result['page'], 'test')    # From URL path
        self.assertIn('Main Title', result['headings'])


class TestCreateCollection(unittest.TestCase):
    """Test the create_collection function"""

    @patch('main.initialize_qdrant_client')
    def test_create_collection_success(self, mock_init_client):
        """Test successful collection creation"""
        # Mock the Qdrant client
        mock_client = Mock()
        mock_collection = Mock()
        mock_collection.name = 'existing_collection'
        mock_client.get_collections.return_value = Mock(collections=[mock_collection])
        mock_init_client.return_value = mock_client

        # Test creating a new collection
        result = main.create_collection("test_collection")

        # Should call create_collection method on the client
        mock_client.create_collection.assert_called_once()
        self.assertTrue(result)


class TestSaveChunkToQdrant(unittest.TestCase):
    """Test the save_chunk_to_qdrant function"""

    @patch('main.initialize_qdrant_client')
    @patch('builtins.hash')
    def test_save_chunk_to_qdrant_success(self, mock_hash, mock_init_client):
        """Test successful saving of chunk to Qdrant"""
        # Mock the Qdrant client
        mock_client = Mock()
        mock_init_client.return_value = mock_client
        mock_hash.return_value = 12345  # Mock hash to avoid randomness

        # Test data
        chunk = {
            'text': 'test content',
            'metadata': {
                'url': 'https://example.com/test',
                'title': 'Test Page',
                'module': 'module1',
                'page': 'page1',
                'heading': 'Test Heading'
            }
        }
        embedding = [0.1, 0.2, 0.3]

        result = main.save_chunk_to_qdrant(chunk, embedding, "test_collection")

        # Should call upsert method on the client
        mock_client.upsert.assert_called_once()
        # Result should be the record ID
        self.assertIsNotNone(result)


if __name__ == '__main__':
    unittest.main()