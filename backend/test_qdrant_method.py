import sys
sys.path.insert(0, '.')

from qdrant_client import QdrantClient
from config import load_config
from dotenv import load_dotenv

load_dotenv()

# Load config to get Qdrant credentials
config = load_config()

# Initialize client
client = QdrantClient(
    url=config['qdrant_url'],
    api_key=config['qdrant_api_key'],
    timeout=30
)

# Check available methods on the client
print("Available methods on QdrantClient:")
methods = [method for method in dir(client) if not method.startswith('_')]
search_methods = [method for method in methods if 'search' in method.lower()]
print(f"Search-related methods: {search_methods}")

# Check if 'search' method exists
if hasattr(client, 'search'):
    print("[PASS] 'search' method exists")
else:
    print("[FAIL] 'search' method does NOT exist")

if hasattr(client, 'search_points'):
    print("[PASS] 'search_points' method exists")
else:
    print("[FAIL] 'search_points' method does NOT exist")

# Check all available methods that might be related to search
all_methods = [method for method in dir(client) if not method.startswith('_')]
search_like_methods = [method for method in all_methods if 'search' in method.lower() or 'retrieve' in method.lower() or 'find' in method.lower()]
print(f"All search-like methods: {search_like_methods}")

# Check if there are other common methods that might be used for vector search
common_methods = ['search', 'search_points', 'query', 'find', 'get', 'scroll', 'list']
available_common_methods = {method: hasattr(client, method) for method in common_methods}
print(f"Common method availability: {available_common_methods}")