import os
from dotenv import load_dotenv

# Load environment variables from .env file
load_dotenv()

print("Environment variables loaded:")
print(f"QDRANT_URL: {os.getenv('QDRANT_URL', 'NOT SET')}")
print(f"OPENAI_API_KEY: {os.getenv('OPENAI_API_KEY', 'NOT SET')[:20]}...")
print(f"NEON_DATABASE_URL: {os.getenv('NEON_DATABASE_URL', 'NOT SET')}")
print(f"COHERE_API_KEY: {os.getenv('COHERE_API_KEY', 'NOT SET')}")

# Now try to load the config
from config import load_config
config = load_config()
print("\nConfiguration loaded successfully!")
print(f"Model: {config.get('model_name', 'Unknown')}")
print(f"Max tokens: {config.get('max_tokens', 'Unknown')}")