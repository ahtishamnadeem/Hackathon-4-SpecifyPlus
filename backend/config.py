"""
Configuration management for the RAG Agent

Handles loading and validation of environment variables for the RAG agent service.
Ensures all required configuration values are present and valid before startup.
"""

import os
import logging
from typing import Optional

logger = logging.getLogger(__name__)


class ConfigurationError(Exception):
    """Raised when configuration parameters are invalid"""
    pass


def load_config() -> dict:
    """
    Load configuration from environment variables with validation
    """
    config = {}

    # Qdrant configuration
    qdrant_url = os.getenv('QDRANT_URL')
    if not qdrant_url:
        raise ConfigurationError("QDRANT_URL environment variable is required")
    config['qdrant_url'] = qdrant_url

    qdrant_api_key = os.getenv('QDRANT_API_KEY')
    if not qdrant_api_key:
        raise ConfigurationError("QDRANT_API_KEY environment variable is required")
    config['qdrant_api_key'] = qdrant_api_key

    # OpenAI configuration
    openai_api_key = os.getenv('OPENAI_API_KEY')
    if not openai_api_key:
        raise ConfigurationError("OPENAI_API_KEY environment variable is required")
    config['openai_api_key'] = openai_api_key

    # Agent configuration (with defaults)
    config['model_name'] = os.getenv('MODEL_NAME', 'gpt-4-turbo')
    config['collection_name'] = os.getenv('COLLECTION_NAME', 'rag_embedding')
    config['max_retrievals'] = int(os.getenv('MAX_RETRIEVALS', '5'))
    config['temperature'] = float(os.getenv('TEMPERATURE', '0.7'))
    config['max_tokens'] = int(os.getenv('MAX_TOKENS', '500'))

    # Server configuration (with defaults)
    config['host'] = os.getenv('HOST', '0.0.0.0')
    config['port'] = int(os.getenv('PORT', '8000'))

    logger.info("Configuration loaded successfully")
    return config


def validate_config(config: dict) -> bool:
    """
    Validate the loaded configuration
    """
    required_keys = [
        'qdrant_url', 'qdrant_api_key', 'openai_api_key',
        'model_name', 'collection_name', 'max_retrievals',
        'temperature', 'max_tokens', 'host', 'port'
    ]

    for key in required_keys:
        if key not in config:
            raise ConfigurationError(f"Missing required configuration key: {key}")

    # Validate numeric values
    if config['max_retrievals'] <= 0:
        raise ConfigurationError("MAX_RETRIEVALS must be greater than 0")

    if not 0.0 <= config['temperature'] <= 1.0:
        raise ConfigurationError("TEMPERATURE must be between 0.0 and 1.0")

    if config['max_tokens'] <= 0:
        raise ConfigurationError("MAX_TOKENS must be greater than 0")

    if config['port'] <= 0 or config['port'] > 65535:
        raise ConfigurationError("PORT must be between 1 and 65535")

    logger.info("Configuration validation passed")
    return True


# Convenience function to load and validate in one call
def load_and_validate_config() -> dict:
    """
    Load and validate configuration in a single call
    """
    config = load_config()
    validate_config(config)
    return config


# For testing purposes - if run as main, validate the current environment
if __name__ == "__main__":
    try:
        config = load_and_validate_config()
        print("Configuration loaded and validated successfully!")
        print(f"Qdrant URL: {config['qdrant_url']}")
        print(f"Model: {config['model_name']}")
        print(f"Host: {config['host']}, Port: {config['port']}")
    except ConfigurationError as e:
        print(f"Configuration error: {e}")
        exit(1)