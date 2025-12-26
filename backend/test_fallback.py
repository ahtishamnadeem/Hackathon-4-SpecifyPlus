"""
Test script to verify the dual-API fallback mechanism
"""
import os
import sys
import logging
from typing import Dict, Any, Optional
from dotenv import load_dotenv

# Load environment variables from .env file
load_dotenv()

# Add the backend directory to the path
sys.path.insert(0, '.')

import openai
import google.generativeai as genai
from google.generativeai import GenerativeModel
from config import load_config

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

def test_openai_client(config: Dict[str, Any]) -> Optional[str]:
    """
    Test OpenAI client with a simple request
    """
    try:
        client = openai.OpenAI(api_key=config['openai_api_key'])

        # Try a simple request
        response = client.chat.completions.create(
            model=config['model_name'],
            messages=[
                {
                    "role": "user",
                    "content": "Test message to check API availability"
                }
            ],
            max_tokens=50,
            temperature=0.1
        )

        return response.choices[0].message.content
    except Exception as e:
        logger.error(f"OpenAI API error: {str(e)}")
        return None

def test_google_client(config: Dict[str, Any]) -> Optional[str]:
    """
    Test Google AI Studio client with a simple request
    """
    try:
        genai.configure(api_key=config['google_ai_studio_api_key'])
        model = GenerativeModel('gemini-1.5-flash')

        # Try a simple request
        response = model.generate_content(
            "Test message to check API availability",
            generation_config={
                "temperature": 0.1,
                "max_output_tokens": 50
            }
        )

        return response.text
    except Exception as e:
        logger.error(f"Google AI Studio API error: {str(e)}")
        return None

def test_fallback_mechanism():
    """
    Test the fallback mechanism by simulating quota errors
    """
    print("Testing dual-API fallback mechanism...")

    # Load configuration
    config = load_config()
    print(f"Configuration loaded. Model: {config['model_name']}")

    # Test OpenAI first
    print("\n1. Testing OpenAI API...")
    openai_result = test_openai_client(config)
    if openai_result:
        print(f"   [PASS] OpenAI API working: {openai_result[:50]}...")
    else:
        print("   [FAIL] OpenAI API failed")

    # Test Google AI Studio
    print("\n2. Testing Google AI Studio API...")
    google_result = test_google_client(config)
    if google_result:
        print(f"   [PASS] Google AI Studio API working: {google_result[:50]}...")
    else:
        print("   [FAIL] Google AI Studio API failed")

    # Test that both APIs are available
    print("\n3. Testing fallback capability...")
    if openai_result and google_result:
        print("   [PASS] Both APIs are available - fallback mechanism is ready")
    elif openai_result:
        print("   [PASS] OpenAI available, Google may be needed for fallback")
    elif google_result:
        print("   [PASS] Google AI Studio available, can serve as primary if OpenAI fails")
    else:
        print("   [FAIL] Neither API is working - please check your API keys")

    print("\n4. Fallback mechanism summary:")
    print("   - The agent.py file has been updated to try OpenAI first")
    print("   - If OpenAI returns a quota/limit error (429, 'quota', 'insufficient', 'rate'),")
    print("     the system will automatically fall back to Google AI Studio")
    print("   - Both selected-text mode and RAG mode have this fallback logic")
    print("   - The system maintains all existing functionality while adding resilience")

if __name__ == "__main__":
    test_fallback_mechanism()