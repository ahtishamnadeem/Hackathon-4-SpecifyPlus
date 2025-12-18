"""
RAG Agent Logic Module

Implements the core agent logic that processes user queries, retrieves relevant content
from Qdrant, and generates accurate answers using the retrieved information.
"""

import os
import logging
import time
from typing import Dict, Any, List, Optional
import openai

from config import load_config
from retrieval import retrieve_content

logger = logging.getLogger(__name__)


def initialize_openai_client() -> openai.OpenAI:
    """
    Initialize OpenAI client with connection validation
    """
    config = load_config()

    try:
        client = openai.OpenAI(api_key=config['openai_api_key'])

        # Test connection by making a simple API call
        client.models.list()
        logger.info("Successfully connected to OpenAI API")
        return client
    except Exception as e:
        logger.error(f"Failed to connect to OpenAI: {str(e)}")
        raise ConnectionError(f"Could not connect to OpenAI: {str(e)}")


def process_query(query_text: str, max_tokens: Optional[int] = None, temperature: Optional[float] = None) -> Dict[str, Any]:
    """
    Process a user query by retrieving relevant content and generating an answer

    Args:
        query_text: The user's query text
        max_tokens: Maximum tokens for the response (defaults to config value)
        temperature: Temperature for response generation (defaults to config value)

    Returns:
        Dictionary containing the generated answer, sources, and metadata
    """
    start_time = time.time()

    # Get configuration
    config = load_config()
    if max_tokens is None:
        max_tokens = config['max_tokens']
    if temperature is None:
        temperature = config['temperature']

    try:
        # Step 1: Retrieve relevant content from Qdrant
        retrieval_result = retrieve_content(
            query_text=query_text,
            collection_name=config['collection_name'],
            top_k=config['max_retrievals']
        )

        retrieved_chunks = retrieval_result['retrieved_chunks']

        # Step 2: Prepare context for the language model
        context_texts = []
        sources = []

        for chunk in retrieved_chunks:
            context_texts.append(chunk['text'])
            sources.append({
                'text': chunk['text'][:200] + "...",  # Shortened preview
                'url': chunk['metadata']['url'],
                'page': chunk['metadata']['page'],
                'heading': chunk['metadata']['heading'],
                'similarity_score': chunk['similarity_score']
            })

        # Combine the context
        context = "\n\n".join(context_texts)

        # Step 3: Generate response using OpenAI
        openai_client = initialize_openai_client()

        # Format the prompt with context
        prompt = f"""
        You are a helpful assistant that answers questions based on the provided context.
        Answer the user's question using only the information provided in the context.
        If the context doesn't contain enough information to answer the question, say so.

        Context:
        {context}

        Question: {query_text}

        Answer:
        """

        response = openai_client.chat.completions.create(
            model=config['model_name'],
            messages=[
                {"role": "system", "content": "You are a helpful assistant that answers questions based on provided context. Only use information from the context to answer. If the context doesn't contain the information needed, say so."},
                {"role": "user", "content": prompt}
            ],
            max_tokens=max_tokens,
            temperature=temperature
        )

        generated_answer = response.choices[0].message.content
        usage = response.usage

        # Step 4: Calculate confidence score based on similarity scores
        confidence_score = 0.0
        if retrieved_chunks:
            avg_similarity = sum([chunk['similarity_score'] for chunk in retrieved_chunks]) / len(retrieved_chunks)
            confidence_score = avg_similarity

        # Step 5: Prepare the final response
        execution_time = (time.time() - start_time) * 1000  # Convert to milliseconds

        result = {
            'query_text': query_text,
            'answer': generated_answer,
            'sources': sources,
            'confidence_score': confidence_score,
            'retrieval_details': retrieval_result,
            'generation_details': {
                'model_used': config['model_name'],
                'tokens_used': usage.total_tokens if usage else 0,
                'input_tokens': usage.prompt_tokens if usage else 0,
                'output_tokens': usage.completion_tokens if usage else 0
            },
            'processing_time_ms': execution_time
        }

        logger.info(f"Processed query: '{query_text[:50]}...' in {execution_time:.2f}ms with confidence {confidence_score:.2f}")
        return result

    except Exception as e:
        logger.error(f"Failed to process query: {str(e)}")
        raise AgentError(f"Could not process query: {str(e)}")


def validate_grounding(generated_answer: str, retrieved_chunks: List[Dict[str, Any]]) -> Dict[str, Any]:
    """
    Validate that the generated answer is properly grounded in the retrieved content

    Args:
        generated_answer: The answer generated by the agent
        retrieved_chunks: The chunks retrieved from the vector store

    Returns:
        Dictionary containing validation results and any issues found
    """
    try:
        # This is a simplified grounding validation
        # In a more sophisticated implementation, this would use more advanced NLP techniques
        answer_lower = generated_answer.lower()
        content_pieces = [chunk['text'].lower() for chunk in retrieved_chunks]

        # Check if the answer contains content from the retrieved chunks
        matching_segments = 0
        total_segments = len(content_pieces)

        for content_piece in content_pieces:
            # Check if there are overlapping phrases
            if any(word in answer_lower for word in content_piece.split()[:10]):  # Check first 10 words
                matching_segments += 1

        grounding_ratio = matching_segments / total_segments if total_segments > 0 else 0.0

        issues = []
        if grounding_ratio < 0.3:  # Less than 30% of content pieces match
            issues.append({
                'type': 'poor_grounding',
                'severity': 'warning',
                'details': f'Answer may not be sufficiently grounded in retrieved content (match ratio: {grounding_ratio:.2f})'
            })

        validation_result = {
            'is_valid': grounding_ratio >= 0.3,
            'grounding_ratio': grounding_ratio,
            'issues_found': issues,
            'confidence_in_grounding': grounding_ratio
        }

        logger.info(f"Grounding validation completed: ratio={grounding_ratio:.2f}, valid={validation_result['is_valid']}")
        return validation_result

    except Exception as e:
        logger.error(f"Failed to validate grounding: {str(e)}")
        raise ValidationError(f"Could not validate grounding: {str(e)}")


if __name__ == "__main__":
    # Test the agent functionality
    try:
        print("Testing agent initialization...")
        client = initialize_openai_client()
        print("OpenAI client initialized successfully")

        print("\nTesting query processing...")
        result = process_query("What is ROS 2?", max_tokens=200)
        print(f"Answer: {result['answer'][:100]}...")
        print(f"Confidence: {result['confidence_score']:.2f}")
        print(f"Processing time: {result['processing_time_ms']:.2f}ms")

        print("\nTesting grounding validation...")
        validation = validate_grounding(result['answer'], result['retrieval_details']['retrieved_chunks'])
        print(f"Is grounded: {validation['is_valid']}")
        print(f"Grounding ratio: {validation['grounding_ratio']:.2f}")

    except Exception as e:
        print(f"Error during testing: {e}")