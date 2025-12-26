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
import google.generativeai as genai
from google.generativeai import GenerativeModel

from config import load_config
from retrieval import retrieve_content
from openai_agents import get_rag_agent
from db_utils import create_session, add_message_to_session, get_session_messages

logger = logging.getLogger(__name__)

# Define custom exceptions
class AgentError(Exception):
    """Raised when agent processing fails"""
    pass

class ValidationError(Exception):
    """Raised when validation parameters are invalid"""
    pass


def process_query(query_text: str, selected_text: Optional[str] = None, max_tokens: Optional[int] = None, temperature: Optional[float] = None, user_id: Optional[str] = None, session_id: Optional[str] = None) -> Dict[str, Any]:
    """
    Process a user query by retrieving relevant content and generating an answer

    Args:
        query_text: The user's query text
        selected_text: Optional selected text to use as context (overrides RAG retrieval)
        max_tokens: Maximum tokens for the response (defaults to config value)
        temperature: Temperature for response generation (defaults to config value)
        user_id: Optional user identifier for memory management
        session_id: Optional session identifier

    Returns:
        Dictionary containing the generated answer, sources, and metadata
    """
    start_time = time.time()

    # Get configuration
    try:
        config = load_config()
        if max_tokens is None:
            max_tokens = config['max_tokens']
        if temperature is None:
            temperature = config['temperature']
    except Exception as e:
        logger.error(f"Failed to load configuration in process_query: {str(e)}")
        execution_time = (time.time() - start_time) * 1000  # Convert to milliseconds
        raise AgentError(f"Configuration error: {str(e)}")

    # Create or use existing session
    if not session_id:
        session = create_session(user_id=user_id, title=query_text[:50] + "..." if len(query_text) > 50 else query_text)
        session_id = session.id
    else:
        session = None  # We'll get the session if needed later

    try:
        # Add user message to session
        add_message_to_session(
            session_id=session_id,
            role='user',
            content=query_text,
            selected_text=selected_text
        )

        # Determine which mode to use based on selected_text
        if selected_text and selected_text.strip():
            # Use selected-text-only mode
            result = _process_selected_text_mode(query_text, selected_text, max_tokens, temperature, user_id, session_id)
        else:
            # Use RAG mode with retrieval
            result = _process_rag_mode(query_text, max_tokens, temperature, user_id, session_id)

        # Add assistant response to session
        add_message_to_session(
            session_id=session_id,
            role='assistant',
            content=result.get('answer', ''),
            selected_text=selected_text,
            context_metadata=result.get('retrieval_details', {})
        )

        # Calculate processing time
        execution_time = (time.time() - start_time) * 1000  # Convert to milliseconds

        # Add processing time to result
        result['processing_time_ms'] = execution_time
        result['session_id'] = session_id

        logger.info(f"Processed query: '{query_text[:50]}...' in {execution_time:.2f}ms, session: {session_id}")
        return result

    except Exception as e:
        logger.error(f"Failed to process query: {str(e)}")
        execution_time = (time.time() - start_time) * 1000  # Convert to milliseconds

        # Add error message to session
        try:
            add_message_to_session(
                session_id=session_id,
                role='system',
                content=f"Error processing query: {str(e)}"
            )
        except:
            pass  # Don't let session logging errors interfere with main error

        raise AgentError(f"Could not process query: {str(e)}")


def _process_selected_text_mode(query_text: str, selected_text: str, max_tokens: Optional[int], temperature: Optional[float], user_id: Optional[str], session_id: Optional[str]) -> Dict[str, Any]:
    """
    Process query in selected-text-only mode (no RAG retrieval)
    Attempts Google AI Studio (Gemini 1.5 Flash) first as primary LLM
    If Gemini fails, returns retrieval-only mode with the selected text
    """
    start_time = time.time()

    config = load_config()

    # First, try Google AI Studio API (Gemini 1.5 Flash) as primary LLM
    try:
        # Initialize Google AI Studio API
        genai.configure(api_key=config['google_ai_studio_api_key'])

        # Use a compatible model from Google AI Studio
        model = GenerativeModel('gemini-2.5-flash')  # Use latest available model

        # Format the prompt for Google AI Studio
        google_prompt = f"""
        You are a helpful assistant that answers questions based ONLY on the provided selected text.
        Do not use any other knowledge or make assumptions beyond what is explicitly stated in the provided text.

        Selected text context:
        {selected_text}

        User question:
        {query_text}

        Please answer the question based ONLY on the information provided in the selected text.
        If the selected text does not contain enough information to answer the question, clearly state that the information is not available in the provided text.
        """

        # Generate response using Google AI Studio
        response = model.generate_content(
            google_prompt,
            generation_config={
                "temperature": temperature,
                "max_output_tokens": max_tokens
            }
        )

        generated_answer = response.text

        # Calculate confidence - higher confidence for selected-text mode since it's based on exact provided context
        confidence_score = 0.9

        execution_time = (time.time() - start_time) * 1000  # Convert to milliseconds

        result = {
            'query_text': query_text,
            'answer': generated_answer,
            'sources': [{
                'type': 'selected_text',
                'content_preview': selected_text[:200] + "..." if len(selected_text) > 200 else selected_text,
                'original_length': len(selected_text),
                'used_as_primary_context': True
            }],
            'confidence_score': confidence_score,
            'retrieval_details': {
                'method': 'selected_text_only',
                'chunks_retrieved': 0,
                'execution_time_ms': 0,
                'collection_name': config['collection_name'],
                'top_k_requested': 0
            },
            'generation_details': {
                'model_used': 'gemini-2.5-flash',  # Google's model
                'provider_used': 'google_ai_studio',
                'tokens_used': 0,  # Google AI Studio may not provide token counts
                'input_tokens': 0,
                'output_tokens': 0,
                'generation_method': 'selected_text_mode'
            },
            'processing_time_ms': execution_time,
            'mode': 'selected_text_only'
        }

        logger.info(f"Processed in selected-text mode with Google AI Studio (Gemini 2.5 Flash): '{query_text[:50]}...' in {execution_time:.2f}ms with confidence {confidence_score:.2f}")
        return result

    except Exception as google_error:
        logger.error(f"Google AI Studio (Gemini 2.5 Flash) failed in selected-text mode: {str(google_error)}")

        # Check if it's a quota/limit error that warrants fallback to retrieval-only mode
        google_error_msg = str(google_error).lower()

        # Check for model-not-found error specifically
        if 'not found' in google_error_msg and 'model' in google_error_msg:
            logger.error(f"Google AI Studio model not found error: {str(google_error)} - This is NOT a quota issue, it's a model/API version mismatch")
            # For model-not-found errors, we should not fall back to retrieval-only mode immediately
            # Instead, we should log this as a configuration error
            raise AgentError(f"Google AI Studio model not found: {str(google_error)}. Please check model name configuration.")
        elif 'quota' in google_error_msg or 'insufficient' in google_error_msg or '429' in google_error_msg or 'rate' in google_error_msg or 'billing' in google_error_msg or 'permission' in google_error_msg:
            logger.info("Google AI Studio quota/limit error detected - using retrieval-only mode")

            # If Gemini fails due to quota/limit, return retrieval-only mode response
            retrieval_only_answer = f"Note: The LLM service is currently unavailable due to quota limits. However, here is the selected text content related to your query '{query_text}':\n\n{selected_text}"

            execution_time = (time.time() - start_time) * 1000  # Convert to milliseconds

            result = {
                'query_text': query_text,
                'answer': retrieval_only_answer,
                'sources': [{
                    'type': 'selected_text',
                    'content_preview': selected_text[:200] + "..." if len(selected_text) > 200 else selected_text,
                    'original_length': len(selected_text),
                    'used_as_primary_context': True
                }],
                'confidence_score': 0.3,  # Lower confidence when LLM is unavailable
                'retrieval_details': {
                    'method': 'selected_text_only_retrieval_only',
                    'chunks_retrieved': 1,  # We have the selected text as the "retrieved" content
                    'execution_time_ms': execution_time,
                    'collection_name': config['collection_name'],
                    'top_k_requested': 0
                },
                'generation_details': {
                    'model_used': 'retrieval_only_mode',
                    'provider_used': 'retrieval_only',
                    'tokens_used': 0,
                    'input_tokens': 0,
                    'output_tokens': 0,
                    'generation_method': 'retrieval_only_mode'
                },
                'processing_time_ms': execution_time,
                'mode': 'selected_text_only_retrieval_only'
            }

            logger.warning("Using retrieval-only mode due to Google AI Studio (Gemini) failure")
            return result
        else:
            # If it's not a quota/limit error, try OpenAI as a secondary fallback
            logger.info("Non-quota Google AI Studio error - attempting OpenAI fallback")
            try:
                # Use OpenAI client as fallback
                client = openai.OpenAI(api_key=config['openai_api_key'])

                # Create prompt that focuses only on the selected text
                prompt = f"""
                You are a helpful assistant that answers questions based ONLY on the provided selected text.
                Do not use any other knowledge or make assumptions beyond what is explicitly stated in the provided text.

                Selected text context:
                {selected_text}

                User question:
                {query_text}

                Please answer the question based ONLY on the information provided in the selected text.
                If the selected text does not contain enough information to answer the question, clearly state that the information is not available in the provided text.

                Answer:
                """

                response = client.chat.completions.create(
                    model=config['model_name'],
                    messages=[
                        {
                            "role": "system",
                            "content": "You are a helpful assistant. Answer the user's question based ONLY on the provided selected text. Do not use any other knowledge or make assumptions beyond what is explicitly stated in the provided text. If the selected text does not contain enough information to answer the question, clearly state that the information is not available in the provided text."
                        },
                        {
                            "role": "user",
                            "content": prompt
                        }
                    ],
                    max_tokens=max_tokens,
                    temperature=temperature
                )

                generated_answer = response.choices[0].message.content
                usage = response.usage

                # Extract token usage
                input_tokens = usage.prompt_tokens if usage else 0
                output_tokens = usage.completion_tokens if usage else 0
                total_tokens = usage.total_tokens if usage else (input_tokens + output_tokens)

                # Calculate confidence - higher confidence for selected-text mode since it's based on exact provided context
                confidence_score = 0.85  # Slightly lower than primary Google AI Studio

                execution_time = (time.time() - start_time) * 1000  # Convert to milliseconds

                result = {
                    'query_text': query_text,
                    'answer': generated_answer,
                    'sources': [{
                        'type': 'selected_text',
                        'content_preview': selected_text[:200] + "..." if len(selected_text) > 200 else selected_text,
                        'original_length': len(selected_text),
                        'used_as_primary_context': True
                    }],
                    'confidence_score': confidence_score,
                    'retrieval_details': {
                        'method': 'selected_text_only_fallback',
                        'chunks_retrieved': 0,
                        'execution_time_ms': 0,
                        'collection_name': config['collection_name'],
                        'top_k_requested': 0
                    },
                    'generation_details': {
                        'model_used': config['model_name'],
                        'provider_used': 'openai',
                        'tokens_used': total_tokens,
                        'input_tokens': input_tokens,
                        'output_tokens': output_tokens,
                        'generation_method': 'selected_text_mode_fallback'
                    },
                    'processing_time_ms': execution_time,
                    'mode': 'selected_text_only_fallback'
                }

                logger.info(f"Processed in selected-text mode with OpenAI fallback: '{query_text[:50]}...' in {execution_time:.2f}ms with confidence {confidence_score:.2f}")
                return result

            except Exception as openai_error:
                logger.error(f"OpenAI fallback also failed: {str(openai_error)}")
                execution_time = (time.time() - start_time) * 1000  # Convert to milliseconds

                # If both APIs fail, return retrieval-only mode response
                retrieval_only_answer = f"Note: The LLM service is currently unavailable due to quota limits. However, here is the selected text content related to your query '{query_text}':\n\n{selected_text}"

                result = {
                    'query_text': query_text,
                    'answer': retrieval_only_answer,
                    'sources': [{
                        'type': 'selected_text',
                        'content_preview': selected_text[:200] + "..." if len(selected_text) > 200 else selected_text,
                        'original_length': len(selected_text),
                        'used_as_primary_context': True
                    }],
                    'confidence_score': 0.2,  # Lower confidence when both APIs fail
                    'retrieval_details': {
                        'method': 'selected_text_only_retrieval_only',
                        'chunks_retrieved': 1,  # We have the selected text as the "retrieved" content
                        'execution_time_ms': execution_time,
                        'collection_name': config['collection_name'],
                        'top_k_requested': 0
                    },
                    'generation_details': {
                        'model_used': 'retrieval_only_mode',
                        'provider_used': 'retrieval_only',
                        'tokens_used': 0,
                        'input_tokens': 0,
                        'output_tokens': 0,
                        'generation_method': 'retrieval_only_mode'
                    },
                    'processing_time_ms': execution_time,
                    'mode': 'selected_text_only_retrieval_only'
                }

                logger.warning("Using retrieval-only mode due to both Google AI Studio and OpenAI failures")
                return result


def _process_rag_mode(query_text: str, max_tokens: Optional[int], temperature: Optional[float], user_id: Optional[str], session_id: Optional[str]) -> Dict[str, Any]:
    """
    Process query in RAG mode with content retrieval from Qdrant
    Implements answer routing logic:
    - If retrieval returns relevant content (confidence above threshold) → Book-grounded mode
    - If retrieval returns no content or low confidence → General knowledge mode
    """
    start_time = time.time()

    config = load_config()

    # Step 1: Retrieve relevant content from Qdrant (always executes first)
    retrieval_result = retrieve_content(
        query_text=query_text,
        collection_name=config['collection_name'],
        top_k=config['max_retrievals']
    )

    retrieved_chunks = retrieval_result['retrieved_chunks']

    # Calculate initial confidence score based on similarity scores
    confidence_score = 0.0
    if retrieved_chunks:
        avg_similarity = sum([chunk['similarity_score'] for chunk in retrieved_chunks]) / len(retrieved_chunks)
        confidence_score = avg_similarity

    # Determine if we have sufficient book content for book-grounded answers
    # Using a threshold of 0.3 for similarity score
    threshold = 0.3
    has_sufficient_content = len(retrieved_chunks) > 0 and confidence_score >= threshold

    if has_sufficient_content:
        # BOOK-GROUNDED MODE: Use retrieved content to generate answer
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

        # Step 3: Generate response using Google AI Studio (Gemini 1.5 Flash) as primary LLM
        try:
            # Initialize Google AI Studio API
            genai.configure(api_key=config['google_ai_studio_api_key'])

            # Use a compatible model from Google AI Studio
            model = GenerativeModel('gemini-2.5-flash')  # Use latest available model

            # Format the prompt for Google AI Studio - FOR BOOK-GROUNDED ANSWERS
            google_prompt = f"""
            You are a helpful assistant that answers questions based ONLY on the provided context from the book.
            Answer the user's question using only the information provided in the context.
            Do NOT use any external knowledge or general robotics knowledge.
            If the context doesn't contain enough information to answer the question, say so.
            Start your answer with: --- According to this book: ---

            Context:
            {context}

            Question: {query_text}

            Answer:
            """

            # Generate response using Google AI Studio
            response = model.generate_content(
                google_prompt,
                generation_config={
                    "temperature": temperature,
                    "max_output_tokens": max_tokens
                }
            )

            generated_answer = response.text

            # Update confidence score after generation
            if retrieved_chunks:
                avg_similarity = sum([chunk['similarity_score'] for chunk in retrieved_chunks]) / len(retrieved_chunks)
                confidence_score = avg_similarity

            execution_time = (time.time() - start_time) * 1000  # Convert to milliseconds

            result = {
                'query_text': query_text,
                'answer': generated_answer,
                'sources': sources,
                'confidence_score': confidence_score,
                'retrieval_details': retrieval_result,
                'generation_details': {
                    'model_used': 'gemini-2.5-flash',  # Google's model
                    'provider_used': 'google_ai_studio',
                    'tokens_used': 0,  # Google AI Studio may not provide token counts
                    'input_tokens': 0,
                    'output_tokens': 0,
                    'generation_method': 'rag_retrieval_book_grounded'
                },
                'processing_time_ms': execution_time,
                'mode': 'rag_retrieval_book_grounded'
            }

            logger.info(f"Processed in RAG book-grounded mode with Google AI Studio (Gemini 2.5 Flash): '{query_text[:50]}...' in {execution_time:.2f}ms with confidence {confidence_score:.2f}")
            return result

        except Exception as google_error:
            logger.error(f"Google AI Studio (Gemini 2.5 Flash) failed in RAG book-grounded mode: {str(google_error)}")

            # Check if it's a quota/limit error that warrants fallback to retrieval-only mode
            google_error_msg = str(google_error).lower()

            # Check for model-not-found error specifically
            if 'not found' in google_error_msg and 'model' in google_error_msg:
                logger.error(f"Google AI Studio model not found error: {str(google_error)} - This is NOT a quota issue, it's a model/API version mismatch")
                # For model-not-found errors, we should not fall back to retrieval-only mode immediately
                # Instead, we should log this as a configuration error
                raise AgentError(f"Google AI Studio model not found: {str(google_error)}. Please check model name configuration.")
            elif 'quota' in google_error_msg or 'insufficient' in google_error_msg or '429' in google_error_msg or 'rate' in google_error_msg or 'billing' in google_error_msg or 'permission' in google_error_msg:
                logger.info("Google AI Studio quota/limit error detected in RAG mode - activating retrieval-only mode")

                # ACTIVATE RETRIEVAL-ONLY MODE: Return the retrieved chunks with notification about LLM unavailability
                retrieved_chunks_formatted = []
                for chunk in retrieved_chunks:
                    retrieved_chunks_formatted.append({
                        'text': chunk['text'][:500] + "..." if len(chunk['text']) > 500 else chunk['text'],
                        'metadata': chunk['metadata'],
                        'similarity_score': chunk['similarity_score'],
                        'source_url': chunk['metadata'].get('url', ''),
                        'source_page': chunk['metadata'].get('page', ''),
                        'source_heading': chunk['metadata'].get('heading', '')
                    })

                retrieved_chunks_text = "\n\n".join([f"Source: {chunk['source_url']} | Page: {chunk['source_page']} | Heading: {chunk['source_heading']} | Score: {chunk['similarity_score']:.3f}\nContent: {chunk['text']}\n" for chunk in retrieved_chunks_formatted])

                retrieval_only_answer = f"""--- Retrieval-Only Mode Activated ---
LLM Service Unavailable Due to Quota Limits
However, here are the most relevant chunks from the knowledge base related to your query '{query_text}':

{retrieved_chunks_text}

Note: The LLM service is currently unavailable due to quota limits. These are raw chunks from the knowledge base."""

                execution_time = (time.time() - start_time) * 1000  # Convert to milliseconds

                result = {
                    'query_text': query_text,
                    'answer': retrieval_only_answer,
                    'sources': retrieved_chunks_formatted,  # Return the actual retrieved chunks
                    'confidence_score': 0.3,  # Lower confidence when LLM is unavailable
                    'retrieval_details': retrieval_result,
                    'generation_details': {
                        'model_used': 'retrieval_only_mode',
                        'provider_used': 'retrieval_only',
                        'tokens_used': 0,
                        'input_tokens': 0,
                        'output_tokens': 0,
                        'generation_method': 'retrieval_only_mode'
                    },
                    'processing_time_ms': execution_time,
                    'mode': 'rag_retrieval_only'
                }

                logger.warning("Using retrieval-only mode in RAG due to Google AI Studio (Gemini) quota failure")
                return result
            else:
                # If it's not a quota/limit error, try OpenAI as a secondary fallback
                logger.info("Non-quota Google AI Studio error in RAG mode - attempting OpenAI fallback")
                try:
                    # Use OpenAI client as fallback
                    client = openai.OpenAI(api_key=config['openai_api_key'])

                    # Prepare context from retrieved chunks
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

                    # Format the prompt with context - FOR BOOK-GROUNDED ANSWERS WHEN POSSIBLE
                    prompt = f"""
                    You are a helpful assistant that answers questions based ONLY on the provided context from the book.
                    Answer the user's question using only the information provided in the context.
                    Do NOT use any external knowledge or general robotics knowledge.
                    If the context doesn't contain enough information to answer the question, say so.
                    Start your answer with: --- According to this book: ---

                    Context:
                    {context}

                    Question: {query_text}

                    Answer:
                    """

                    response = client.chat.completions.create(
                        model=config['model_name'],
                        messages=[
                            {"role": "system", "content": "You are a helpful assistant that answers questions based ONLY on the provided context from the book. Answer the user's question using only the information provided in the context. Do NOT use any external knowledge or general robotics knowledge. If the context doesn't contain enough information to answer the question, say so. Start your answer with: --- According to this book: ---"},
                            {"role": "user", "content": prompt}
                        ],
                        max_tokens=max_tokens,
                        temperature=temperature
                    )

                    generated_answer = response.choices[0].message.content
                    usage = response.usage

                    # Extract token usage
                    input_tokens = usage.prompt_tokens if usage else 0
                    output_tokens = usage.completion_tokens if usage else 0
                    total_tokens = usage.total_tokens if usage else (input_tokens + output_tokens)

                    # Calculate confidence score based on similarity scores
                    confidence_score = 0.0
                    if retrieved_chunks:
                        avg_similarity = sum([chunk['similarity_score'] for chunk in retrieved_chunks]) / len(retrieved_chunks)
                        confidence_score = avg_similarity * 0.8  # Slightly reduce confidence for fallback

                    execution_time = (time.time() - start_time) * 1000  # Convert to milliseconds

                    result = {
                        'query_text': query_text,
                        'answer': generated_answer,
                        'sources': sources,
                        'confidence_score': confidence_score,
                        'retrieval_details': retrieval_result,
                        'generation_details': {
                            'model_used': config['model_name'],
                            'provider_used': 'openai',
                            'tokens_used': total_tokens,
                            'input_tokens': input_tokens,
                            'output_tokens': output_tokens,
                            'generation_method': 'rag_retrieval_book_grounded_fallback'
                        },
                        'processing_time_ms': execution_time,
                        'mode': 'rag_retrieval_book_grounded_fallback'
                    }

                    logger.info(f"Processed in RAG book-grounded mode with OpenAI fallback: '{query_text[:50]}...' in {execution_time:.2f}ms with confidence {confidence_score:.2f}")
                    return result

                except Exception as openai_error:
                    logger.error(f"OpenAI fallback also failed in RAG mode: {str(openai_error)}")

                    # If both APIs fail, activate RETRIEVAL-ONLY MODE
                    retrieved_chunks_formatted = []
                    for chunk in retrieved_chunks:
                        retrieved_chunks_formatted.append({
                            'text': chunk['text'][:500] + "..." if len(chunk['text']) > 500 else chunk['text'],
                            'metadata': chunk['metadata'],
                            'similarity_score': chunk['similarity_score'],
                            'source_url': chunk['metadata'].get('url', ''),
                            'source_page': chunk['metadata'].get('page', ''),
                            'source_heading': chunk['metadata'].get('heading', '')
                        })

                    retrieval_only_answer = f"""
--- Retrieval-Only Mode Activated ---
Both LLM Services Unavailable Due to Quota Limits
However, here are the most relevant chunks from the knowledge base related to your query '{query_text}':

{chr(10).join([f"Source: {chunk['source_url']} | Page: {chunk['source_page']} | Heading: {chunk['source_heading']} | Score: {chunk['similarity_score']:.3f}\nContent: {chunk['text']}{chr(10)}" for chunk in retrieved_chunks_formatted])}

Note: Both LLM services are currently unavailable due to quota limits. These are raw chunks from the knowledge base.
"""

                    execution_time = (time.time() - start_time) * 1000  # Convert to milliseconds

                    result = {
                        'query_text': query_text,
                        'answer': retrieval_only_answer,
                        'sources': retrieved_chunks_formatted,  # Return the actual retrieved chunks
                        'confidence_score': 0.2,  # Lower confidence when both APIs fail
                        'retrieval_details': retrieval_result,
                        'generation_details': {
                            'model_used': 'retrieval_only_mode',
                            'provider_used': 'retrieval_only',
                            'tokens_used': 0,
                            'input_tokens': 0,
                            'output_tokens': 0,
                            'generation_method': 'retrieval_only_mode'
                        },
                        'processing_time_ms': execution_time,
                        'mode': 'rag_retrieval_only'
                    }

                    logger.warning("Using retrieval-only mode in RAG due to both Google AI Studio and OpenAI failures")
                    return result
    else:
        # GENERAL KNOWLEDGE MODE: No sufficient content found, use general knowledge
        try:
            # Initialize Google AI Studio API for general knowledge
            genai.configure(api_key=config['google_ai_studio_api_key'])

            # Use a compatible model from Google AI Studio
            model = GenerativeModel('gemini-2.5-flash')  # Use latest available model

            # Format the prompt for Google AI Studio - FOR GENERAL KNOWLEDGE ANSWERS
            google_prompt = f"""
            You are a helpful assistant that answers questions using your general robotics knowledge.
            Do NOT reference or cite the book or any specific content.
            Answer the question based on your general knowledge of robotics, AI, and related topics.
            Start your answer with: --- Based on general robotics knowledge (not from the book): ---

            Question: {query_text}

            Answer:
            """

            # Generate response using Google AI Studio
            response = model.generate_content(
                google_prompt,
                generation_config={
                    "temperature": temperature,
                    "max_output_tokens": max_tokens
                }
            )

            generated_answer = response.text

            # For general knowledge mode, confidence is based on retrieval results
            confidence_score = 0.1  # Low confidence since no specific book content was used

            execution_time = (time.time() - start_time) * 1000  # Convert to milliseconds

            result = {
                'query_text': query_text,
                'answer': generated_answer,
                'sources': [],  # No sources since it's general knowledge
                'confidence_score': confidence_score,
                'retrieval_details': retrieval_result,
                'generation_details': {
                    'model_used': 'gemini-2.5-flash',  # Google's model
                    'provider_used': 'google_ai_studio_general',
                    'tokens_used': 0,  # Google AI Studio may not provide token counts
                    'input_tokens': 0,
                    'output_tokens': 0,
                    'generation_method': 'rag_general_knowledge'
                },
                'processing_time_ms': execution_time,
                'mode': 'rag_general_knowledge'
            }

            logger.info(f"Processed in RAG general knowledge mode with Google AI Studio (Gemini 2.5 Flash): '{query_text[:50]}...' in {execution_time:.2f}ms with confidence {confidence_score:.2f}")
            return result

        except Exception as google_error:
            logger.error(f"Google AI Studio (Gemini 2.5 Flash) failed in general knowledge mode: {str(google_error)}")

            # Check if it's a quota/limit error that warrants fallback to OpenAI
            google_error_msg = str(google_error).lower()

            # Check for model-not-found error specifically
            if 'not found' in google_error_msg and 'model' in google_error_msg:
                logger.error(f"Google AI Studio model not found error: {str(google_error)} - This is NOT a quota issue, it's a model/API version mismatch")
                # For model-not-found errors, we should not fall back immediately
                # Instead, we should log this as a configuration error
                raise AgentError(f"Google AI Studio model not found: {str(google_error)}. Please check model name configuration.")
            elif 'quota' in google_error_msg or 'insufficient' in google_error_msg or '429' in google_error_msg or 'rate' in google_error_msg or 'billing' in google_error_msg or 'permission' in google_error_msg:
                logger.info("Google AI Studio quota/limit error detected in general knowledge mode - using OpenAI fallback")

                # Try OpenAI as fallback for general knowledge
                try:
                    client = openai.OpenAI(api_key=config['openai_api_key'])

                    # Format the prompt for general knowledge
                    prompt = f"""
                    You are a helpful assistant that answers questions using your general robotics knowledge.
                    Do NOT reference or cite the book or any specific content.
                    Answer the question based on your general knowledge of robotics, AI, and related topics.
                    Start your answer with: --- Based on general robotics knowledge (not from the book): ---

                    Question: {query_text}

                    Answer:
                    """

                    response = client.chat.completions.create(
                        model=config['model_name'],
                        messages=[
                            {"role": "system", "content": "You are a helpful assistant that answers questions using your general robotics knowledge. Do NOT reference or cite the book or any specific content. Answer the question based on your general knowledge of robotics, AI, and related topics."},
                            {"role": "user", "content": prompt}
                        ],
                        max_tokens=max_tokens,
                        temperature=temperature
                    )

                    generated_answer = response.choices[0].message.content
                    usage = response.usage

                    # Extract token usage
                    input_tokens = usage.prompt_tokens if usage else 0
                    output_tokens = usage.completion_tokens if usage else 0
                    total_tokens = usage.total_tokens if usage else (input_tokens + output_tokens)

                    # For general knowledge mode, confidence is low
                    confidence_score = 0.1  # Low confidence since no specific book content was used

                    execution_time = (time.time() - start_time) * 1000  # Convert to milliseconds

                    result = {
                        'query_text': query_text,
                        'answer': generated_answer,
                        'sources': [],  # No sources since it's general knowledge
                        'confidence_score': confidence_score,
                        'retrieval_details': retrieval_result,
                        'generation_details': {
                            'model_used': config['model_name'],
                            'provider_used': 'openai_general',
                            'tokens_used': total_tokens,
                            'input_tokens': input_tokens,
                            'output_tokens': output_tokens,
                            'generation_method': 'rag_general_knowledge_fallback'
                        },
                        'processing_time_ms': execution_time,
                        'mode': 'rag_general_knowledge_fallback'
                    }

                    logger.info(f"Processed in RAG general knowledge mode with OpenAI fallback: '{query_text[:50]}...' in {execution_time:.2f}ms with confidence {confidence_score:.2f}")
                    return result

                except Exception as openai_error:
                    logger.error(f"OpenAI fallback also failed in general knowledge mode: {str(openai_error)}")

                    # If both APIs fail in general knowledge mode, return a simple response
                    fallback_answer = f"--- Based on general robotics knowledge (not from the book): ---\n\nI don't have specific information about '{query_text}' in the book content, and I'm unable to access the LLM service for general knowledge. Please check other resources or try rephrasing your question."

                    execution_time = (time.time() - start_time) * 1000  # Convert to milliseconds

                    result = {
                        'query_text': query_text,
                        'answer': fallback_answer,
                        'sources': [],  # No sources since it's general knowledge
                        'confidence_score': 0.05,  # Very low confidence
                        'retrieval_details': retrieval_result,
                        'generation_details': {
                            'model_used': 'fallback_response',
                            'provider_used': 'fallback',
                            'tokens_used': 0,
                            'input_tokens': 0,
                            'output_tokens': 0,
                            'generation_method': 'rag_general_knowledge_fallback_response'
                        },
                        'processing_time_ms': execution_time,
                        'mode': 'rag_general_knowledge_fallback_response'
                    }

                    logger.warning("Using fallback response in general knowledge mode due to both Google AI Studio and OpenAI failures")
                    return result
            else:
                # If it's not a quota/limit error, try OpenAI as a secondary fallback
                logger.info("Non-quota Google AI Studio error in general knowledge mode - attempting OpenAI fallback")
                try:
                    client = openai.OpenAI(api_key=config['openai_api_key'])

                    # Format the prompt for general knowledge
                    prompt = f"""
                    You are a helpful assistant that answers questions using your general robotics knowledge.
                    Do NOT reference or cite the book or any specific content.
                    Answer the question based on your general knowledge of robotics, AI, and related topics.
                    Start your answer with: --- Based on general robotics knowledge (not from the book): ---

                    Question: {query_text}

                    Answer:
                    """

                    response = client.chat.completions.create(
                        model=config['model_name'],
                        messages=[
                            {"role": "system", "content": "You are a helpful assistant that answers questions using your general robotics knowledge. Do NOT reference or cite the book or any specific content. Answer the question based on your general knowledge of robotics, AI, and related topics."},
                            {"role": "user", "content": prompt}
                        ],
                        max_tokens=max_tokens,
                        temperature=temperature
                    )

                    generated_answer = response.choices[0].message.content
                    usage = response.usage

                    # Extract token usage
                    input_tokens = usage.prompt_tokens if usage else 0
                    output_tokens = usage.completion_tokens if usage else 0
                    total_tokens = usage.total_tokens if usage else (input_tokens + output_tokens)

                    # For general knowledge mode, confidence is low
                    confidence_score = 0.1  # Low confidence since no specific book content was used

                    execution_time = (time.time() - start_time) * 1000  # Convert to milliseconds

                    result = {
                        'query_text': query_text,
                        'answer': generated_answer,
                        'sources': [],  # No sources since it's general knowledge
                        'confidence_score': confidence_score,
                        'retrieval_details': retrieval_result,
                        'generation_details': {
                            'model_used': config['model_name'],
                            'provider_used': 'openai_general',
                            'tokens_used': total_tokens,
                            'input_tokens': input_tokens,
                            'output_tokens': output_tokens,
                            'generation_method': 'rag_general_knowledge_fallback'
                        },
                        'processing_time_ms': execution_time,
                        'mode': 'rag_general_knowledge_fallback'
                    }

                    logger.info(f"Processed in RAG general knowledge mode with OpenAI fallback: '{query_text[:50]}...' in {execution_time:.2f}ms with confidence {confidence_score:.2f}")
                    return result

                except Exception as openai_error:
                    logger.error(f"OpenAI fallback also failed in general knowledge mode: {str(openai_error)}")

                    # If both APIs fail in general knowledge mode, return a simple response
                    fallback_answer = f"--- Based on general robotics knowledge (not from the book): ---\n\nI don't have specific information about '{query_text}' in the book content, and I'm unable to access the LLM service for general knowledge. Please check other resources or try rephrasing your question."

                    execution_time = (time.time() - start_time) * 1000  # Convert to milliseconds

                    result = {
                        'query_text': query_text,
                        'answer': fallback_answer,
                        'sources': [],  # No sources since it's general knowledge
                        'confidence_score': 0.05,  # Very low confidence
                        'retrieval_details': retrieval_result,
                        'generation_details': {
                            'model_used': 'fallback_response',
                            'provider_used': 'fallback',
                            'tokens_used': 0,
                            'input_tokens': 0,
                            'output_tokens': 0,
                            'generation_method': 'rag_general_knowledge_fallback_response'
                        },
                        'processing_time_ms': execution_time,
                        'mode': 'rag_general_knowledge_fallback_response'
                    }

                    logger.warning("Using fallback response in general knowledge mode due to both Google AI Studio and OpenAI failures")
                    return result


def initialize_openai_client():
    """
    Initialize OpenAI client for health check
    """
    config = load_config()
    client = openai.OpenAI(api_key=config['openai_api_key'])
    return client


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
        print("Testing query processing...")
        result = process_query("What is ROS 2?", max_tokens=200)
        print(f"Answer: {result['answer'][:100]}...")
        print(f"Confidence: {result['confidence_score']:.2f}")
        print(f"Processing time: {result['processing_time_ms']:.2f}ms")
        print(f"Mode: {result['mode']}")

        print("\nTesting selected-text mode...")
        sample_text = "ROS 2 is a flexible framework for writing robot software. It is a set of libraries and tools that help developers create robot applications."
        result2 = process_query("What is ROS 2?", selected_text=sample_text, max_tokens=200)
        print(f"Answer: {result2['answer'][:100]}...")
        print(f"Mode: {result2['mode']}")

    except Exception as e:
        print(f"Error during testing: {e}")