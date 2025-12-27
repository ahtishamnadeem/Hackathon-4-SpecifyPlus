"""
Simplified OpenAI Agents Implementation for RAG Chatbot

A simplified implementation that focuses on the core functionality without complex langchain dependencies.
"""
import os
import logging
import time
from typing import Dict, Any, List, Optional
from datetime import datetime
import json

from openai import OpenAI

from config import load_config
from retrieval import retrieve_content

logger = logging.getLogger(__name__)


class SimpleRAGAgent:
    def __init__(self):
        self.config = load_config()
        self.client = OpenAI(api_key=self.config['openai_api_key'])

    def _process_selected_text_mode(self, query: str, selected_text: str, user_id: Optional[str] = None) -> Dict[str, Any]:
        """
        Process query in selected-text-only mode
        """
        try:
            # Create prompt that focuses only on the selected text
            prompt = f"""
            You are a helpful assistant that answers questions based ONLY on the provided selected text.
            Do not use any other knowledge or make assumptions beyond what is explicitly stated in the provided text.

            Selected text context:
            {selected_text}

            User question:
            {query}

            Please answer the question based ONLY on the information provided in the selected text.
            If the selected text does not contain enough information to answer the question, clearly state that the information is not available in the provided text.

            Answer:
            """

            response = self.client.chat.completions.create(
                model=self.config['model_name'],
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
                max_tokens=self.config['max_tokens'],
                temperature=self.config['temperature']
            )

            generated_answer = response.choices[0].message.content

            return {
                'answer': generated_answer,
                'sources': [{'type': 'selected_text', 'content': selected_text[:200] + "..."}],
                'confidence': 0.9,
                'mode': 'selected_text_only'
            }

        except Exception as e:
            logger.error(f"Error in selected text mode: {str(e)}")
            return {
                'answer': "There was an issue processing your query based on the selected text. Please try reselecting the text or ask a general question.",
                'sources': [],
                'confidence': 0.0,
                'mode': 'selected_text_error'
            }

    def _process_rag_mode(self, query: str, user_id: Optional[str] = None) -> Dict[str, Any]:
        """
        Process query in RAG mode with content retrieval
        """
        try:
            # Retrieve relevant content from Qdrant
            retrieval_result = retrieve_content(
                query_text=query,
                collection_name=self.config['collection_name'],
                top_k=self.config['max_retrievals']
            )

            retrieved_chunks = retrieval_result['retrieved_chunks']

            # Prepare context for the language model
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

            # Generate response using OpenAI
            prompt = f"""
            You are a helpful assistant that answers questions based on the provided context.
            Answer the user's question using only the information provided in the context.
            If the context doesn't contain enough information to answer the question, say so.

            Context:
            {context}

            Question: {query}

            Answer:
            """

            response = self.client.chat.completions.create(
                model=self.config['model_name'],
                messages=[
                    {"role": "system", "content": "You are a helpful assistant that answers questions based on provided context. Only use information from the context to answer. If the context doesn't contain the information needed, say so."},
                    {"role": "user", "content": prompt}
                ],
                max_tokens=self.config['max_tokens'],
                temperature=self.config['temperature']
            )

            generated_answer = response.choices[0].message.content

            # Calculate confidence score based on similarity scores
            confidence = 0.0
            if retrieved_chunks:
                avg_similarity = sum([chunk['similarity_score'] for chunk in retrieved_chunks]) / len(retrieved_chunks)
                confidence = avg_similarity

            return {
                'answer': generated_answer,
                'sources': sources,
                'confidence': confidence,
                'mode': 'rag_retrieval'
            }

        except Exception as e:
            logger.error(f"Error in RAG mode: {str(e)}")
            return {
                'answer': "There was an issue retrieving information to answer your question. Please try rephrasing your query.",
                'sources': [],
                'confidence': 0.0,
                'mode': 'rag_error'
            }


# For backward compatibility
_simple_agent = None


def get_rag_agent():
    """
    Get or create the RAG agent instance
    """
    global _simple_agent
    if _simple_agent is None:
        _simple_agent = SimpleRAGAgent()
    return _simple_agent