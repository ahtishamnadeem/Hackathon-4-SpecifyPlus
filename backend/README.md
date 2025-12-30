---
title: RAG Chatbot Backend
emoji: 🤖
colorFrom: purple
colorTo: blue
sdk: docker
python_version: 3.11
---

# RAG Chatbot Backend

This is the backend service for a RAG (Retrieval-Augmented Generation) chatbot that powers educational content about Physical AI and Humanoid Robotics.

## Features
- Vector database integration with Qdrant Cloud
- AI-powered question answering using OpenAI and Google AI Studio
- Content retrieval from book materials
- Session management for conversation history

## Environment Variables Required
This application requires the following environment variables to be set in the Space settings:
- `QDRANT_URL`: Your Qdrant Cloud cluster URL
- `QDRANT_API_KEY`: Your Qdrant API key
- `COHERE_API_KEY`: Your Cohere API key
- `GOOGLE_AI_STUDIO_API_KEY`: Your Google AI Studio API key
- `OPENAI_API_KEY`: Your OpenAI API key
- `COLLECTION_NAME`: Qdrant collection name (default: rag_embedding)

## API Endpoints
- `GET /health`: Health check
- `POST /chat/send`: Send a query to the RAG agent
- `GET /info`: Get agent information

## Architecture
Built with FastAPI, integrates with Qdrant Cloud, Cohere embeddings, and multiple LLM providers.