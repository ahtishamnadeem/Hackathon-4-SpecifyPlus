# RAG Agent Backend

This is the backend service for the RAG (Retrieval-Augmented Generation) chatbot that powers the educational book on Physical AI and Humanoid Robotics.

## Deployment to Railway

This application is designed to be deployed on Railway. Here's how to deploy:

### Prerequisites
- Railway account (sign up at https://railway.app)
- Your API keys for:
  - Qdrant Cloud
  - OpenAI
  - Google AI Studio
  - Cohere

### Deployment Steps

1. **Deploy to Railway**:
   - Go to your Railway dashboard
   - Click "New Project"
   - Select your GitHub repository
   - Choose the `backend` directory

2. **Set Environment Variables**:
   After deployment, go to the "Variables" section in your Railway project and add:

   ```
   QDRANT_URL=your_qdrant_cluster_url
   QDRANT_API_KEY=your_qdrant_api_key
   COHERE_API_KEY=your_cohere_api_key
   GOOGLE_AI_STUDIO_API_KEY=your_google_ai_studio_api_key
   OPENAI_API_KEY=your_openai_api_key
   COLLECTION_NAME=rag_embedding
   MODEL_NAME=gpt-3.5-turbo
   MAX_RETRIEVALS=5
   TEMPERATURE=0.7
   MAX_TOKENS=500
   BACKEND_HOST=0.0.0.0
   BACKEND_PORT=8000
   ```

3. **Configure the Service**:
   - Make sure your service is configured to use the PORT environment variable
   - The Procfile is already configured for Railway deployment

### Environment Variables

The following environment variables are required for the application to run:

- `QDRANT_URL`: Your Qdrant Cloud cluster URL
- `QDRANT_API_KEY`: Your Qdrant API key
- `COHERE_API_KEY`: Your Cohere API key for embeddings
- `GOOGLE_AI_STUDIO_API_KEY`: Your Google AI Studio API key
- `OPENAI_API_KEY`: Your OpenAI API key
- `NEON_DATABASE_URL`: Your Postgres database URL (optional, uses SQLite if empty)

### API Endpoints

- `GET /health` - Health check
- `GET /info` - Agent information
- `POST /chat/send` - Send a query to the RAG agent

### Architecture

The backend uses:
- FastAPI for the web framework
- Qdrant Cloud for vector storage and retrieval
- Cohere for generating embeddings
- Google AI Studio and OpenAI for language model responses
- SQLite for session storage (can be upgraded to Postgres)

## Local Development

For local development:

```bash
pip install -r requirements.txt
python -m uvicorn main:app --host 0.0.0.0 --port 8000
```

## Configuration

The application loads configuration from environment variables using the config.py module. Make sure all required environment variables are set in your Railway deployment.