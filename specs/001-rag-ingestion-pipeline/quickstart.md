# Quickstart: RAG Ingestion Pipeline

## Overview
Quick setup guide to run the RAG ingestion pipeline that extracts, chunks, and embeds content from the deployed book website.

## Prerequisites
- Python 3.11 or higher
- UV package manager installed
- Cohere API key
- Qdrant Cloud API key and URL

## Setup

### 1. Navigate to backend directory
```bash
cd backend
```

### 2. Install dependencies with UV
```bash
uv venv
source .venv/bin/activate  # On Windows: .venv\Scripts\activate
uv pip install -r requirements.txt
```

### 3. Configure environment variables
Ensure your `.env` file contains the required API keys:
```env
COHERE_API_KEY=your_cohere_api_key_here
QDRANT_URL=your_qdrant_cloud_url_here
QDRANT_API_KEY=your_qdrant_api_key_here
```

## Usage

### 1. Run the ingestion pipeline
```bash
python main.py
```

The script will:
1. Discover all URLs from the book website using sitemap.xml
2. Extract text content from each page
3. Chunk the content preserving metadata (module, page, heading, URL)
4. Generate embeddings using Cohere
5. Create "rag_embedding" collection in Qdrant
6. Store embeddings in Qdrant with metadata

### 2. Verify ingestion
Check your Qdrant Cloud dashboard to confirm embeddings have been stored in the `rag_embedding` collection.

## Configuration
The pipeline can be configured by modifying the constants in `main.py`:
- `CHUNK_SIZE`: Size of text chunks in tokens (default: 800)
- `CHUNK_OVERLAP`: Overlap between chunks in tokens (default: 160)
- `RATE_LIMIT`: Requests per second for web scraping (default: 1)

## Troubleshooting

### API Rate Limits
If you encounter rate limit errors:
- The script implements exponential backoff for Cohere API calls
- Check your Cohere and Qdrant usage limits
- Consider upgrading your plan for higher throughput

### Connection Issues
If you have connection issues with Qdrant:
- Verify your QDRANT_URL is correct
- Check that your QDRANT_API_KEY is valid
- Ensure your network allows outbound connections to Qdrant Cloud

### Scraping Issues
If content extraction fails:
- Check that the source website (https://sigma-hackathon-4-specify-plus.vercel.app/) is accessible
- Verify the CSS selectors in the extraction function match the site structure
- Ensure your IP isn't blocked by the hosting service