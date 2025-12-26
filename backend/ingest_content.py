#!/usr/bin/env python3
"""
Content ingestion script for the RAG Agent

This script loads all Docusaurus book content (markdown files), cleans and chunks the content,
generates embeddings using Cohere, and stores them in Qdrant for RAG retrieval.
"""

import os
import re
import logging
import time
import hashlib
import uuid
from pathlib import Path
from typing import List, Dict, Any, Optional, Tuple
from qdrant_client import QdrantClient
from qdrant_client.http import models
from qdrant_client.models import VectorParams, Distance
import cohere
import markdown
from bs4 import BeautifulSoup

from config import load_config

logger = logging.getLogger(__name__)

def clean_markdown_content(content: str) -> str:
    """
    Clean markdown content by removing frontmatter and other noise.
    """
    # Remove YAML frontmatter if present
    if content.startswith('---'):
        try:
            end_frontmatter = content.find('---', 3)
            if end_frontmatter != -1:
                content = content[end_frontmatter + 3:].strip()
        except:
            pass

    # Remove any other common metadata patterns
    content = re.sub(r'^title:.*\n', '', content, flags=re.MULTILINE)
    content = re.sub(r'^sidebar_label:.*\n', '', content, flags=re.MULTILINE)
    content = re.sub(r'^description:.*\n', '', content, flags=re.MULTILINE)
    content = re.sub(r'^keywords:.*\n', '', content, flags=re.MULTILINE)

    return content.strip()

def extract_metadata_from_path(file_path: Path) -> Dict[str, str]:
    """
    Extract metadata from the file path.
    """
    parts = file_path.parts
    # Find the 'docs' directory and extract module info
    docs_index = -1
    for i, part in enumerate(parts):
        if part == 'docs':
            docs_index = i
            break

    metadata = {
        'url': f"/docs/{'/'.join(parts[docs_index+1:])}",
        'module': 'unknown',
        'page': file_path.stem,
        'heading': '',
        'title': file_path.stem.replace('-', ' ').title()
    }

    # Extract module from path if available
    if docs_index != -1 and len(parts) > docs_index + 1:
        for part in parts[docs_index+1:]:
            if part.startswith('module-'):
                metadata['module'] = part
                break

    return metadata

def chunk_content(content: str, max_chunk_size: int = 1000, overlap: int = 100) -> List[Dict[str, Any]]:
    """
    Split content into overlapping chunks for better semantic retrieval.
    """
    # Convert markdown to plain text to get actual content length
    html = markdown.markdown(content)
    soup = BeautifulSoup(html, 'html.parser')
    plain_text = soup.get_text()

    # Split content into sentences
    sentences = re.split(r'[.!?]+\s+', plain_text)

    chunks = []
    current_chunk = ""
    current_size = 0

    for sentence in sentences:
        sentence_size = len(sentence)

        # If adding this sentence would exceed the chunk size
        if current_size + sentence_size > max_chunk_size and current_chunk:
            # Save the current chunk
            chunks.append({
                'text': current_chunk.strip(),
                'size': current_size
            })

            # Start a new chunk with overlap
            if len(current_chunk) > overlap:
                # Use the last 'overlap' characters from the current chunk as starting point
                overlap_text = current_chunk[-overlap:]
                current_chunk = overlap_text + " " + sentence
                current_size = len(current_chunk)
            else:
                current_chunk = sentence
                current_size = sentence_size
        else:
            # Add sentence to current chunk
            if current_chunk:
                current_chunk += " " + sentence
            else:
                current_chunk = sentence
            current_size += sentence_size + 1  # +1 for the space

    # Add the last chunk if it has content
    if current_chunk.strip():
        chunks.append({
            'text': current_chunk.strip(),
            'size': current_size
        })

    # Also handle cases where individual chunks are still too large
    final_chunks = []
    for chunk in chunks:
        if len(chunk['text']) > max_chunk_size:
            # Split large chunks further using character-based splitting
            text = chunk['text']
            while len(text) > max_chunk_size:
                # Find a good breaking point (sentence or paragraph boundary)
                split_point = max_chunk_size
                for separator in ['\n\n', '. ', '! ', '? ', '; ', ': ']:
                    last_sep = text.rfind(separator, 0, max_chunk_size)
                    if last_sep != -1:
                        split_point = last_sep + len(separator)
                        break

                sub_chunk = text[:split_point].strip()
                if sub_chunk:
                    final_chunks.append({
                        'text': sub_chunk,
                        'size': len(sub_chunk)
                    })

                text = text[split_point:].strip()

            if text:
                final_chunks.append({
                    'text': text,
                    'size': len(text)
                })
        else:
            final_chunks.append(chunk)

    return final_chunks

def get_embedding(text: str, cohere_client: cohere.Client) -> List[float]:
    """
    Generate embedding for text using Cohere with rate limiting.
    """
    try:
        response = cohere_client.embed(
            texts=[text],
            model='embed-english-v3.0',
            input_type='search_document'  # Using search_document for content chunks
        )
        return response.embeddings[0]
    except Exception as e:
        logger.error(f"Error generating embedding: {str(e)}")
        # If it's a rate limit error, wait a bit before retrying
        if "429" in str(e) or "Too Many Requests" in str(e) or "rate limit" in str(e).lower():
            logger.info("Rate limit hit, waiting 60 seconds before retrying...")
            time.sleep(60)
            # Retry once after waiting
            try:
                response = cohere_client.embed(
                    texts=[text],
                    model='embed-english-v3.0',
                    input_type='search_document'
                )
                return response.embeddings[0]
            except Exception as retry_error:
                logger.error(f"Retry failed after rate limit: {str(retry_error)}")
                raise
        else:
            raise

def generate_document_id(text: str, metadata: Dict[str, str]) -> str:
    """
    Generate a unique document ID using integer ID to ensure proper format for Qdrant.
    """
    # Create a hash-based ID that can be converted to integer for Qdrant compatibility
    content_hash = hashlib.md5((text + metadata.get('url', '')).encode('utf-8')).hexdigest()
    # Convert hex to int and use modulo to keep it within reasonable range
    return abs(int(content_hash[:16], 16)) % (2**63 - 1)  # Max value for signed 64-bit integer

def load_and_process_markdown_files(docs_dir: str) -> List[Dict[str, Any]]:
    """
    Load and process all markdown files from the docs directory.
    """
    docs_path = Path(docs_dir)
    all_chunks = []

    # Find all markdown files in the docs directory and subdirectories
    md_files = list(docs_path.rglob("*.md")) + list(docs_path.rglob("*.mdx"))

    logger.info(f"Found {len(md_files)} markdown files to process")

    for file_path in md_files:
        try:
            logger.info(f"Processing file: {file_path}")

            # Read the markdown content
            with open(file_path, 'r', encoding='utf-8') as f:
                content = f.read()

            # Clean the content
            cleaned_content = clean_markdown_content(content)

            # Extract metadata
            metadata = extract_metadata_from_path(file_path)

            # Chunk the content
            chunks = chunk_content(cleaned_content)

            # Add metadata to each chunk
            for i, chunk in enumerate(chunks):
                chunk_data = {
                    'text': chunk['text'],
                    'metadata': {
                        **metadata,
                        'chunk_index': i,
                        'total_chunks': len(chunks),
                        'file_path': str(file_path)
                    }
                }
                all_chunks.append(chunk_data)

            logger.info(f"Processed {len(chunks)} chunks from {file_path}")

        except Exception as e:
            logger.error(f"Error processing file {file_path}: {str(e)}")
            continue

    logger.info(f"Total chunks generated: {len(all_chunks)}")
    return all_chunks

def ingest_content_to_qdrant(chunks: List[Dict[str, Any]], collection_name: str = "rag_embedding"):
    """
    Ingest processed content chunks to Qdrant.
    """
    config = load_config()

    # Initialize Cohere client for embeddings
    cohere_client = cohere.Client(config['cohere_api_key'])

    # Initialize Qdrant client
    qdrant_client = QdrantClient(
        url=config['qdrant_url'],
        api_key=config['qdrant_api_key'],
        timeout=30
    )

    # Check if collection exists, create if it doesn't
    try:
        collections = qdrant_client.get_collections()
        collection_names = [coll.name for coll in collections.collections]

        if collection_name not in collection_names:
            logger.info(f"Creating collection '{collection_name}'...")
            qdrant_client.create_collection(
                collection_name=collection_name,
                vectors_config=VectorParams(
                    size=1024,  # Cohere embeddings size for embed-english-v3.0
                    distance=Distance.COSINE
                )
            )
            logger.info(f"Collection '{collection_name}' created successfully")
        else:
            logger.info(f"Collection '{collection_name}' already exists, using existing collection")
    except Exception as e:
        logger.error(f"Error checking/creating collection: {str(e)}")
        raise

    # Prepare points for upsert
    points = []
    for i, chunk_data in enumerate(chunks):
        text = chunk_data['text']
        metadata = chunk_data['metadata']

        try:
            # Generate embedding
            embedding = get_embedding(text, cohere_client)

            # Generate unique ID
            doc_id = generate_document_id(text, metadata)

            # Create point
            point = models.PointStruct(
                id=doc_id,
                vector=embedding,
                payload={
                    'text': text,
                    'module': metadata.get('module', ''),
                    'page': metadata.get('page', ''),
                    'heading': metadata.get('heading', ''),
                    'url': metadata.get('url', ''),
                    'title': metadata.get('title', ''),
                    'chunk_index': metadata.get('chunk_index', 0),
                    'total_chunks': metadata.get('total_chunks', 1),
                    'file_path': metadata.get('file_path', '')
                }
            )
            points.append(point)

            if len(points) >= 100:  # Batch upsert every 100 points
                logger.info(f"Upserting batch of {len(points)} points to Qdrant...")
                qdrant_client.upsert(
                    collection_name=collection_name,
                    points=points
                )
                logger.info(f"Successfully upserted {len(points)} points")
                points = []  # Reset for next batch

        except Exception as e:
            logger.error(f"Error processing chunk {i}: {str(e)}")
            continue

    # Upsert remaining points
    if points:
        logger.info(f"Upserting final batch of {len(points)} points to Qdrant...")
        qdrant_client.upsert(
            collection_name=collection_name,
            points=points
        )
        logger.info(f"Successfully upserted final {len(points)} points")

    # Get final collection info
    collection_info = qdrant_client.get_collection(collection_name)
    vector_count = collection_info.points_count
    logger.info(f"Ingestion complete! Total vectors in collection: {vector_count}")

def main():
    """
    Main function to run the content ingestion pipeline.
    """
    logging.basicConfig(level=logging.INFO)

    # Load configuration
    config = load_config()

    # Define docs directory - looking for the Docusaurus docs
    docs_dir = os.path.join(os.path.dirname(__file__), '..', 'frontend_book', 'docs')
    if not os.path.exists(docs_dir):
        docs_dir = os.path.join(os.path.dirname(__file__), '..', 'docs')

    if not os.path.exists(docs_dir):
        raise FileNotFoundError(f"Docs directory not found at {docs_dir}")

    logger.info(f"Starting content ingestion from: {docs_dir}")

    # Load and process markdown files
    chunks = load_and_process_markdown_files(docs_dir)

    if not chunks:
        logger.warning("No content found to ingest!")
        return

    logger.info(f"Processing {len(chunks)} content chunks for ingestion")

    # Ingest to Qdrant
    ingest_content_to_qdrant(chunks, config['collection_name'])

    logger.info("Content ingestion pipeline completed successfully!")

if __name__ == "__main__":
    main()