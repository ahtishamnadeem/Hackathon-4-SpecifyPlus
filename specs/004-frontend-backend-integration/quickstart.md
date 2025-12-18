# Quickstart: RAG Spec-4 Frontend-Backend Integration

## Overview
Quick setup guide for the frontend-backend integration that connects the Docusaurus chat component with the FastAPI backend for RAG agent communication.

## Prerequisites
- Node.js 16+ for frontend development
- Python 3.11+ for backend services
- Access to Qdrant Cloud with existing `rag_embedding` collection
- OpenAI API access for the RAG agent
- Git for version control
- Docusaurus project already set up

## Frontend Setup

### 1. Install Frontend Dependencies
```bash
cd your-docusaurus-project/
npm install axios react-markdown
```

### 2. Add Chat Component to Your Site
Import and use the chat component in your Docusaurus pages:

```jsx
// In your Docusaurus page or layout
import ChatInterface from './components/ChatInterface';

function MyPage() {
  return (
    <div>
      <h1>My Book Content</h1>
      {/* Your book content */}
      <ChatInterface />
    </div>
  );
}
```

### 3. Enable Text Selection on Book Pages
The chat component automatically captures text selections when users highlight content on book pages.

## Backend Setup

### 1. Navigate to Backend Directory
```bash
cd backend/
```

### 2. Install Backend Dependencies
```bash
pip install fastapi uvicorn python-dotenv pydantic slowapi
```

### 3. Create Environment Configuration
Create a `.env` file in the backend directory with the following variables:
```env
# Backend Configuration
BACKEND_HOST=0.0.0.0
BACKEND_PORT=8000

# Qdrant Configuration
QDRANT_URL=your_qdrant_cloud_url_here
QDRANT_API_KEY=your_qdrant_api_key_here

# OpenAI Configuration
OPENAI_API_KEY=your_openai_api_key_here

# CORS Configuration
ALLOWED_ORIGINS=http://localhost:3000,http://localhost:3001,https://your-docusaurus-site.com
```

### 4. Run the Backend Service
```bash
uvicorn integration_api:app --host 0.0.0.0 --port 8000 --reload
```

## Usage

### Integrate Chat Component
Add the chat component to your Docusaurus pages where you want users to interact with the RAG agent:

```jsx
import ChatInterface from '../components/ChatInterface';

function BookPage() {
  return (
    <div>
      <h1>Book Chapter Content</h1>
      {/* Book content here */}

      <div className="chat-section">
        <h2>Ask Questions About This Content</h2>
        <ChatInterface />
      </div>
    </div>
  );
}
```

### Make API Requests
The component will automatically handle API communication with the backend endpoints.

## API Endpoints

### Process User Queries
```bash
curl -X POST "http://localhost:8000/chat/query" \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What is ROS 2?",
    "selected_text": "Robot Operating System 2 is a flexible framework for writing robot software",
    "context_metadata": {
      "page_url": "https://book.example.com/ros2-intro",
      "page_title": "Introduction to ROS 2",
      "module": "Chapter 1"
    }
  }'
```

### Send Text Selection
```bash
curl -X POST "http://localhost:8000/chat/text-selection" \
  -H "Content-Type: application/json" \
  -d '{
    "selected_text": "This is the selected text from the book",
    "page_url": "https://book.example.com/chapter-2",
    "page_title": "Advanced Concepts",
    "position_start": 150,
    "position_end": 190,
    "timestamp": "2025-12-19T10:30:00Z"
  }'
```

### Check Service Health
```bash
curl -X GET "http://localhost:8000/chat/health"
```

## Configuration Options

### Frontend Configuration
- `CHAT_CONTAINER_ID`: ID of the container element for the chat component
- `API_BASE_URL`: Base URL for the backend API (default: http://localhost:8000)
- `ENABLE_LOADING_STATES`: Whether to show loading indicators (default: true)
- `ENABLE_ERROR_HANDLING`: Whether to show error messages (default: true)

### Backend Configuration
- `BACKEND_HOST`: Host for the backend server (default: 0.0.0.0)
- `BACKEND_PORT`: Port for the backend server (default: 8000)
- `ALLOWED_ORIGINS`: Comma-separated list of allowed origins for CORS
- `QDRANT_URL`: URL of the Qdrant Cloud instance
- `QDRANT_API_KEY`: API key for accessing Qdrant Cloud
- `OPENAI_API_KEY`: API key for OpenAI services

## Testing

### Run Frontend Tests
```bash
npm test
```

### Run Backend Tests
```bash
python -m pytest tests/ -v
```

### End-to-End Test
1. Start the backend service
2. Integrate the chat component in your Docusaurus site
3. Select text on a book page
4. Submit a query in the chat interface
5. Verify that you receive a relevant response from the RAG agent

## Troubleshooting

### Common Issues

1. **CORS Errors**
   - Verify `ALLOWED_ORIGINS` in the backend .env file includes your frontend URL
   - Check that the frontend is served from an allowed origin

2. **API Connection Issues**
   - Verify that Qdrant and OpenAI services are accessible
   - Check that API keys are correctly configured

3. **Text Selection Not Working**
   - Ensure the text selection event listeners are properly attached
   - Verify the page structure allows for text selection capture

### Health Check
If experiencing issues, run the health check:
```bash
curl -X GET "http://localhost:8000/chat/health"
```

## Next Steps
1. Integrate the chat component into your Docusaurus pages
2. Customize the chat UI to match your site's styling
3. Add analytics to track user interactions
4. Implement additional error handling and loading states
5. Optimize performance for production deployment