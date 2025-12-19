# RAG Agent Integrated Docusaurus Book - ROS 2 Fundamentals for Physical AI and Humanoid Robotics

This educational book provides comprehensive coverage of ROS 2 as the robotic nervous system for humanoid control, communication, and embodiment. It is designed for AI and robotics students with basic Python knowledge entering Physical AI and humanoid robotics.

The book features an integrated RAG (Retrieval-Augmented Generation) chatbot that allows users to ask questions about the book content and get contextually relevant answers.

## Overview

This book covers four main modules:

1. **Introduction to ROS 2 and Robotic Middleware** - Learn the fundamentals of ROS 2 as the robotic nervous system for humanoid control, communication, and embodiment
2. **Advanced ROS 2 Concepts** - Explore advanced publisher-subscriber patterns, lifecycle nodes, and performance optimization
3. **Perception and Navigation Systems** - Learn about sensor integration, computer vision, and navigation systems for humanoid robots
4. **AI Integration and Capstone Project** - Integrate voice processing, cognitive planning, and autonomous behaviors in humanoid robots

## Learning Objectives

After completing this course, students will be able to:
- Understand the role of ROS 2 in Physical AI and humanoid robotics
- Use rclpy to build ROS 2 nodes that interface with Python AI agents
- Create URDF models with proper links, joints, and coordinate frames
- Implement publishers, subscribers, services, and actions in ROS 2
- Prepare humanoid robots for simulation and control
- Interact with the book content using AI-powered question answering

## Features

- **Docusaurus Book**: Complete documentation site with 4 modules covering ROS 2 fundamentals
- **RAG Chatbot**: Integrated chat interface that answers questions based on book content
- **Text Selection**: Users can select text on any page to provide context for their questions
- **Floating Widget**: Chat interface accessible from all pages via a floating widget
- **Source Attribution**: Responses include references to the original book content

## Technology Stack

- **Frontend**: Docusaurus v3 with React
- **Backend**: FastAPI with Python
- **Vector Database**: Qdrant Cloud
- **Embeddings**: Cohere
- **LLM**: OpenAI GPT-4 Turbo
- **Communication**: HTTP/REST API

## Project Structure

```
├── frontend_book/          # Docusaurus book application
│   ├── docs/              # Book content and modules
│   ├── src/               # Frontend source code
│   │   ├── components/    # React components (including FloatingChatbot)
│   │   ├── pages/         # Docusaurus pages
│   │   └── theme/         # Docusaurus theme customization
│   └── package.json       # Frontend dependencies
├── backend/               # RAG agent backend
│   ├── integration_api.py # FastAPI integration layer
│   ├── agent.py           # RAG agent logic
│   ├── retrieval.py       # Content retrieval from Qdrant
│   ├── config.py          # Configuration management
│   └── requirements.txt   # Backend dependencies
└── README.md              # This file
```

## Prerequisites

- Python 3.8+
- Node.js 18+
- Access to OpenAI API
- Access to Cohere API
- Qdrant Cloud account

## Installation and Setup

### Backend Setup

1. Navigate to the backend directory:
```bash
cd backend/
```

2. Create and activate a virtual environment:
```bash
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
```

3. Install dependencies:
```bash
pip install -r requirements.txt
```

4. Create a `.env` file with your API keys:
```env
# Qdrant Configuration
QDRANT_URL=your_qdrant_url_here
QDRANT_API_KEY=your_qdrant_api_key_here
COLLECTION_NAME=book_content

# OpenAI Configuration
OPENAI_API_KEY=your_openai_api_key_here
MODEL_NAME=gpt-4-turbo

# Cohere Configuration (for embeddings)
COHERE_API_KEY=your_cohere_api_key_here

# Agent Configuration
MAX_RETRIEVALS=5
TEMPERATURE=0.7
MAX_TOKENS=500

# Server Configuration
BACKEND_HOST=0.0.0.0
BACKEND_PORT=8000
ALLOWED_ORIGINS=http://localhost:3000,http://localhost:3001,http://localhost:3002,http://localhost:3003,https://your-docusaurus-site.com
```

5. Start the backend server:
```bash
python integration_api.py
```

The backend will be available at `http://localhost:8000`.

### Frontend Setup

1. Navigate to the frontend_book directory:
```bash
cd frontend_book/
```

2. Install dependencies:
```bash
npm install
```

3. Start the development server:
```bash
npm start
```

The frontend will be available at `http://localhost:3000`.

## Usage

1. Access the book at `http://localhost:3000`
2. Use the floating chat icon in the bottom-right corner to open the RAG chatbot
3. Ask questions about the book content
4. Select text on any page to provide context for your questions
5. The chatbot will respond with answers based on the book content

## API Endpoints

The backend provides the following endpoints:

- `GET /chat/health` - Health check
- `POST /chat/send` - Send a query to the RAG agent
- `POST /chat/text-selection` - Submit selected text for context

## Architecture

The system consists of:

1. **Frontend Layer**: Docusaurus application with a floating chat widget
2. **Integration Layer**: FastAPI backend that handles communication between frontend and RAG agent
3. **RAG Layer**: Agent that retrieves relevant content from Qdrant and generates answers using OpenAI
4. **Vector Database**: Qdrant Cloud storing book content embeddings for semantic search

## Configuration

### Environment Variables

The system uses several environment variables for configuration:

- `QDRANT_URL` and `QDRANT_API_KEY`: For connecting to Qdrant Cloud
- `OPENAI_API_KEY`: For accessing OpenAI services
- `COHERE_API_KEY`: For generating embeddings
- `BACKEND_HOST` and `BACKEND_PORT`: For configuring the backend server
- `ALLOWED_ORIGINS`: For CORS configuration

### Customization

You can customize the following aspects:

- **Book Content**: Edit files in `frontend_book/docs/`
- **Chat Widget**: Modify `frontend_book/src/components/FloatingChatbot.jsx`
- **RAG Logic**: Update `backend/agent.py` and `backend/retrieval.py`
- **UI Styling**: Modify `frontend_book/src/css/custom.css`

## Deployment

### Frontend Deployment

The Docusaurus frontend can be deployed to various platforms:

- Vercel, Netlify, GitHub Pages
- Any static hosting service
- Run `npm run build` to generate static files in the `build/` directory

### Backend Deployment

The FastAPI backend can be deployed to:

- Heroku, Railway, Render
- AWS, GCP, Azure
- Any platform that supports Python applications

## Troubleshooting

### Common Issues

1. **CORS Errors**: Ensure your `ALLOWED_ORIGINS` includes your frontend URL
2. **API Keys**: Verify all required API keys are set in the `.env` file
3. **Network Issues**: Check that both frontend and backend are running on the correct ports

### Validation

Run the validation script to check the RAG pipeline:
```bash
cd backend/
python -m validation --collection-name book_content
```

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Add tests if applicable
5. Submit a pull request

## Support

For support, please open an issue in the GitHub repository.
