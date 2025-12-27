# HACKATHON BY (GIAIC)
# RAG Agent Integrated Docusaurus Book - Physical AI and Humanoid Robotics

This educational book provides comprehensive coverage of ROS 2 as the robotic nervous system for humanoid control, communication, and embodiment. It is designed for AI and robotics students with basic Python knowledge entering Physical AI and humanoid robotics.

The book features an integrated RAG (Retrieval-Augmented Generation) chatbot that allows users to ask questions about the book content and get contextually relevant answers.

## 📚 Overview

This book covers four main modules:

1. **Introduction to ROS 2 and Robotic Middleware** - Learn the fundamentals of ROS 2 as the robotic nervous system for humanoid control, communication, and embodiment
2. **Advanced ROS 2 Concepts** - Explore advanced publisher-subscriber patterns, lifecycle nodes, and performance optimization
3. **Perception and Navigation Systems** - Learn about sensor integration, computer vision, and navigation systems for humanoid robots
4. **AI Integration and Capstone Project** - Integrate voice processing, cognitive planning, and autonomous behaviors in humanoid robots

## 🎯 Learning Objectives

After completing this course, students will be able to:
- Understand the role of ROS 2 in Physical AI and humanoid robotics
- Use rclpy to build ROS 2 nodes that interface with Python AI agents
- Create URDF models with proper links, joints, and coordinate frames
- Implement publishers, subscribers, services, and actions in ROS 2
- Prepare humanoid robots for simulation and control
- Interact with the book content using AI-powered question answering

## ✨ Key Features

### Frontend Features
- **Docusaurus Book**: Complete documentation site with 4 modules covering ROS 2 fundamentals
- **Modern UI**: Professional design with animations, dark/light mode, and responsive layout
- **Floating Chatbot**: Accessible AI assistant on all pages with modern interface
- **Text Selection**: Users can select text on any page to provide context for their questions
- **Hero Banner**: Professional background image with overlay for enhanced visual appeal
- **Card Animations**: Scroll-triggered animations for homepage features
- **Footer Enhancement**: Modern robotics-inspired footer with improved accessibility
- **Urdu Translation**: Optional Urdu translation feature for chapters (opt-in functionality)
- **Dark Mode Default**: Website loads in dark mode by default with toggle option

### Backend Features
- **RAG Chatbot**: Integrated chat interface that answers questions based on book content
- **Vector Database**: Qdrant Cloud integration for semantic search and retrieval
- **Multi-LLM Support**: Google AI Studio (Gemini) as primary, with OpenAI as fallback
- **Answer Routing**: Intelligent routing between book-grounded and general knowledge responses
- **Source Attribution**: Responses include references to the original book content
- **Cohere Embeddings**: High-quality embeddings for semantic search
- **Session Management**: SQLite-based session storage for conversation history
- **Error Handling**: Robust error handling with graceful fallbacks

## 🛠️ Technology Stack

### Frontend
- **Framework**: Docusaurus v3 with React
- **Styling**: CSS modules, custom CSS with responsive design
- **Animations**: Intersection Observer API for scroll-triggered animations
- **UI Components**: Custom React components for chatbot and translation features

### Backend
- **Framework**: FastAPI with Python 3.8+
- **Vector Database**: Qdrant Cloud with query_points API
- **Embeddings**: Cohere for text embeddings
- **LLMs**: Google AI Studio (Gemini) as primary, OpenAI as fallback
- **Database**: SQLite for development, Postgres support available
- **Communication**: HTTP/REST API with CORS support

## 📁 Project Structure

```
├── frontend_book/          # Docusaurus book application
│   ├── docs/              # Book content and modules
│   │   ├── module-1/      # Introduction to ROS 2
│   │   ├── module-2/      # Advanced ROS 2 Concepts
│   │   ├── module-3/      # Perception and Navigation
│   │   └── module-4/      # AI Integration
│   ├── src/               # Frontend source code
│   │   ├── components/    # React components (FloatingChatbot, TranslationToggle)
│   │   ├── pages/         # Custom pages (chat, index)
│   │   ├── css/           # Custom CSS styles
│   │   ├── theme/         # Docusaurus theme customization
│   │   └── utils/         # Utility functions (translation API)
│   ├── static/            # Static assets (images, files)
│   └── package.json       # Frontend dependencies
├── backend/               # RAG agent backend
│   ├── main.py            # FastAPI application entry point
│   ├── agent.py           # RAG agent logic and processing
│   ├── retrieval.py       # Content retrieval from Qdrant
│   ├── config.py          # Configuration management
│   ├── db_models.py       # Database models (SQLAlchemy)
│   ├── db_utils.py        # Database utility functions
│   ├── openai_agents.py   # OpenAI agent integration
│   └── requirements.txt   # Backend dependencies
├── README.md              # This file
└── .env                   # Environment variables (not tracked)
```

## 📋 Prerequisites

- Python 3.8+
- Node.js 18+
- Access to Google AI Studio API (Gemini models)
- Access to OpenAI API (as fallback)
- Access to Cohere API (for embeddings)
- Qdrant Cloud account
- Git for version control

## 🚀 Installation and Setup

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

4. Create a `.env` file with your API keys (use placeholders as shown, but replace with your actual keys):
```env
# Qdrant Configuration (Update with your actual Qdrant Cloud credentials)
# Format: https://<cluster-name>.<region>.cloud.qdrant.io:6333
# Example: https://your-cluster-name.us-east-1.aws.cloud.qdrant.io:6333
QDRANT_URL=your_actual_qdrant_url_here
QDRANT_API_KEY=your_actual_qdrant_api_key_here
COLLECTION_NAME=rag_embedding

# Cohere Configuration
COHERE_API_KEY=your_actual_cohere_api_key_here

# Google AI Studio Configuration
GOOGLE_AI_STUDIO_API_KEY=your_actual_google_ai_studio_api_key_here

# OpenAI Configuration
OPENAI_API_KEY=your_actual_openai_api_key_here

# Neon Postgres Configuration (set to actual URL if using Postgres, or leave empty to use SQLite)
NEON_DATABASE_URL=

# Backend Configuration
BACKEND_HOST=0.0.0.0
BACKEND_PORT=8000
ALLOWED_ORIGINS=http://localhost:3000,http://localhost:3001,http://localhost:3002,http://localhost:3003,https://your-docusaurus-site.com

# Agent Configuration
MODEL_NAME=gpt-3.5-turbo
MAX_RETRIEVALS=5
TEMPERATURE=0.7
MAX_TOKENS=500
```

> ⚠️ **Security Warning**: Never commit actual API keys to version control. The `.env` file is automatically excluded by `.gitignore`, but make sure to keep your real API keys secure and never share them publicly.

5. Start the backend server:
```bash
python -m uvicorn main:app --host 0.0.0.0 --port 8000
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

## 📖 Usage

1. Access the book at `http://localhost:3000`
2. Use the floating chat icon in the bottom-right corner to open the RAG chatbot
3. Ask questions about the book content
4. Select text on any page to provide context for your questions
5. The chatbot will respond with answers based on the book content
6. Toggle between light/dark mode using the theme switcher
7. Use the Urdu translation toggle (if enabled) to translate chapter content

### Answer Routing Logic
- **Book-Grounded Answers**: When retrieval returns relevant content with confidence above threshold, the system responds with "--- According to this book: ---" prefix
- **General Knowledge Answers**: When no relevant content is found, the system responds with "--- Based on general robotics knowledge (not from the book): ---" prefix

## 🔌 API Endpoints

The backend provides the following endpoints:

- `GET /health` - Health check for services
- `GET /info` - Agent information and capabilities
- `POST /chat/send` - Send a query to the RAG agent
- `POST /chat/text-selection` - Submit selected text for context (if implemented)

## 🏗️ Architecture

The system consists of:

### Frontend Layer
- **Docusaurus v3**: Static site generator with React
- **Custom Components**: Floating chatbot, translation toggle, animated cards
- **Theme Integration**: Custom Docusaurus theme components
- **CSS Modules**: Modern styling with responsive design

### Backend Layer
- **FastAPI**: High-performance Python web framework
- **RAG Agent**: Core logic for retrieval-augmented generation
- **Qdrant Client**: Vector database interaction
- **Cohere Client**: Embedding generation
- **LLM Clients**: Google AI Studio and OpenAI integration
- **Database Layer**: SQLite for session management

### Integration Layer
- **CORS**: Cross-origin resource sharing configuration
- **Rate Limiting**: API rate limiting for protection
- **Error Handling**: Comprehensive error handling with fallbacks

## ⚙️ Configuration

### Environment Variables

The system uses several environment variables for configuration:

- `QDRANT_URL` and `QDRANT_API_KEY`: For connecting to Qdrant Cloud
- `COHERE_API_KEY`: For generating embeddings
- `GOOGLE_AI_STUDIO_API_KEY`: For Google's Gemini models
- `OPENAI_API_KEY`: For OpenAI services (fallback)
- `NEON_DATABASE_URL`: For Postgres database (optional, uses SQLite if empty)
- `BACKEND_HOST` and `BACKEND_PORT`: For configuring the backend server
- `ALLOWED_ORIGINS`: For CORS configuration
- `MODEL_NAME`: LLM model selection
- `MAX_RETRIEVALS`: Maximum number of chunks to retrieve
- `TEMPERATURE`: LLM temperature setting
- `MAX_TOKENS`: Maximum tokens for responses

### Customization Options

You can customize the following aspects:

- **Book Content**: Edit files in `frontend_book/docs/`
- **Chat Widget**: Modify `frontend_book/src/components/FloatingChatbot.jsx`
- **Translation Feature**: Update `frontend_book/src/components/TranslationToggle/`
- **RAG Logic**: Update `backend/agent.py` and `backend/retrieval.py`
- **UI Styling**: Modify `frontend_book/src/css/custom.css`
- **Theme Components**: Customize Docusaurus theme overrides in `frontend_book/src/theme/`

## 🌐 Deployment

### Frontend Deployment

The Docusaurus frontend can be deployed to various platforms:

- **Vercel, Netlify, GitHub Pages**: Static hosting services
- **Any static hosting service**: Run `npm run build` to generate static files in the `build/` directory
- **Custom servers**: Serve the `build/` directory as static files

### Backend Deployment

The FastAPI backend can be deployed to:

- **Heroku, Railway, Render**: Python application hosting
- **AWS, GCP, Azure**: Cloud platform deployment
- **Docker**: Containerized deployment with `Dockerfile` (if created)
- **Any platform**: That supports Python applications with dependencies

### Environment Configuration for Production

For production deployment, ensure:
- Secure API keys are properly configured
- CORS settings are restricted to your domain
- Database is properly configured for production
- Logging is configured appropriately

## 🔧 Troubleshooting

### Common Issues

1. **CORS Errors**: Ensure your `ALLOWED_ORIGINS` includes your frontend URL
2. **API Keys**: Verify all required API keys are set in the `.env` file
3. **Network Issues**: Check that both frontend and backend are running on the correct ports
4. **Qdrant Connection**: Verify Qdrant URL and API key are correct
5. **Database Issues**: Ensure database URL is properly formatted

### Validation

Run the validation script to check the RAG pipeline:
```bash
cd backend/
python -c "from retrieval import validate_qdrant_connection; result = validate_qdrant_connection(); print(result)"
```

### Debugging Tips

- Check backend logs for detailed error messages
- Verify API keys are valid and have sufficient permissions
- Ensure Qdrant collection contains data for retrieval
- Test individual components separately before full integration

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/AmazingFeature`)
3. Make your changes
4. Add tests if applicable
5. Ensure code follows project standards
6. Commit your changes (`git commit -m 'Add some AmazingFeature'`)
7. Push to the branch (`git push origin feature/AmazingFeature`)
8. Open a pull request

### Development Guidelines

- Follow PEP 8 for Python code
- Use meaningful variable and function names
- Write comprehensive comments and docstrings
- Maintain consistent code style
- Test changes thoroughly before submitting

## 📄 License

This project is licensed under the MIT License - see the LICENSE file for details.

## 🆘 Support

For support, please:
- Open an issue in the GitHub repository
- Check the troubleshooting section above
- Verify your environment configuration
- Review the API documentation and logs

## 🙏 Acknowledgments

- Docusaurus team for the excellent documentation framework
- FastAPI team for the high-performance web framework
- Qdrant team for the vector database solution
- Cohere for embedding services
- Google AI Studio and OpenAI for LLM services
- The open-source community for various libraries and tools used in this project

---

<p align="center">
  <b>Developed By ❤️ | CodeWithAhtii</b>
</p>

For more information, see the [Docusaurus documentation](https://docusaurus.io/).

## Alot efforts
