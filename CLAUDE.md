# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a Physical AI & Humanoid Robotics Textbook with an integrated RAG (Retrieval Augmented Generation) chatbot. The project combines a Docusaurus-based frontend textbook interface with a FastAPI backend that uses Qdrant vector database for RAG functionality to answer questions about the textbook content.

## Architecture

### Backend (Python/FastAPI)
- FastAPI web framework for the backend API
- Qdrant vector database for RAG (Retrieval Augmented Generation)
- Neon Postgres database for storing chat logs and analytics
- OpenAI for generating answers
- Services architecture with dedicated modules for Qdrant, embedding, RAG, and Postgres
- Models for Question, Answer, and ChatSession entities

### Frontend (React/Docusaurus)
- Docusaurus-based textbook interface
- Embedded chatbot window component
- Text selection functionality to ask questions about specific passages
- Conversation history tracking
- React-based UI components with CSS styling

## Key Files and Directories

- `backend/` - Python FastAPI backend with services, models, and API routes
- `frontend/` - Docusaurus-based frontend with React components
- `backend/src/api/main.py` - Main FastAPI application with startup/shutdown events
- `backend/src/api/chat.py` - Chat-related API endpoints (ask, context, history)
- `frontend/src/components/ChatbotWindow.jsx` - Main chatbot UI component
- `backend/requirements.txt` - Python dependencies
- `package.json` - Root package configuration
- `frontend/package.json` - Frontend package configuration

## Development Commands

### Backend Development
```bash
cd backend
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
pip install -r requirements.txt
uvicorn src.api.main:app --reload --port 8000
```

### Frontend Development
```bash
cd frontend
npm install
npm run dev
```

### Testing
```bash
cd backend
pytest  # Run all tests
pytest tests/unit/  # Run unit tests only
pytest tests/integration/  # Run integration tests only
pytest tests/contract/  # Run contract tests only
```

### Building and Deployment
```bash
# Frontend build
npm run build

# For Vercel deployment
npm run prepare-deployment
```

## Environment Variables

Both backend and frontend require environment variables:

Backend (.env):
- `OPENAI_API_KEY`: Your OpenAI API key
- `QDRANT_URL`: URL for your Qdrant cluster
- `QDRANT_API_KEY`: API key for Qdrant (if using cloud)
- `NEON_DB_URL`: Connection string for Neon Postgres database
- `API_KEY`: Backend API key for authentication

Frontend (.env):
- `REACT_APP_BACKEND_URL`: URL for the backend API

## API Endpoints

### Backend API
- `GET /health` - Health check endpoint
- `POST /chat/ask` - Submit questions to the RAG system
- `POST /chat/context` - Get context for selected text
- `GET /chat/history` - Retrieve chat history for a session
- `GET /textbook/` - Textbook-related endpoints

### Frontend Components
- ChatbotWindow.jsx - Main chat interface component
- TextSelectionHandler.jsx - Handles text selection for context queries
- ChatHistory.jsx - Displays conversation history

## Testing Structure

The backend includes three types of tests:
- Unit tests: `backend/tests/unit/`
- Integration tests: `backend/tests/integration/`
- Contract tests: `backend/tests/contract/`

## Common Development Tasks

1. Adding new API endpoints: Add to appropriate router file in `backend/src/api/`
2. Modifying chatbot UI: Update components in `frontend/src/components/`
3. Adding new textbook content: Update Docusaurus documentation files in `frontend/docs/`
4. Modifying RAG behavior: Update services in `backend/src/services/`
5. Adding database models: Create in `backend/src/models/`