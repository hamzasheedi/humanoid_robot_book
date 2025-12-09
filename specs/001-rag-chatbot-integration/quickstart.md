# Quickstart Guide: RAG Chatbot Integration

## Overview
This guide will help you set up the RAG Chatbot for the Physical AI & Humanoid Robotics textbook. The system consists of a FastAPI backend that processes questions and retrieves answers from textbook content, and a React frontend that integrates directly into the Docusaurus-based textbook.

## Prerequisites
- Python 3.11+
- Node.js 18+
- Access to OpenAI API key
- Qdrant Cloud Free Tier account
- Neon Postgres database

## Setup Instructions

### 1. Backend Setup (FastAPI)

1. Navigate to the backend directory:
   ```bash
   cd backend
   ```

2. Create a virtual environment and install dependencies:
   ```bash
   python -m venv venv
   source venv/bin/activate  # On Windows: venv\Scripts\activate
   pip install -r requirements.txt
   ```

3. Set up environment variables:
   ```bash
   cp .env.example .env
   # Edit .env with your API keys and connection strings
   ```

4. Run the backend server:
   ```bash
   uvicorn src.api.main:app --reload --port 8000
   ```

### 2. Frontend Setup (Docusaurus)

1. Navigate to the frontend directory:
   ```bash
   cd frontend
   ```

2. Install dependencies:
   ```bash
   npm install
   ```

3. Set up environment variables:
   ```bash
   cp .env.example .env
   # Edit .env with your backend API URL
   ```

4. Run the development server:
   ```bash
   npm run dev
   ```

### 3. Initialize Textbook Content

1. The system needs to load textbook content into Qdrant for vector search. Run the indexing script:
   ```bash
   python -m src.services.embedding_service index_textbook
   ```

2. This will:
   - Parse textbook content from the docs directory
   - Create embeddings using OpenAI
   - Store vectors in Qdrant
   - Save metadata in Neon Postgres

## API Endpoints

### Chat Endpoints
- `POST /chat/ask` - Submit a question and get an answer
- `POST /chat/context` - Get context from selected text
- `GET /chat/history` - Retrieve chat history for a session

### Textbook Endpoints
- `GET /textbook/modules` - Get available textbook modules

## Testing the Integration

1. Visit the Docusaurus site in your browser (usually http://localhost:3000)
2. Navigate to any textbook page
3. Use the embedded chatbot to ask questions about the content
4. Test text selection functionality by selecting text and clicking "Ask about this"

## Environment Configuration

Required environment variables for the backend:
```
OPENAI_API_KEY=your_openai_api_key
QDRANT_URL=your_qdrant_cluster_url
QDRANT_API_KEY=your_qdrant_api_key
NEON_DB_URL=your_neon_postgres_connection_string
```

## Troubleshooting

1. **No answers returned**
   - Check that textbook content has been indexed in Qdrant
   - Verify OpenAI API key is valid and has access

2. **Slow responses**
   - Check network connection to Qdrant and OpenAI
   - Ensure sufficient rate limits on API keys

3. **Frontend not connecting to backend**
   - Verify API URL in frontend environment variables
   - Check that backend server is running and accessible

## Next Steps

1. Deploy the backend to a cloud provider
2. Build and deploy the Docusaurus frontend
3. Set up automated content indexing for textbook updates
4. Monitor accuracy metrics and user feedback