# Quickstart Guide: RAG Chatbot Integration

## Overview
This guide will help you set up and run the RAG Chatbot for the Physical AI & Humanoid Robotics textbook. The system consists of a FastAPI backend that processes questions and retrieves answers from textbook content, and a React frontend that integrates directly into the Docusaurus-based textbook.

## Prerequisites
- Python 3.11+ with pip
- Node.js 18+ with npm
- Access to OpenAI API key
- Qdrant Cloud Free Tier account
- Neon Postgres account

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
   # Edit .env with your API keys and connection strings:
   # OPENAI_API_KEY=your_openai_api_key_here
   # QDRANT_URL=your_qdrant_cluster_url_here
   # QDRANT_API_KEY=your_qdrant_api_key_here
   # NEON_DB_URL=your_neon_postgres_connection_string_here
   # API_KEY=your_backend_api_key_here
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
   # Edit .env with your backend API URL:
   # REACT_APP_API_BASE_URL=http://localhost:8000
   ```

4. Run the development server:
   ```bash
   npm run dev
   ```

### 3. Load Textbook Content

1. Once the system is running, you need to index the textbook content into Qdrant for vector search. Run the indexing script:
   ```bash
   python -m src.services.embedding_service index_textbook
   ```

2. This will parse textbook content, create embeddings using OpenAI, and store vectors in Qdrant with metadata in Neon Postgres.

## API Endpoints

### Chat Endpoints
- `POST /chat/ask` - Submit a question and get an answer
- `POST /chat/context` - Get context from selected text
- `GET /chat/history` - Retrieve chat history for a session

### Textbook Endpoints
- `GET /textbook/modules` - Get available textbook modules
- `GET /textbook/modules/{module_id}/sections` - Get sections in a specific module

## Using the Chatbot

1. Visit the Docusaurus site in your browser (usually http://localhost:3000)
2. Navigate to any textbook page
3. Click the floating chatbot icon in the bottom-right corner
4. Use the embedded chatbot to ask questions about the content
5. Test text selection functionality by selecting text and asking about it

## Configuration

Required environment variables for the backend:
```
OPENAI_API_KEY=your_openai_api_key
QDRANT_URL=your_qdrant_cluster_url
QDRANT_API_KEY=your_qdrant_api_key
NEON_DB_URL=your_neon_postgres_connection_string
API_KEY=your_backend_api_key
```

## Testing the Integration

1. Ask various questions about the textbook content and verify answers are accurate
2. Test text selection functionality by selecting text and clicking "Ask About Selected Text"
3. Check that conversation history is maintained properly
4. Verify that responses include proper citations to textbook content
5. Test the response time - should be <2 seconds for cloud and <5 seconds for local deployments

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

1. Customize the chatbot UI to match your textbook's theme
2. Add more textbook content to the indexing process
3. Fine-tune the RAG parameters for optimal response quality
4. Set up monitoring for accuracy and performance metrics