# Quickstart Guide: RAG Chatbot Integration with Cohere API

## Overview
This guide will help you set up the RAG Chatbot for the Physical AI & Humanoid Robotics textbook. The system consists of a FastAPI backend that processes questions and retrieves answers from textbook content using Cohere embeddings, and a React frontend that integrates directly into the Docusaurus-based textbook.

## Prerequisites
- Python 3.11+
- Node.js 18+
- Access to Cohere API key
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
   # Edit .env with your API keys and connection strings:
   # COHERE_API_KEY=your_cohere_api_key
   # QDRANT_URL=your_qdrant_cluster_url
   # QDRANT_API_KEY=your_qdrant_api_key (if using cloud)
   # NEON_DB_URL=your_neon_postgres_connection_string
   # API_KEY=your_backend_api_key
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

### 3. Initialize Textbook Content

1. The system needs to load textbook content into Qdrant for vector search. Run the indexing script:
   ```bash
   python -m src.services.embedding_service index_textbook
   ```

2. This will:
   - Parse textbook content from the docs directory
   - Create embeddings using Cohere
   - Store vectors in Qdrant
   - Save metadata in Neon Postgres

## API Endpoints

### Chat Endpoints
- `POST /chat/ask`: Submit a question and get an answer based only on textbook content
- `POST /chat/context`: Get relevant context from selected text
- `GET /chat/history`: Retrieve the history of questions and answers for a session

### Textbook Endpoints
- `GET /textbook/modules`: Get a list of available textbook modules

### Admin Endpoints
- `POST /admin/reindex`: Regenerate embeddings for textbook content (requires admin privileges)

## Testing the Integration

1. Visit the Docusaurus site in your browser (usually http://localhost:3000)
2. Navigate to any textbook page
3. Use the embedded chatbot to ask questions about the content
4. Test text selection functionality by selecting text and clicking the floating "Ask About Selected" button

## Security Considerations

- All API keys are stored server-side and never exposed to the client
- The backend validates all requests with the API key
- No sensitive information is logged or transmitted unnecessarily

## Performance Optimization

- Responses are typically returned within 2 seconds for cloud deployments
- For local deployments, responses take up to 5 seconds
- Content is pre-indexed using Cohere embeddings for fast retrieval
- Qdrant vector store enables efficient similarity search

## Troubleshooting

1. **Error: "No response from chatbot"**
   - Verify the backend server is running
   - Check that the Cohere API key is valid in your environment

2. **Error: "Connection refused" for Qdrant**
   - Verify your Qdrant cluster URL and API key are correct
   - Ensure your network allows connections to Qdrant

3. **Slow response times**
   - Check your Cohere and Qdrant connection speeds
   - Verify you're using the fastest available Cohere model for your region