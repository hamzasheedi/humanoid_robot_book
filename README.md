# Physical AI & Humanoid Robotics Textbook with Integrated RAG Chatbot

This project implements an interactive textbook with an embedded RAG (Retrieval Augmented Generation) chatbot for learning about Physical AI and Humanoid Robotics.

## Features

- Interactive textbook with embedded AI assistant
- Ask questions about textbook content and receive context-aware answers
- Text selection functionality to ask questions about specific passages
- Conversation history tracking
- Instructor dashboard for monitoring student progress

## Architecture

### Backend
- FastAPI web framework for the backend API
- Qdrant vector database for RAG (Retrieval Augmented Generation)
- Neon Postgres database for storing chat logs and analytics
- OpenAI for generating answers

### Frontend
- Docusaurus-based textbook interface
- Embedded chatbot window
- React-based UI components

## Installation

### Backend Setup

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

### Frontend Setup

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

## Environment Variables

You'll need the following environment variables:

- `OPENAI_API_KEY`: Your OpenAI API key
- `QDRANT_URL`: URL for your Qdrant cluster
- `QDRANT_API_KEY`: API key for Qdrant (if using cloud)
- `NEON_DB_URL`: Connection string for Neon Postgres database
- `API_KEY`: Backend API key for authentication

## Usage

Once both servers are running, visit the frontend URL (typically http://localhost:3000) to access the textbook. The chatbot will be available as a floating window in the bottom right corner.

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Submit a pull request

## License

This project is licensed under the MIT License.