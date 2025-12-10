# Integrated RAG Chatbot for Textbook

## Overview

The *Physical AI & Humanoid Robotics* textbook now features an integrated RAG (Retrieval-Augmented Generation) chatbot powered by Cohere embeddings. This chatbot enables students to ask questions about textbook content and receive context-aware answers based on the course material.

## Features

### Student Question-Answering

- Ask questions about textbook content directly from any page
- Receive answers based strictly on textbook content
- Get citations showing which textbook sections informed the response
- Track conversation history within each session

### Text Selection Functionality

- Select text on any textbook page
- Ask for clarification on specific content using the "Ask About Selected Text" button
- Receive context-aware responses based on the selected text

### Instructor Tools

- Advanced question handling for complex technical queries
- Confidence scoring for answer accuracy
- Educational value assessment
- Citation functionality for specific textbook references

### Learning Progress Tracking

- Session-based conversation history
- Ability to review previous questions and answers
- Filtering and search capabilities for chat history

## How to Use

### Asking Questions

1. Navigate to any textbook page
2. Use the embedded chatbot interface
3. Type your question about the content
4. Receive an answer based on textbook material

### Text Selection

1. Highlight any text in the textbook
2. Click on the floating "Ask About Selected Text" button
3. The chatbot will provide context-aware explanations

### Reviewing History

1. Access your conversation history
2. Review previous questions and answers
3. Track your learning progress

## Technical Implementation

### Architecture

- **Frontend:** React components integrated into Docusaurus
- **Backend:** FastAPI server with Cohere integration
- **Vector Store:** Qdrant for embedding storage and retrieval
- **Database:** Neon Postgres for logging and session management

### Technology Stack

- **Cohere API:** For embeddings and answer generation
- **Qdrant:** Vector database for semantic search
- **Neon Postgres:** Transactional database for logs and metadata
- **FastAPI:** Backend API framework
- **React:** Frontend components for chat interface
- **Docusaurus:** Static site generator for textbook

## Performance

- **Response times:** &lt;2 seconds for cloud deployment, &lt;5 seconds for local
- **Accuracy:** &gt;=95% textbook content accuracy
- **Security:** All API keys stored server-side with proper validation

## Troubleshooting

### Common Issues

- **No response from chatbot:** Verify backend server is running and Cohere API key is valid
- **Connection refused for Qdrant:** Check Qdrant cluster URL and API key in environment variables
- **Slow response times:** Verify network connection to Cohere and Qdrant

## Integration Benefits

### For Students

- Immediate access to textbook content explanations
- Context-aware responses to specific questions
- Learning progress tracking
- Enhanced engagement with material

### For Instructors

- Tools for testing content accuracy
- Advanced question handling capabilities
- Educational value assessment
- Citations for verification

## Future Enhancements

- Additional textbook modules integration
- Advanced personalization features
- Enhanced natural language understanding
- Multi-language support
