---
sidebar_position: 1
title: Interactive Chatbot
---

# Interactive Chatbot for the Physical AI & Humanoid Robotics Textbook

Our textbook features an integrated AI chatbot that allows students to ask questions and receive context-aware answers based on the textbook content.

## Features

- **Context-Aware Q&A**: Ask questions about any topic covered in the textbook
- **Text Selection**: Select text and ask for clarification on specific passages
- **Citation References**: Get sources for answers back to specific sections
- **Conversation History**: Track your learning journey
- **Instructor Dashboard**: Tools for educators to monitor student progress

## How to Use

### Asking Questions
1. Click the floating chat icon in the bottom-right corner
2. Type your question about the textbook content
3. Press Enter or click "Send"

### Using Text Selection
1. Highlight any text in the textbook
2. Click the "Ask About Selected Text" button that appears
3. The AI will provide context-aware answers about the selected text

### Viewing History
1. Open the chat panel
2. Browse your previous questions and answers
3. Use search to find specific conversations

## Technical Details

The chatbot uses Retrieval-Augmented Generation (RAG) to ensure all answers are grounded in the textbook content. It leverages:

- **FastAPI backend** for processing
- **Qdrant vector database** for content retrieval
- **OpenAI models** for answer generation
- **Neon PostgreSQL** for storing conversation history

All responses are limited to information contained in the textbook and properly cited.