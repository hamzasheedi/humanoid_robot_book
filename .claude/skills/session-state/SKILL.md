---
name: session_state_skill
description: Creates or retrieves a session ID for a user and persists chat history or conversation context in Neon Serverless Postgres. This enables chatbot or agent sessions to maintain context across multiple interactions.
---

# Session State Skill

## Overview

This skill creates or retrieves a session ID for a user and persists chat history or conversation context in Neon Serverless Postgres. This enables chatbot or agent sessions to maintain context across multiple interactions, supporting RAG and chatbot functionality.

## Input

```json
{
  "user_id": "Unique user identifier",
  "session_id": "Optional existing session ID",
  "context": "Optional new message or context to append"
}
```

## Output

```json
{
  "session_id": "Active session ID (new or existing)",
  "context_history": "Full conversation history for the session"
}
```

## Resources

This skill includes resource directories that demonstrate how to organize different types of bundled resources:

### scripts/
Executable code (Python/Bash/etc.) that can be run directly to perform session management and database operations.

### references/
Documentation and reference material for Neon Postgres connection, session management best practices, and database schema specifications.

### assets/
Template files and examples of properly formatted database queries and session data structures.

## Functionality

### Validate Input Parameters
- Ensure user_id is provided and valid
- Validate optional session_id format if provided
- Check that context is properly formatted if provided

### Session Management
- Create a new session if session_id is not provided
- Retrieve existing session from Neon Postgres if session_id is provided
- Generate unique session ID when creating new sessions

### Database Operations
- Connect to Neon Serverless Postgres database
- Execute queries to retrieve or create session data
- Handle database connection pooling and optimization

### Context Management
- Append new context/message to existing conversation history
- Maintain proper ordering of conversation history
- Handle context truncation if needed to manage session size

### Data Persistence
- Persist updated conversation history to Neon Postgres
- Ensure data integrity and consistency
- Handle concurrent access to sessions

### Error Handling
- Handle database connection errors
- Handle session not found errors
- Handle Neon Postgres serverless timeout or unavailable errors
- Manage graceful degradation when database is unavailable

### Response Formatting
- Format responses as structured JSON
- Return active session ID and full conversation history
- Provide clear error messages for debugging