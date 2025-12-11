# Session State Database Schema

## Overview
The session state skill uses Neon Serverless Postgres to persist chat history and conversation context.

## Sessions Table Schema
```sql
CREATE TABLE sessions (
    session_id TEXT PRIMARY KEY,
    user_id TEXT NOT NULL,
    context_history JSONB,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);
```

## Field Descriptions
- `session_id`: Unique identifier for the session (UUID string)
- `user_id`: Unique identifier for the user
- `context_history`: JSON array containing the conversation history
- `created_at`: Timestamp when the session was created
- `updated_at`: Timestamp when the session was last updated

## Indexes
```sql
CREATE INDEX idx_sessions_user_id ON sessions(user_id);
CREATE INDEX idx_sessions_updated_at ON sessions(updated_at);
```

## Connection Configuration
- Use connection pooling for optimal performance
- Handle serverless timeout by reconnecting when needed
- Use environment variable `NEON_DB_URL` for connection string

## Example Data Structure
```json
{
  "session_id": "a1b2c3d4-e5f6-7890-1234-567890abcdef",
  "user_id": "user_123",
  "context_history": [
    "Hello, how can I help you today?",
    "I'm looking for information about humanoid robotics",
    "Humanoid robotics is a fascinating field..."
  ],
  "created_at": "2023-01-01T00:00:00Z",
  "updated_at": "2023-01-01T00:05:00Z"
}
```

## Error Handling
- Handle connection timeout errors (common with serverless databases)
- Retry logic for failed connections
- Graceful degradation when database is unavailable