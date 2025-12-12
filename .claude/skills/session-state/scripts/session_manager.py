#!/usr/bin/env python3
"""
Session State Manager Script

This script manages user sessions and persists chat history in Neon Postgres.
"""

import json
import uuid
from datetime import datetime
from typing import Dict, Any, List, Optional
import psycopg2
from psycopg2.extras import RealDictCursor

class SessionManager:
    def __init__(self, db_url: str):
        """
        Initialize the session manager with database connection.

        Args:
            db_url: Database connection URL for Neon Postgres
        """
        self.db_url = db_url

    def get_or_create_session(self, user_id: str, session_id: Optional[str] = None, context: Optional[str] = None) -> Dict[str, Any]:
        """
        Get existing session or create a new one, optionally appending new context.

        Args:
            user_id: Unique user identifier
            session_id: Optional existing session ID
            context: Optional new message or context to append

        Returns:
            Dictionary with session_id and context_history
        """
        try:
            with psycopg2.connect(self.db_url) as conn:
                with conn.cursor(cursor_factory=RealDictCursor) as cursor:
                    if session_id is None:
                        # Create new session
                        session_id = str(uuid.uuid4())
                        cursor.execute("""
                            INSERT INTO sessions (session_id, user_id, context_history, created_at, updated_at)
                            VALUES (%s, %s, %s, %s, %s)
                        """, (session_id, user_id, [context] if context else [], datetime.utcnow(), datetime.utcnow()))

                        context_history = [context] if context else []
                    else:
                        # Retrieve existing session
                        cursor.execute("""
                            SELECT session_id, user_id, context_history, updated_at
                            FROM sessions
                            WHERE session_id = %s
                        """, (session_id,))

                        result = cursor.fetchone()

                        if result is None:
                            return {
                                "session_id": None,
                                "context_history": [],
                                "error": "Session not found"
                            }

                        if result['user_id'] != user_id:
                            return {
                                "session_id": None,
                                "context_history": [],
                                "error": "User does not have access to this session"
                            }

                        context_history = result['context_history'] if result['context_history'] else []

                        # Append new context if provided
                        if context:
                            context_history.append(context)

                            cursor.execute("""
                                UPDATE sessions
                                SET context_history = %s, updated_at = %s
                                WHERE session_id = %s
                            """, (context_history, datetime.utcnow(), session_id))

                    # Update the session timestamp
                    if context:  # Only update if we added new context
                        cursor.execute("""
                            UPDATE sessions
                            SET updated_at = %s
                            WHERE session_id = %s
                        """, (datetime.utcnow(), session_id))

                    return {
                        "session_id": session_id,
                        "context_history": context_history
                    }

        except psycopg2.OperationalError:
            return {
                "session_id": session_id,
                "context_history": [],
                "error": "Database connection failed"
            }
        except psycopg2.Error as e:
            return {
                "session_id": session_id,
                "context_history": [],
                "error": f"Database error: {str(e)}"
            }
        except Exception as e:
            return {
                "session_id": session_id,
                "context_history": [],
                "error": f"Unexpected error: {str(e)}"
            }

    def create_session_table(self):
        """
        Create the sessions table if it doesn't exist.
        """
        try:
            with psycopg2.connect(self.db_url) as conn:
                with conn.cursor() as cursor:
                    cursor.execute("""
                        CREATE TABLE IF NOT EXISTS sessions (
                            session_id TEXT PRIMARY KEY,
                            user_id TEXT NOT NULL,
                            context_history JSONB,
                            created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                            updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
                        )
                    """)
                    conn.commit()
        except Exception as e:
            print(f"Error creating table: {str(e)}")


def main():
    """
    Example usage of the SessionManager.
    This would typically be called by Claude when executing the skill.
    """
    import os

    # Get database URL from environment or use a default for testing
    db_url = os.getenv("NEON_DB_URL", "postgresql://username:password@localhost:5432/session_db")

    session_manager = SessionManager(db_url)

    # Example usage
    user_id = "user_123"
    session_id = None  # Will create a new session
    context = "Hello, how can I help you today?"

    result = session_manager.get_or_create_session(user_id, session_id, context)
    print(json.dumps(result, indent=2, default=str))


if __name__ == "__main__":
    main()