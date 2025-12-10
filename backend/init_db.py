"""
Database initialization script for the RAG Chatbot
This script creates all required tables in the PostgreSQL database
"""
import asyncpg
import asyncio
import os
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

NEON_DB_URL = os.getenv("NEON_DB_URL")

async def init_db():
    conn = await asyncpg.connect(NEON_DB_URL)
    
    try:
        # Create chat_sessions table
        await conn.execute('''
            CREATE TABLE IF NOT EXISTS chat_sessions (
                id UUID PRIMARY KEY,
                user_id UUID,
                session_start TIMESTAMP NOT NULL,
                session_end TIMESTAMP,
                interaction_count INTEGER DEFAULT 0,
                metadata JSONB
            )
        ''')
        print("+ Created/verified chat_sessions table")

        # Create questions table
        await conn.execute('''
            CREATE TABLE IF NOT EXISTS questions (
                id UUID PRIMARY KEY,
                content TEXT NOT NULL,
                source_context TEXT,
                user_id UUID,
                session_id UUID REFERENCES chat_sessions(id),
                timestamp TIMESTAMP NOT NULL,
                metadata JSONB
            )
        ''')
        print("+ Created/verified questions table")

        # Create answers table
        await conn.execute('''
            CREATE TABLE IF NOT EXISTS answers (
                id UUID PRIMARY KEY,
                question_id UUID REFERENCES questions(id) ON DELETE CASCADE,
                content TEXT NOT NULL,
                sources JSONB,
                confidence_score NUMERIC(3, 2),
                timestamp TIMESTAMP NOT NULL,
                metadata JSONB
            )
        ''')
        print("+ Created/verified answers table")

        # Create user_interaction_logs table
        await conn.execute('''
            CREATE TABLE IF NOT EXISTS user_interaction_logs (
                id UUID PRIMARY KEY,
                question_id UUID REFERENCES questions(id) ON DELETE CASCADE,
                answer_id UUID REFERENCES answers(id) ON DELETE CASCADE,
                accuracy_rating INTEGER CHECK (accuracy_rating >= 1 AND accuracy_rating <= 5),
                useful BOOLEAN,
                feedback_text TEXT,
                timestamp TIMESTAMP NOT NULL,
                metadata JSONB
            )
        ''')
        print("+ Created/verified user_interaction_logs table")

        # Create indexes for performance
        await conn.execute('''
            CREATE INDEX IF NOT EXISTS idx_questions_session_id ON questions(session_id);
        ''')
        print("+ Created index on questions.session_id")

        await conn.execute('''
            CREATE INDEX IF NOT EXISTS idx_questions_timestamp ON questions(timestamp);
        ''')
        print("+ Created index on questions.timestamp")

        await conn.execute('''
            CREATE INDEX IF NOT EXISTS idx_answers_timestamp ON answers(timestamp);
        ''')
        print("+ Created index on answers.timestamp")

        print("\nDatabase tables created successfully!")

    except Exception as e:
        print(f"Error initializing database: {e}")
    finally:
        await conn.close()

if __name__ == "__main__":
    asyncio.run(init_db())