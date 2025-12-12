import asyncpg
from typing import List, Optional, Dict
from datetime import datetime
import uuid
import json
from ..utils.database import db
from ..models import Question, Answer, ChatSession, UserInteractionLog

class PostgresService:
    def __init__(self):
        pass
    
    async def save_question(self, question: Question) -> str:
        """Save a question to the database"""
        try:
            async with db.get_connection() as conn:
                query = """
                    INSERT INTO questions (id, content, source_context, user_id, session_id, timestamp, metadata)
                    VALUES ($1, $2, $3, $4, $5, $6, $7)
                    RETURNING id
                """
                result = await conn.fetchval(
                    query,
                    question.id,
                    question.content,
                    question.source_context,
                    question.user_id,
                    question.session_id,
                    question.timestamp,
                    json.dumps(question.metadata) if question.metadata else '{}'
                )
                return result
        except Exception as e:
            print(f"Failed to save question to database: {e}")
            # Return a default value or handle the error gracefully
            return str(question.id) if question.id else str(uuid.uuid4())

    async def save_answer(self, answer: Answer) -> str:
        """Save an answer to the database"""
        try:
            async with db.get_connection() as conn:
                query = """
                    INSERT INTO answers (id, question_id, content, sources, confidence_score, timestamp, metadata)
                    VALUES ($1, $2, $3, $4, $5, $6, $7)
                    RETURNING id
                """
                result = await conn.fetchval(
                    query,
                    answer.id,
                    answer.question_id,
                    answer.content,
                    json.dumps(answer.sources) if answer.sources else '[]',
                    answer.confidence_score,
                    answer.timestamp,
                    json.dumps(answer.metadata) if answer.metadata else '{}'
                )
                return result
        except Exception as e:
            print(f"Failed to save answer to database: {e}")
            # Return a default value or handle the error gracefully
            return str(answer.id) if answer.id else str(uuid.uuid4())
    
    async def create_session(self, user_id: Optional[uuid.UUID] = None) -> str:
        """Create a new chat session"""
        session_id = uuid.uuid4()
        try:
            async with db.get_connection() as conn:
                query = """
                    INSERT INTO chat_sessions (id, user_id, session_start, session_end, interaction_count, metadata)
                    VALUES ($1, $2, $3, $4, $5, $6)
                    RETURNING id
                """
                result = await conn.fetchval(
                    query,
                    session_id,
                    user_id,
                    datetime.utcnow(),
                    None,  # session_end is null initially
                    0,     # interaction_count starts at 0
                    {}     # empty metadata initially
                )
                return result
        except Exception as e:
            print(f"Failed to create session in database: {e}")
            # Return a session ID even if database save fails
            return str(session_id)
    
    async def get_session(self, session_id: str) -> Optional[ChatSession]:
        """Get a chat session by ID"""
        try:
            async with db.get_connection() as conn:
                query = """
                    SELECT id, user_id, session_start, session_end, interaction_count, metadata
                    FROM chat_sessions
                    WHERE id = $1
                """
                row = await conn.fetchrow(query, session_id)
                if row:
                    return ChatSession(
                        id=row['id'],
                        user_id=row['user_id'],
                        session_start=row['session_start'],
                        session_end=row['session_end'],
                        interaction_count=row['interaction_count'],
                        metadata=row['metadata']
                    )
                return None
        except Exception as e:
            print(f"Failed to get session from database: {e}")
            # Return None if database query fails
            return None
    
    async def update_session_interaction_count(self, session_id: str, count: int):
        """Update the interaction count for a session"""
        async with db.get_connection() as conn:
            query = """
                UPDATE chat_sessions
                SET interaction_count = $2
                WHERE id = $1
            """
            await conn.execute(query, session_id, count)
    
    async def get_session_history(self, session_id: str) -> List[Dict]:
        """Get chat history for a specific session"""
        try:
            async with db.get_connection() as conn:
                # First, get questions for the session
                question_query = """
                    SELECT id, content, timestamp
                    FROM questions
                    WHERE session_id = $1
                    ORDER BY timestamp
                """
                questions = await conn.fetch(question_query, session_id)

                history = []
                for question in questions:
                    # Get the corresponding answer for this question
                    answer_query = """
                        SELECT content, sources, timestamp
                        FROM answers
                        WHERE question_id = $1
                    """
                    answer = await conn.fetchrow(answer_query, question['id'])

                    history_item = {
                        'question': question['content'],
                        'answer': answer['content'] if answer else "No answer found",
                        'timestamp': question['timestamp'],
                        'sources': answer['sources'] if answer else []
                    }
                    history.append(history_item)

                return history
        except Exception as e:
            print(f"Failed to get session history from database: {e}")
            # Return empty history if database query fails
            return []

    async def update_session(self, session_id: str, **kwargs):
        """Update session properties like end time, interaction count, etc."""
        async with db.get_connection() as conn:
            # Build dynamic query based on provided fields
            set_parts = []
            values = [session_id]  # First value is always session_id
            param_index = 2  # Start from $2 since $1 is session_id

            if 'session_end' in kwargs:
                set_parts.append(f"session_end = ${param_index}")
                values.append(kwargs['session_end'])
                param_index += 1

            if 'interaction_count' in kwargs:
                set_parts.append(f"interaction_count = ${param_index}")
                values.append(kwargs['interaction_count'])
                param_index += 1

            if 'metadata' in kwargs:
                set_parts.append(f"metadata = ${param_index}")
                values.append(kwargs['metadata'])
                param_index += 1

            if not set_parts:
                return  # Nothing to update

            set_clause = ", ".join(set_parts)
            query = f"""
                UPDATE chat_sessions
                SET {set_clause}
                WHERE id = $1
            """
            await conn.execute(query, *values)

    async def get_all_sessions_for_user(self, user_id: str) -> List[Dict]:
        """Get all sessions for a specific user"""
        async with db.get_connection() as conn:
            query = """
                SELECT id, session_start, session_end, interaction_count, metadata
                FROM chat_sessions
                WHERE user_id = $1
                ORDER BY session_start DESC
            """
            rows = await conn.fetch(query, user_id)

            sessions = []
            for row in rows:
                sessions.append({
                    'id': str(row['id']),
                    'session_start': row['session_start'],
                    'session_end': row['session_end'],
                    'interaction_count': row['interaction_count'],
                    'metadata': row['metadata']
                })

            return sessions

    async def delete_session(self, session_id: str):
        """Delete a session and all associated questions and answers"""
        async with db.get_connection() as conn:
            # Begin transaction
            async with conn.transaction():
                # First get all question IDs for this session
                question_ids_query = """
                    SELECT id FROM questions WHERE session_id = $1
                """
                question_rows = await conn.fetch(question_ids_query, session_id)
                question_ids = [str(row['id']) for row in question_rows]

                if question_ids:
                    # Delete all answers associated with these questions
                    question_ids_str = ",".join(f"${i+2}" for i in range(len(question_ids)))
                    delete_answers_query = f"""
                        DELETE FROM answers WHERE question_id = ANY(ARRAY[{question_ids_str}])
                    """
                    await conn.execute(delete_answers_query, session_id, *question_ids)

                # Delete all questions in this session
                delete_questions_query = """
                    DELETE FROM questions WHERE session_id = $1
                """
                await conn.execute(delete_questions_query, session_id)

                # Finally, delete the session itself
                delete_session_query = """
                    DELETE FROM chat_sessions WHERE id = $1
                """
                await conn.execute(delete_session_query, session_id)

    async def get_session_summary(self, session_id: str) -> Dict:
        """Get a summary of a session with statistics"""
        async with db.get_connection() as conn:
            # Get session info
            session_query = """
                SELECT id, user_id, session_start, session_end, interaction_count, metadata
                FROM chat_sessions
                WHERE id = $1
            """
            session_row = await conn.fetchrow(session_query, session_id)

            if not session_row:
                return {}

            # Get question count
            question_count_query = """
                SELECT COUNT(*) as count
                FROM questions
                WHERE session_id = $1
            """
            question_count = await conn.fetchval(question_count_query, session_id)

            # Get answer count
            answer_count_query = """
                SELECT COUNT(*) as count
                FROM answers a
                JOIN questions q ON a.question_id = q.id
                WHERE q.session_id = $1
            """
            answer_count = await conn.fetchval(answer_count_query, session_id)

            return {
                'session_id': str(session_row['id']),
                'user_id': str(session_row['user_id']) if session_row['user_id'] else None,
                'session_start': session_row['session_start'],
                'session_end': session_row['session_end'],
                'interaction_count': session_row['interaction_count'],
                'question_count': question_count,
                'answer_count': answer_count,
                'metadata': session_row['metadata']
            }
    
    async def log_interaction(self, user_interaction_log: UserInteractionLog):
        """Log user interaction for analytics"""
        async with db.get_connection() as conn:
            query = """
                INSERT INTO user_interaction_logs (id, question_id, answer_id, accuracy_rating, useful, feedback_text, timestamp, metadata)
                VALUES ($1, $2, $3, $4, $5, $6, $7, $8)
            """
            await conn.execute(
                query,
                user_interaction_log.id,
                user_interaction_log.question_id,
                user_interaction_log.answer_id,
                user_interaction_log.accuracy_rating,
                user_interaction_log.useful,
                user_interaction_log.feedback_text,
                user_interaction_log.timestamp,
                user_interaction_log.metadata
            )

    async def record_accuracy_feedback(self, question_id: str, answer_id: str, accuracy_rating: int, feedback_text: str = None):
        """Record accuracy feedback for an answer"""
        interaction_log = UserInteractionLog(
            id=uuid.uuid4(),
            question_id=uuid.UUID(question_id),
            answer_id=uuid.UUID(answer_id),
            accuracy_rating=accuracy_rating,
            useful=accuracy_rating >= 4,  # Consider ratings 4-5 as useful
            feedback_text=feedback_text,
            timestamp=datetime.utcnow(),
            metadata={"feedback_type": "accuracy_rating"}
        )
        await self.log_interaction(interaction_log)

    async def get_answer_accuracy_stats(self, answer_id: str) -> Dict:
        """Get accuracy statistics for a specific answer"""
        async with db.get_connection() as conn:
            query = """
                SELECT
                    AVG(accuracy_rating) as avg_rating,
                    COUNT(*) as total_feedback,
                    COUNT(CASE WHEN useful = true THEN 1 END) as positive_feedback
                FROM user_interaction_logs
                WHERE answer_id = $1
            """
            row = await conn.fetchrow(query, uuid.UUID(answer_id))

            if row:
                return {
                    "average_rating": row['avg_rating'],
                    "total_feedback": row['total_feedback'],
                    "positive_feedback_count": row['positive_feedback'],
                    "positive_feedback_percentage": (row['positive_feedback'] / row['total_feedback']) * 100 if row['total_feedback'] > 0 else 0
                }
            return {"average_rating": None, "total_feedback": 0, "positive_feedback_count": 0, "positive_feedback_percentage": 0}

    async def get_question_accuracy_stats(self, question_id: str) -> Dict:
        """Get accuracy statistics for a specific question"""
        async with db.get_connection() as conn:
            query = """
                SELECT
                    AVG(accuracy_rating) as avg_rating,
                    COUNT(*) as total_feedback
                FROM user_interaction_logs
                WHERE question_id = $1
            """
            row = await conn.fetchrow(query, uuid.UUID(question_id))

            if row:
                return {
                    "average_rating": row['avg_rating'],
                    "total_feedback": row['total_feedback']
                }
            return {"average_rating": None, "total_feedback": 0}
    
    async def get_accuracy_stats(self) -> Dict:
        """Get accuracy statistics for the system"""
        async with db.get_connection() as conn:
            # Get total number of interactions
            total_query = "SELECT COUNT(*) FROM user_interaction_logs"
            total_interactions = await conn.fetchval(total_query)
            
            # Get average accuracy rating
            avg_rating_query = "SELECT AVG(accuracy_rating) FROM user_interaction_logs WHERE accuracy_rating IS NOT NULL"
            avg_rating = await conn.fetchval(avg_rating_query)
            
            # Get percentage of useful answers
            useful_query = """
                SELECT 
                    COUNT(*) * 100.0 / (SELECT COUNT(*) FROM user_interaction_logs) 
                FROM user_interaction_logs 
                WHERE useful = true
            """
            useful_percentage = await conn.fetchval(useful_query)
            
            return {
                "total_interactions": total_interactions,
                "average_accuracy_rating": avg_rating,
                "useful_answer_percentage": useful_percentage
            }
    
    async def save_answer_with_question(self, question: Question, answer: Answer) -> str:
        """Save both question and answer, linking them together"""
        try:
            async with db.get_connection() as conn:
                # Begin transaction
                async with conn.transaction():
                    # Save the question and get its ID
                    q_query = """
                        INSERT INTO questions (id, content, source_context, user_id, session_id, timestamp, metadata)
                        VALUES ($1, $2, $3, $4, $5, $6, $7)
                        RETURNING id
                    """
                    question_id = await conn.fetchval(
                        q_query,
                        question.id or uuid.uuid4(),
                        question.content,
                        question.source_context,
                        question.user_id,
                        question.session_id,
                        question.timestamp or datetime.utcnow(),
                        json.dumps(question.metadata) if question.metadata else '{}'
                    )

                    # Save the answer with the question ID
                    a_query = """
                        INSERT INTO answers (id, question_id, content, sources, confidence_score, timestamp, metadata)
                        VALUES ($1, $2, $3, $4, $5, $6, $7)
                        RETURNING id
                    """
                    answer_id = await conn.fetchval(
                        a_query,
                        answer.id or uuid.uuid4(),
                        question_id,
                        answer.content,
                        json.dumps(answer.sources) if answer.sources else '[]',
                        answer.confidence_score,
                        answer.timestamp or datetime.utcnow(),
                        json.dumps(answer.metadata) if answer.metadata else '{}'
                    )

                    # Update session interaction count
                    session_query = """
                        UPDATE chat_sessions
                        SET interaction_count = interaction_count + 1
                        WHERE id = $1
                    """
                    await conn.execute(session_query, question.session_id)

                    return answer_id
        except Exception as e:
            print(f"Failed to save answer with question to database: {e}")
            # Return a default value or handle the error gracefully
            return str(answer.id) if answer.id else str(uuid.uuid4())