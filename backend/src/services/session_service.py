from typing import Optional
from datetime import datetime, timedelta
import uuid
import asyncpg
from ..models.session import Session, SessionCreate
from ..utils.database import db


class SessionService:
    @staticmethod
    async def create_session(user_id: str, ip_address: Optional[str] = None, user_agent: Optional[str] = None) -> Optional[Session]:
        """
        Create a new session for a user with a timeout in Neon Postgres.
        """
        session_id = str(uuid.uuid4())
        session_token = str(uuid.uuid4())
        now = datetime.utcnow()
        expires_at = now + timedelta(minutes=30)  # 30 minutes timeout

        async with db.get_connection() as conn:
            try:
                query = """
                    INSERT INTO sessions (id, user_id, session_token, expires_at, created_at, updated_at, ip_address, user_agent)
                    VALUES ($1, $2, $3, $4, $5, $6, $7, $8)
                    RETURNING id, user_id, session_token, created_at, updated_at, expires_at, is_active, ip_address, user_agent
                """
                result = await conn.fetchrow(
                    query,
                    session_id,
                    user_id,
                    session_token,
                    expires_at,
                    now,
                    now,
                    ip_address,
                    user_agent
                )

                if result:
                    return Session(
                        id=result['id'],
                        user_id=result['user_id'],
                        session_token=result['session_token'],
                        created_at=result['created_at'],
                        updated_at=result['updated_at'],
                        expires_at=result['expires_at'],
                        is_active=result['is_active'],
                        ip_address=result['ip_address'],
                        user_agent=result['user_agent']
                    )
                return None
            except Exception as e:
                print(f"Error creating session: {e}")
                return None

    @staticmethod
    async def get_session_by_token(session_token: str) -> Optional[Session]:
        """
        Retrieve a session by its token from Neon Postgres.
        """
        async with db.get_connection() as conn:
            try:
                query = """
                    SELECT id, user_id, session_token, created_at, updated_at, expires_at, is_active, ip_address, user_agent
                    FROM sessions
                    WHERE session_token = $1 AND is_active = TRUE AND expires_at > $2
                """
                result = await conn.fetchrow(query, session_token, datetime.utcnow())

                if result:
                    return Session(
                        id=result['id'],
                        user_id=result['user_id'],
                        session_token=result['session_token'],
                        created_at=result['created_at'],
                        updated_at=result['updated_at'],
                        expires_at=result['expires_at'],
                        is_active=result['is_active'],
                        ip_address=result['ip_address'],
                        user_agent=result['user_agent']
                    )
                return None
            except Exception as e:
                print(f"Error getting session by token: {e}")
                return None

    @staticmethod
    async def validate_session(session_token: str) -> bool:
        """
        Validate if a session is still active and not expired.
        """
        session = await SessionService.get_session_by_token(session_token)
        if session and session.is_active and session.expires_at > datetime.utcnow():
            return True
        return False

    @staticmethod
    async def invalidate_session(session_token: str) -> bool:
        """
        Mark a session as inactive (logout) in Neon Postgres.
        """
        async with db.get_connection() as conn:
            try:
                query = """
                    UPDATE sessions
                    SET is_active = FALSE, updated_at = $2
                    WHERE session_token = $1
                    RETURNING id
                """
                result = await conn.fetchval(query, session_token, datetime.utcnow())
                return result is not None
            except Exception as e:
                print(f"Error invalidating session: {e}")
                return False

    @staticmethod
    async def refresh_session(session_token: str) -> Optional[Session]:
        """
        Extend the expiration time of an active session in Neon Postgres.
        """
        async with db.get_connection() as conn:
            try:
                # First, get the current session
                current_session = await SessionService.get_session_by_token(session_token)
                if not current_session:
                    return None

                # Extend session by 30 minutes from now
                new_expires_at = datetime.utcnow() + timedelta(minutes=30)
                now = datetime.utcnow()

                # Update the session in the database
                query = """
                    UPDATE sessions
                    SET expires_at = $2, updated_at = $3
                    WHERE session_token = $1
                    RETURNING id, user_id, session_token, created_at, updated_at, expires_at, is_active, ip_address, user_agent
                """
                result = await conn.fetchrow(query, session_token, new_expires_at, now)

                if result:
                    return Session(
                        id=result['id'],
                        user_id=result['user_id'],
                        session_token=result['session_token'],
                        created_at=result['created_at'],
                        updated_at=result['updated_at'],
                        expires_at=result['expires_at'],
                        is_active=result['is_active'],
                        ip_address=result['ip_address'],
                        user_agent=result['user_agent']
                    )
                return None
            except Exception as e:
                print(f"Error refreshing session: {e}")
                return None