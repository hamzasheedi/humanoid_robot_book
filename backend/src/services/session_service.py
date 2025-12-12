from typing import Optional
from datetime import datetime, timedelta
from uuid import uuid4

from ..models.session import Session, SessionCreate


class SessionService:
    @staticmethod
    async def create_session(user_id: str, ip_address: Optional[str] = None, user_agent: Optional[str] = None) -> Session:
        """
        Create a new session for a user with a timeout.
        In a real implementation, this would store in Neon Postgres.
        """
        # Generate a unique session token
        session_token = str(uuid4())

        # Set session to expire after 30 minutes of inactivity
        expires_at = datetime.utcnow() + timedelta(minutes=30)

        # Create session object
        session_data = SessionCreate(
            user_id=user_id,
            session_token=session_token,
            expires_at=expires_at,
            ip_address=ip_address,
            user_agent=user_agent
        )

        # Create the full session object
        session = Session(
            id=str(uuid4()),
            user_id=session_data.user_id,
            session_token=session_data.session_token,
            created_at=datetime.utcnow(),
            updated_at=datetime.utcnow(),
            expires_at=session_data.expires_at,
            is_active=True,
            ip_address=session_data.ip_address,
            user_agent=session_data.user_agent
        )

        # In a real implementation, we would store this in Neon Postgres
        # await database.sessions.insert_one(session.dict())

        return session

    @staticmethod
    async def get_session_by_token(session_token: str) -> Optional[Session]:
        """
        Retrieve a session by its token.
        In a real implementation, this would fetch from Neon Postgres.
        """
        # Placeholder implementation
        # In a real implementation, we would fetch from Neon Postgres:
        # session_data = await database.sessions.find_one({
        #     "session_token": session_token,
        #     "is_active": True,
        #     "expires_at": {"$gt": datetime.utcnow()}
        # })
        # if session_data:
        #     return Session(**session_data)
        # return None
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
        Mark a session as inactive (logout).
        In a real implementation, this would update Neon Postgres.
        """
        # Placeholder implementation
        # In a real implementation, we would update in Neon Postgres:
        # result = await database.sessions.update_one(
        #     {"session_token": session_token},
        #     {"$set": {"is_active": False, "updated_at": datetime.utcnow()}}
        # )
        # return result.modified_count > 0
        return True

    @staticmethod
    async def refresh_session(session_token: str) -> Optional[Session]:
        """
        Extend the expiration time of an active session.
        In a real implementation, this would update Neon Postgres.
        """
        session = await SessionService.get_session_by_token(session_token)
        if session:
            # Extend session by 30 minutes from now
            new_expires_at = datetime.utcnow() + timedelta(minutes=30)

            # Update the session expiration
            session.expires_at = new_expires_at
            session.updated_at = datetime.utcnow()

            # In a real implementation, we would update in Neon Postgres:
            # await database.sessions.update_one(
            #     {"id": session.id},
            #     {"$set": {"expires_at": new_expires_at, "updated_at": datetime.utcnow()}}
            # )

            return session
        return None