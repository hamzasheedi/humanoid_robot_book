from typing import Optional
from datetime import datetime

from ..models.user import User, UserCreate, UserUpdate
from .auth_service import AuthService


class ProfileService:
    @staticmethod
    async def create_user_profile(user_data: UserCreate) -> Optional[User]:
        """
        Create a new user profile with hashed password.
        In a real implementation, this would connect to Neon Postgres.
        """
        # This is a placeholder implementation - in real app would connect to Neon Postgres
        from uuid import uuid4

        # Hash the password before storing
        hashed_password = AuthService.get_password_hash(user_data.password)

        # Create a new user with generated ID and timestamps
        new_user = User(
            id=str(uuid4()),
            email=user_data.email,
            created_at=datetime.utcnow(),
            updated_at=datetime.utcnow(),
            os=user_data.os,
            cpu=user_data.cpu,
            gpu=user_data.gpu,
            ram_gb=user_data.ram_gb,
            programming_experience=user_data.programming_experience,
            robotics_experience=user_data.robotics_experience,
            development_environment=user_data.development_environment,
            primary_language=user_data.primary_language,
            learning_goals=user_data.learning_goals
        )

        # In a real implementation, we would store this in Neon Postgres
        # await database.users.insert_one(new_user.dict())

        return new_user

    @staticmethod
    async def get_user_profile(user_id: str) -> Optional[User]:
        """
        Retrieve a user profile by ID.
        In a real implementation, this would fetch from Neon Postgres.
        """
        # Placeholder implementation
        # In a real implementation, we would fetch from Neon Postgres:
        # user_data = await database.users.find_one({"id": user_id})
        # if user_data:
        #     return User(**user_data)
        # return None
        return None

    @staticmethod
    async def update_user_profile(user_id: str, update_data: UserUpdate) -> Optional[User]:
        """
        Update a user profile.
        In a real implementation, this would update in Neon Postgres.
        """
        # Placeholder implementation
        # In a real implementation, we would update in Neon Postgres:
        # update_data_dict = update_data.dict(exclude_unset=True)
        # update_data_dict["updated_at"] = datetime.utcnow()
        # result = await database.users.update_one(
        #     {"id": user_id},
        #     {"$set": update_data_dict}
        # )
        # if result.modified_count > 0:
        #     return await cls.get_user_profile(user_id)
        # return None
        return None