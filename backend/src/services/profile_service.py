from typing import Optional
from datetime import datetime
import uuid
import asyncpg
from ..models.user import User, UserCreate, UserUpdate
from .auth_service import AuthService
from ..utils.database import db


class ProfileService:
    @staticmethod
    async def create_user_profile(user_data: UserCreate) -> Optional[User]:
        """
        Create a new user profile with hashed password.
        Connects to Neon Postgres to store the user.
        """
        # Hash the password before storing - AuthService handles bcrypt byte limits
        print(f"Original password length: {len(user_data.password)}")
        hashed_password = AuthService.get_password_hash(user_data.password)

        user_id = str(uuid.uuid4())
        now = datetime.utcnow()

        async with db.get_connection() as conn:
            try:
                # Insert the new user into the database
                query = """
                    INSERT INTO users (id, email, hashed_password, os, cpu, gpu, ram_gb,
                                      programming_experience, robotics_experience,
                                      development_environment, primary_language, learning_goals,
                                      created_at, updated_at)
                    VALUES ($1, $2, $3, $4, $5, $6, $7, $8, $9, $10, $11, $12, $13, $14)
                    RETURNING id, email, os, cpu, gpu, ram_gb, programming_experience,
                            robotics_experience, development_environment, primary_language,
                            learning_goals, created_at, updated_at
                """

                # Handle learning_goals properly - convert None to empty array for PostgreSQL
                learning_goals = user_data.learning_goals if user_data.learning_goals is not None else []

                result = await conn.fetchrow(
                    query,
                    user_id,
                    user_data.email,
                    hashed_password,
                    user_data.os,
                    user_data.cpu,
                    user_data.gpu,
                    user_data.ram_gb,
                    user_data.programming_experience,
                    user_data.robotics_experience,
                    user_data.development_environment,
                    user_data.primary_language,
                    learning_goals,
                    now,
                    now
                )

                if result:
                    return User(
                        id=result['id'],
                        email=result['email'],
                        created_at=result['created_at'],
                        updated_at=result['updated_at'],
                        os=result['os'],
                        cpu=result['cpu'],
                        gpu=result['gpu'],
                        ram_gb=result['ram_gb'],
                        programming_experience=result['programming_experience'],
                        robotics_experience=result['robotics_experience'],
                        development_environment=result['development_environment'],
                        primary_language=result['primary_language'],
                        learning_goals=result['learning_goals']
                    )
                return None
            except asyncpg.UniqueViolationError:
                # Email already exists
                return None
            except Exception as e:
                print(f"Error creating user profile: {e}")
                return None

    @staticmethod
    async def get_user_profile(user_id: str) -> Optional[User]:
        """
        Retrieve a user profile by ID from Neon Postgres.
        """
        async with db.get_connection() as conn:
            try:
                query = """
                    SELECT id, email, os, cpu, gpu, ram_gb, programming_experience,
                           robotics_experience, development_environment, primary_language,
                           learning_goals, created_at, updated_at
                    FROM users
                    WHERE id = $1
                """
                result = await conn.fetchrow(query, user_id)

                if result:
                    return User(
                        id=result['id'],
                        email=result['email'],
                        created_at=result['created_at'],
                        updated_at=result['updated_at'],
                        os=result['os'],
                        cpu=result['cpu'],
                        gpu=result['gpu'],
                        ram_gb=result['ram_gb'],
                        programming_experience=result['programming_experience'],
                        robotics_experience=result['robotics_experience'],
                        development_environment=result['development_environment'],
                        primary_language=result['primary_language'],
                        learning_goals=result['learning_goals']
                    )
                return None
            except Exception as e:
                print(f"Error getting user profile: {e}")
                return None

    @staticmethod
    async def get_user_by_email(email: str) -> Optional[User]:
        """
        Retrieve a user profile by email from Neon Postgres.
        """
        async with db.get_connection() as conn:
            try:
                query = """
                    SELECT id, email, os, cpu, gpu, ram_gb, programming_experience,
                           robotics_experience, development_environment, primary_language,
                           learning_goals, created_at, updated_at
                    FROM users
                    WHERE email = $1
                """
                result = await conn.fetchrow(query, email)

                if result:
                    return User(
                        id=result['id'],
                        email=result['email'],
                        created_at=result['created_at'],
                        updated_at=result['updated_at'],
                        os=result['os'],
                        cpu=result['cpu'],
                        gpu=result['gpu'],
                        ram_gb=result['ram_gb'],
                        programming_experience=result['programming_experience'],
                        robotics_experience=result['robotics_experience'],
                        development_environment=result['development_environment'],
                        primary_language=result['primary_language'],
                        learning_goals=result['learning_goals']
                    )
                return None
            except Exception as e:
                print(f"Error getting user by email: {e}")
                return None

    @staticmethod
    async def update_user_profile(user_id: str, update_data: UserUpdate) -> Optional[User]:
        """
        Update a user profile in Neon Postgres.
        """
        async with db.get_connection() as conn:
            try:
                # Build dynamic query based on provided fields
                set_parts = []
                values = [user_id]  # First value is always user_id
                param_index = 2  # Start from $2 since $1 is user_id

                if update_data.os is not None:
                    set_parts.append(f"os = ${param_index}")
                    values.append(update_data.os)
                    param_index += 1

                if update_data.cpu is not None:
                    set_parts.append(f"cpu = ${param_index}")
                    values.append(update_data.cpu)
                    param_index += 1

                if update_data.gpu is not None:
                    set_parts.append(f"gpu = ${param_index}")
                    values.append(update_data.gpu)
                    param_index += 1

                if update_data.ram_gb is not None:
                    set_parts.append(f"ram_gb = ${param_index}")
                    values.append(update_data.ram_gb)
                    param_index += 1

                if update_data.programming_experience is not None:
                    set_parts.append(f"programming_experience = ${param_index}")
                    values.append(update_data.programming_experience)
                    param_index += 1

                if update_data.robotics_experience is not None:
                    set_parts.append(f"robotics_experience = ${param_index}")
                    values.append(update_data.robotics_experience)
                    param_index += 1

                if update_data.development_environment is not None:
                    set_parts.append(f"development_environment = ${param_index}")
                    values.append(update_data.development_environment)
                    param_index += 1

                if update_data.primary_language is not None:
                    set_parts.append(f"primary_language = ${param_index}")
                    values.append(update_data.primary_language)
                    param_index += 1

                if update_data.learning_goals is not None:
                    # Handle learning_goals properly for PostgreSQL
                    learning_goals = update_data.learning_goals
                    set_parts.append(f"learning_goals = ${param_index}")
                    values.append(learning_goals)
                    param_index += 1

                # Always update the updated_at timestamp
                set_parts.append(f"updated_at = ${param_index}")
                values.append(datetime.utcnow())

                if not set_parts:
                    # Nothing to update except timestamp
                    set_parts.append(f"updated_at = ${param_index}")

                set_clause = ", ".join(set_parts)
                query = f"""
                    UPDATE users
                    SET {set_clause}
                    WHERE id = $1
                    RETURNING id, email, os, cpu, gpu, ram_gb, programming_experience,
                              robotics_experience, development_environment, primary_language,
                              learning_goals, created_at, updated_at
                """

                result = await conn.fetchrow(query, *values)

                if result:
                    return User(
                        id=result['id'],
                        email=result['email'],
                        created_at=result['created_at'],
                        updated_at=result['updated_at'],
                        os=result['os'],
                        cpu=result['cpu'],
                        gpu=result['gpu'],
                        ram_gb=result['ram_gb'],
                        programming_experience=result['programming_experience'],
                        robotics_experience=result['robotics_experience'],
                        development_environment=result['development_environment'],
                        primary_language=result['primary_language'],
                        learning_goals=result['learning_goals']
                    )
                return None
            except Exception as e:
                print(f"Error updating user profile: {e}")
                return None