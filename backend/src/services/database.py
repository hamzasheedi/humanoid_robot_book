from sqlalchemy import create_engine
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import sessionmaker
from neon_pool import ConnectionPool
import os
from typing import AsyncGenerator

# Get database URL from environment
DATABASE_URL = os.getenv("DATABASE_URL", "postgresql://username:password@localhost:5432/humanoid_robot_book")

# For Neon Postgres connection pooling
pool = ConnectionPool(DATABASE_URL)

# SQLAlchemy setup (for ORM models if needed)
engine = create_engine(DATABASE_URL)
SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)
Base = declarative_base()

async def get_db_connection():
    """
    Get a database connection from the pool for Neon Postgres.
    """
    conn = await pool.acquire()
    try:
        yield conn
    finally:
        await pool.release(conn)

def get_sync_db():
    """
    Get synchronous database session for SQLAlchemy models.
    """
    db = SessionLocal()
    try:
        yield db
    finally:
        db.close()