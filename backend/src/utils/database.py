import asyncpg
from typing import Optional
from contextlib import asynccontextmanager
from .config import NEON_DB_URL

class Database:
    def __init__(self):
        self.pool: Optional[asyncpg.Pool] = None
    
    async def connect(self):
        """Initialize database connection pool"""
        try:
            self.pool = await asyncpg.create_pool(
                NEON_DB_URL,
                min_size=5,
                max_size=20,
                command_timeout=60
            )
            print("Database connection pool created successfully")
        except Exception as e:
            print(f"Error creating database connection pool: {e}")
            raise
    
    async def disconnect(self):
        """Close database connection pool"""
        if self.pool:
            await self.pool.close()
            print("Database connection pool closed")
    
    @asynccontextmanager
    async def get_connection(self):
        """Get a connection from the pool"""
        if not self.pool:
            raise RuntimeError("Database not connected")
        
        conn = await self.pool.acquire()
        try:
            yield conn
        finally:
            await self.pool.release(conn)

# Global database instance
db = Database()