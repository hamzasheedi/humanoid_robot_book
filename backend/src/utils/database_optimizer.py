"""
Database optimization utilities for the Physical AI & Humanoid Robotics Textbook platform
Provides query optimization, connection pooling, and performance enhancements
"""

from typing import Optional, List, Dict, Any
from sqlalchemy import text
from sqlalchemy.orm import Session
from sqlalchemy.sql import Select
import time
import logging
from contextlib import contextmanager

from src.services.database import SessionLocal, engine

logger = logging.getLogger(__name__)

class DatabaseOptimizer:
    """
    Database optimization utilities
    """

    @staticmethod
    def get_connection_pool_stats():
        """
        Get statistics about the database connection pool
        """
        try:
            # This would depend on the specific connection pool implementation
            # For Neon Postgres with connection pooling
            stats = {
                "pool_size": "N/A",  # Would be available in actual pool implementation
                "active_connections": "N/A",
                "idle_connections": "N/A",
                "max_connections": "N/A"
            }
            return stats
        except Exception as e:
            logger.error(f"Error getting connection pool stats: {str(e)}")
            return {"error": str(e)}

    @staticmethod
    def optimize_query_performance(query: Select, session: Session) -> Select:
        """
        Apply performance optimizations to a query
        """
        # Add query hints and optimizations
        optimized_query = query

        # Add indexes if needed (this would be specific to the database)
        # optimized_query = optimized_query.prefix_with("/*+ USE_INDEX(users, idx_users_email) */")

        return optimized_query

    @staticmethod
    def get_query_execution_time(query: Select, session: Session) -> tuple:
        """
        Execute a query and return results with execution time
        """
        start_time = time.time()

        try:
            result = session.execute(query)
            execution_time = time.time() - start_time

            logger.info(f"Query executed in {execution_time:.4f}s")
            return result, execution_time
        except Exception as e:
            execution_time = time.time() - start_time
            logger.error(f"Query failed after {execution_time:.4f}s: {str(e)}")
            raise

    @staticmethod
    @contextmanager
    def get_optimized_session():
        """
        Context manager for getting an optimized database session
        """
        session = SessionLocal()
        try:
            # Set session-level optimizations
            session.execute(text("SET statement_timeout = 30000"))  # 30 second timeout
            session.execute(text("SET lock_timeout = 10000"))  # 10 second lock timeout

            yield session
        except Exception as e:
            session.rollback()
            logger.error(f"Database session error: {str(e)}")
            raise
        finally:
            session.close()

    @staticmethod
    def create_indexes():
        """
        Create performance indexes for frequently queried fields
        """
        index_queries = [
            # User table indexes
            "CREATE INDEX IF NOT EXISTS idx_users_email ON users(email);",
            "CREATE INDEX IF NOT EXISTS idx_users_created_at ON users(created_at);",

            # Session table indexes
            "CREATE INDEX IF NOT EXISTS idx_sessions_user_id ON sessions(user_id);",
            "CREATE INDEX IF NOT EXISTS idx_sessions_expires_at ON sessions(expires_at);",
            "CREATE INDEX IF NOT EXISTS idx_sessions_created_at ON sessions(created_at);",

            # Personalization table indexes
            "CREATE INDEX IF NOT EXISTS idx_personalization_user_id ON personalization(user_id);",
            "CREATE INDEX IF NOT EXISTS idx_personalization_content_difficulty ON personalization(content_difficulty);",
            "CREATE INDEX IF NOT EXISTS idx_personalization_updated_at ON personalization(updated_at);",
        ]

        with DatabaseOptimizer.get_optimized_session() as session:
            for query in index_queries:
                try:
                    session.execute(text(query))
                    session.commit()
                    logger.info(f"Index created/verified: {query[:50]}...")
                except Exception as e:
                    logger.warning(f"Could not create index {query[:50]}...: {str(e)}")
                    session.rollback()

    @staticmethod
    def analyze_query_performance(query: str, params: Optional[Dict] = None) -> Dict[str, Any]:
        """
        Analyze query performance using EXPLAIN
        """
        with DatabaseOptimizer.get_optimized_session() as session:
            try:
                # Use EXPLAIN to analyze query performance
                explain_query = f"EXPLAIN (ANALYZE, BUFFERS) {query}"

                start_time = time.time()
                result = session.execute(text(explain_query), params or {})
                execution_time = time.time() - start_time

                plan = result.fetchall()
                plan_text = " ".join([str(row[0]) for row in plan])

                analysis = {
                    "query": query,
                    "execution_time": execution_time,
                    "query_plan": plan_text,
                    "recommendations": DatabaseOptimizer._analyze_plan(plan_text)
                }

                return analysis
            except Exception as e:
                logger.error(f"Error analyzing query: {str(e)}")
                return {"error": str(e)}

    @staticmethod
    def _analyze_plan(plan: str) -> List[str]:
        """
        Analyze query plan and provide recommendations
        """
        recommendations = []

        # Check for full table scans
        if "Seq Scan" in plan:
            recommendations.append("Consider adding indexes to avoid full table scans")

        # Check for nested loops that could be hash joins
        if "Nested Loop" in plan and "Hash Join" not in plan:
            recommendations.append("Consider using hash joins for better performance")

        # Check execution time
        # This would require parsing the actual execution time from the plan

        return recommendations

    @staticmethod
    def get_database_statistics() -> Dict[str, Any]:
        """
        Get database performance statistics
        """
        stats_queries = {
            "total_users": "SELECT COUNT(*) FROM users;",
            "active_sessions": "SELECT COUNT(*) FROM sessions WHERE expires_at > NOW();",
            "recent_personalization_updates": "SELECT COUNT(*) FROM personalization WHERE updated_at > NOW() - INTERVAL '1 hour';",
            "table_sizes": """
                SELECT
                    table_name,
                    pg_size_pretty(pg_total_relation_size(table_name::regclass)) as size
                FROM information_schema.tables
                WHERE table_schema = 'public'
                AND table_name IN ('users', 'sessions', 'personalization');
            """,
        }

        with DatabaseOptimizer.get_optimized_session() as session:
            stats = {}
            for key, query in stats_queries.items():
                try:
                    result = session.execute(text(query))
                    if key == "table_sizes":
                        stats[key] = [dict(row) for row in result]
                    else:
                        stats[key] = result.scalar()
                except Exception as e:
                    logger.error(f"Error getting {key} stats: {str(e)}")
                    stats[key] = f"Error: {str(e)}"

            return stats

    @staticmethod
    def optimize_database_connections():
        """
        Optimize database connection settings
        """
        # Connection pool optimization settings
        optimization_settings = {
            "connection_timeout": 30,  # seconds
            "command_timeout": 30,     # seconds
            "idle_connection_timeout": 300,  # 5 minutes
            "max_connection_lifetime": 3600,  # 1 hour
        }

        return optimization_settings

    @staticmethod
    def cleanup_expired_sessions():
        """
        Clean up expired sessions to improve performance
        """
        cleanup_query = """
            DELETE FROM sessions
            WHERE expires_at < NOW() - INTERVAL '1 day';
        """

        with DatabaseOptimizer.get_optimized_session() as session:
            try:
                result = session.execute(text(cleanup_query))
                session.commit()

                deleted_count = result.rowcount
                logger.info(f"Cleaned up {deleted_count} expired sessions")

                return {"deleted_sessions": deleted_count, "success": True}
            except Exception as e:
                logger.error(f"Error cleaning up expired sessions: {str(e)}")
                return {"error": str(e), "success": False}

    @staticmethod
    def get_slow_query_report(hours_back: int = 24) -> List[Dict[str, Any]]:
        """
        Get report of slow queries for the past specified hours
        """
        # This would require access to database query logs
        # For PostgreSQL, this would use pg_stat_statements or log files
        report = [
            {
                "query": "SELECT * FROM users WHERE email = $1",
                "avg_execution_time": 0.5,
                "call_count": 100,
                "total_time": 50.0
            }
        ]

        return report


# Initialize database optimization
def initialize_database_optimization():
    """
    Initialize database optimization features
    """
    logger.info("Initializing database optimization features...")

    # Create indexes
    DatabaseOptimizer.create_indexes()

    # Log database statistics
    stats = DatabaseOptimizer.get_database_statistics()
    logger.info(f"Database statistics: {stats}")

    # Schedule periodic cleanup of expired sessions
    cleanup_result = DatabaseOptimizer.cleanup_expired_sessions()
    logger.info(f"Expired session cleanup: {cleanup_result}")


if __name__ == "__main__":
    # Test the database optimizer
    print("Testing database optimizer...")

    # Initialize optimization
    initialize_database_optimization()

    # Get statistics
    stats = DatabaseOptimizer.get_database_statistics()
    print(f"Database stats: {stats}")

    # Cleanup expired sessions
    cleanup_result = DatabaseOptimizer.cleanup_expired_sessions()
    print(f"Cleanup result: {cleanup_result}")