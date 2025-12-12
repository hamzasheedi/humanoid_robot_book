import os
from dotenv import load_dotenv
import logging

# Load environment variables
load_dotenv()

# Configuration settings
QDRANT_URL = os.getenv("QDRANT_URL", "http://localhost:6333")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
NEON_DB_URL = os.getenv("DATABASE_URL")  # Changed from NEON_DB_URL to DATABASE_URL to match .env file
COHERE_API_KEY = os.getenv("COHERE_API_KEY")
API_KEY = os.getenv("API_KEY")

# Additional configuration settings
DEBUG = os.getenv("DEBUG", "False").lower() == "true"
LOG_LEVEL = os.getenv("LOG_LEVEL", "INFO")
MAX_CONTENT_LENGTH = int(os.getenv("MAX_CONTENT_LENGTH", "2097152"))  # 2MB in bytes
ALLOWED_ORIGINS = os.getenv("ALLOWED_ORIGINS", "*").split(",")

# Configuration validation
def validate_config():
    required_vars = ["COHERE_API_KEY", "QDRANT_URL", "DATABASE_URL", "API_KEY"]  # Changed from NEON_DB_URL to DATABASE_URL
    missing_vars = [var for var in required_vars if not os.getenv(var)]

    if missing_vars:
        raise ValueError(f"Missing required environment variables: {', '.join(missing_vars)}")

    # Validate log level
    valid_log_levels = ["DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"]
    if LOG_LEVEL not in valid_log_levels:
        raise ValueError(f"Invalid LOG_LEVEL: {LOG_LEVEL}. Must be one of {valid_log_levels}")

    # Set logging level
    logging.basicConfig(level=getattr(logging, LOG_LEVEL))

    print("Configuration validated successfully")

if __name__ == "__main__":
    validate_config()