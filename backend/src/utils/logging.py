import logging
from fastapi import HTTPException
from fastapi.responses import JSONResponse
from typing import Dict, Any
import traceback
import sys

# Set up logging configuration
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler('app.log'),
        logging.StreamHandler(sys.stdout)
    ]
)

logger = logging.getLogger(__name__)

# Custom exception classes
class RAGException(Exception):
    """Base exception for RAG-related errors"""
    def __init__(self, message: str, error_code: str = "RAG_ERROR"):
        self.message = message
        self.error_code = error_code
        super().__init__(self.message)

class ContentNotFoundException(RAGException):
    """Exception raised when no relevant content is found"""
    def __init__(self, message: str = "No relevant content found in the textbook"):
        super().__init__(message, "CONTENT_NOT_FOUND")

class DatabaseException(RAGException):
    """Exception raised for database-related errors"""
    def __init__(self, message: str = "Database error occurred"):
        super().__init__(message, "DATABASE_ERROR")

class VectorStoreException(RAGException):
    """Exception raised for vector store-related errors"""
    def __init__(self, message: str = "Vector store error occurred"):
        super().__init__(message, "VECTOR_STORE_ERROR")

class ModelException(RAGException):
    """Exception raised for model/LLM-related errors"""
    def __init__(self, message: str = "Model service error occurred"):
        super().__init__(message, "MODEL_ERROR")

# Exception handlers
async def content_not_found_handler(request, exc: ContentNotFoundException):
    logger.error(f"Content not found: {exc.message}")
    return JSONResponse(
        status_code=404,
        content={
            "error": exc.message,
            "code": exc.error_code
        }
    )

async def database_exception_handler(request, exc: DatabaseException):
    logger.error(f"Database error: {exc.message}")
    return JSONResponse(
        status_code=500,
        content={
            "error": exc.message,
            "code": exc.error_code
        }
    )

async def vector_store_exception_handler(request, exc: VectorStoreException):
    logger.error(f"Vector store error: {exc.message}")
    return JSONResponse(
        status_code=500,
        content={
            "error": exc.message,
            "code": exc.error_code
        }
    )

async def model_exception_handler(request, exc: ModelException):
    logger.error(f"Model error: {exc.message}")
    return JSONResponse(
        status_code=500,
        content={
            "error": exc.message,
            "code": exc.error_code
        }
    )

async def general_exception_handler(request, exc: Exception):
    logger.error(f"Unhandled exception: {str(exc)}")
    logger.error(traceback.format_exc())
    return JSONResponse(
        status_code=500,
        content={
            "error": "An unexpected error occurred",
            "code": "INTERNAL_ERROR",
            "details": str(exc) if __name__ == "__main__" else "Internal server error"
        }
    )

# Utility function for logging requests
def log_request(request, extra_data: Dict[str, Any] = None):
    logger.info(f"Request: {request.method} {request.url}")
    if extra_data:
        logger.info(f"Extra data: {extra_data}")

# Utility function for logging responses
def log_response(response, request_id: str = None):
    # Check if response is an HTTP response object or a dictionary
    if hasattr(response, 'status_code'):
        logger.info(f"Response status: {response.status_code}")
    else:
        # For dictionary responses, just log the content
        logger.info(f"Response content: {response}")
    if request_id:
        logger.info(f"Request ID: {request_id}")

# Utility function for logging errors
def log_error(error: Exception, context: str = ""):
    logger.error(f"Error in {context}: {str(error)}")
    logger.error(traceback.format_exc())