import os
import logging
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

app = FastAPI(
    title="Physical AI & Humanoid Robotics Textbook API",
    description="API for textbook platform with authentication and personalization",
    version="1.0.0"
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, replace with specific origins
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

@app.get("/")
async def root():
    return {"message": "Physical AI & Humanoid Robotics Textbook API"}

@app.get("/health")
async def health_check():
    return {"status": "healthy", "service": "backend-api"}

# Include API routes
from .api import auth, profile, personalization, chat, textbook

app.include_router(auth.router, prefix="/api/auth", tags=["authentication"])
app.include_router(profile.router, prefix="/api/profile", tags=["profile"])
app.include_router(personalization.router, prefix="/api/personalization", tags=["personalization"])
app.include_router(chat.router, prefix="/chat", tags=["chat"])
app.include_router(textbook.router, prefix="/textbook", tags=["textbook"])

from .utils.database import db
from .services.qdrant_service import QdrantService
from .services.embedding_service import EmbeddingService
from .services.postgres_service import PostgresService
from .services.rag_service import RAGService

# Initialize services
qdrant_service = QdrantService()
postgres_service = PostgresService()
embedding_service = EmbeddingService(qdrant_service)
rag_service = RAGService(qdrant_service, embedding_service, postgres_service)

# Event handlers
@app.on_event("startup")
async def startup_event():
    logger.info("Starting up the authentication service...")
    # Initialize database connections
    try:
        await db.connect()
        logger.info("Database connected successfully")
    except Exception as e:
        logger.error(f"Database connection failed: {e}. Some features may be unavailable.")

    # Initialize Qdrant connections
    try:
        await qdrant_service.connect()
        if qdrant_service.connected:
            await qdrant_service.setup_collection()
            logger.info("Qdrant connected and collection setup successfully")
        else:
            logger.error("Qdrant connection failed. Some features may be unavailable.")
    except Exception as e:
        logger.error(f"Qdrant setup failed: {e}. Some features may be unavailable.")

@app.on_event("shutdown")
async def shutdown_event():
    logger.info("Shutting down the authentication service...")
    # Clean up resources
    await db.disconnect()