from fastapi import FastAPI, Depends
from fastapi.middleware.cors import CORSMiddleware
from contextlib import asynccontextmanager
import logging

# Import services and config
from ..services.qdrant_service import QdrantService
from ..services.embedding_service import EmbeddingService
from ..services.rag_service import RAGService
from ..services.postgres_service import PostgresService
from ..utils.database import db
from ..config import API_KEY

# Initialize services
qdrant_service = QdrantService()
embedding_service = EmbeddingService(qdrant_service)
postgres_service = PostgresService()
rag_service = RAGService(qdrant_service, embedding_service, postgres_service)

@asynccontextmanager
async def lifespan(app: FastAPI):
    """Lifespan manager to handle startup and shutdown events"""
    # Startup
    await db.connect()
    await qdrant_service.connect()
    await qdrant_service.setup_collection()
    yield
    # Shutdown
    await db.disconnect()

# Create FastAPI app with lifespan
app = FastAPI(lifespan=lifespan)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, replace with specific origins
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# API key validation dependency
def validate_api_key(api_key: str = Depends(lambda: API_KEY)):
    # In a real implementation, you would validate the API key from the request
    # This is a simplified version for the example
    pass

# Include API routes
from .chat import router as chat_router
from .textbook import router as textbook_router

app.include_router(chat_router, prefix="/chat", tags=["chat"])
app.include_router(textbook_router, prefix="/textbook", tags=["textbook"])

# Health check endpoint
@app.get("/health")
async def health_check():
    return {"status": "ok", "message": "RAG Chatbot API is running"}

# Root endpoint
@app.get("/")
async def root():
    return {"message": "Welcome to the RAG Chatbot API for Physical AI & Humanoid Robotics Textbook"}