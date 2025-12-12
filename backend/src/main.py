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
from .api import auth, profile, personalization

app.include_router(auth.router, prefix="/api/auth", tags=["authentication"])
app.include_router(profile.router, prefix="/api/profile", tags=["profile"])
app.include_router(personalization.router, prefix="/api/personalization", tags=["personalization"])

# Event handlers
@app.on_event("startup")
async def startup_event():
    logger.info("Starting up the authentication service...")
    # Initialize database connections, caches, etc. here

@app.on_event("shutdown")
async def shutdown_event():
    logger.info("Shutting down the authentication service...")
    # Clean up resources here