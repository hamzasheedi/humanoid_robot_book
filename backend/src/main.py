from fastapi import FastAPI
from dotenv import load_dotenv
import os

# Load environment variables
load_dotenv()

app = FastAPI(
    title="Physical AI & Humanoid Robotics Textbook API",
    description="API for textbook platform with authentication and personalization",
    version="1.0.0"
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