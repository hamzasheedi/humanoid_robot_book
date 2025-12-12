import os
from typing import Optional
from datetime import datetime, timedelta
import jwt
import hashlib
from dotenv import load_dotenv
import aiohttp

from ..models.user import User

load_dotenv()

# For development, using a simpler approach to avoid bcrypt issues on Windows
# In production, should use proper password hashing like bcrypt or argon2

# Get secret key from environment
SECRET_KEY = os.getenv("SECRET_KEY", "fallback_secret_key_for_development")
ALGORITHM = "HS256"
ACCESS_TOKEN_EXPIRE_MINUTES = 30

# Better-Auth API configuration
BACKEND_HOST = os.getenv("BACKEND_HOST", "localhost")
BACKEND_PORT = os.getenv("BACKEND_PORT", "8000")
BETTER_AUTH_URL = os.getenv("BETTER_AUTH_URL", f"http://{BACKEND_HOST}:{BACKEND_PORT}")

class AuthService:
    @staticmethod
    def verify_password(plain_password: str, hashed_password: str) -> bool:
        """Verify a plain password against a hashed password."""
        # For development: simple hash comparison (not secure for production)
        return AuthService.get_password_hash(plain_password) == hashed_password

    @staticmethod
    def get_password_hash(password: str) -> str:
        """Hash a plain password using SHA-256 (for development only)."""
        # For development: simple SHA-256 hash (not secure for production)
        # In production, use proper password hashing like bcrypt or argon2
        return hashlib.sha256(password.encode('utf-8')).hexdigest()

    @staticmethod
    async def create_access_token(data: dict, expires_delta: Optional[timedelta] = None) -> str:
        """Create a JWT access token."""
        to_encode = data.copy()
        if expires_delta:
            expire = datetime.utcnow() + expires_delta
        else:
            expire = datetime.utcnow() + timedelta(minutes=15)
        to_encode.update({"exp": expire})
        encoded_jwt = jwt.encode(to_encode, SECRET_KEY, algorithm=ALGORITHM)
        return encoded_jwt

    @staticmethod
    async def verify_access_token(token: str) -> Optional[dict]:
        """Verify a JWT access token and return the payload."""
        try:
            payload = jwt.decode(token, SECRET_KEY, algorithms=[ALGORITHM])
            return payload
        except jwt.JWTError:
            return None

    @staticmethod
    async def create_user_with_better_auth(email: str, password: str) -> Optional[dict]:
        """
        Create a user via Better-Auth API.
        """
        try:
            async with aiohttp.ClientSession() as session:
                async with session.post(
                    f"{BETTER_AUTH_URL}/api/auth/signup",
                    json={
                        "email": email,
                        "password": password
                    },
                    headers={"Content-Type": "application/json"}
                ) as response:
                    if response.status == 200:
                        result = await response.json()
                        return result
                    else:
                        print(f"Better-Auth registration failed: {response.status}")
                        return None
        except Exception as e:
            print(f"Error calling Better-Auth API: {str(e)}")
            return None

    @staticmethod
    async def authenticate_user_with_better_auth(email: str, password: str) -> Optional[dict]:
        """
        Authenticate a user via Better-Auth API.
        """
        try:
            async with aiohttp.ClientSession() as session:
                async with session.post(
                    f"{BETTER_AUTH_URL}/api/auth/signin",
                    json={
                        "email": email,
                        "password": password
                    },
                    headers={"Content-Type": "application/json"}
                ) as response:
                    if response.status == 200:
                        result = await response.json()
                        return result
                    else:
                        print(f"Better-Auth authentication failed: {response.status}")
                        return None
        except Exception as e:
            print(f"Error calling Better-Auth API: {str(e)}")
            return None

    @staticmethod
    async def create_user_payload(user: User) -> dict:
        """Create a payload for the user JWT token."""
        return {
            "sub": user.id,
            "email": user.email,
            "exp": datetime.utcnow() + timedelta(minutes=ACCESS_TOKEN_EXPIRE_MINUTES)
        }