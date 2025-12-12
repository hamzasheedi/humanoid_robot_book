from fastapi import APIRouter, HTTPException, Depends, Request
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials
from typing import Optional
from pydantic import BaseModel

from ..models.user import UserCreate, User
from ..services.auth_service import AuthService
from ..services.profile_service import ProfileService
from ..services.session_service import SessionService

router = APIRouter()
security = HTTPBearer()


@router.post("/signup", response_model=dict)
async def signup(request: Request, user_data: UserCreate):
    """
    Create a new user account with optional profile information.
    """
    try:
        print(f"Received signup request with data: {user_data.dict()}")

        # Create user profile with hashed password
        user = await ProfileService.create_user_profile(user_data)

        if not user:
            raise HTTPException(status_code=400, detail="Failed to create user account - email may already exist")

        # Create a session for the new user
        session = await SessionService.create_session(
            user_id=user.id,
            ip_address=request.client.host if request.client else None,
            user_agent=request.headers.get('user-agent')
        )

        if not session:
            raise HTTPException(status_code=500, detail="Failed to create session for user")

        # Return success response with user and session info
        return {
            "success": True,
            "message": "User created successfully",
            "user_id": user.id,
            "auth_token": session.session_token,
            "user": {
                "id": user.id,
                "email": user.email,
                "os": user.os,
                "cpu": user.cpu,
                "gpu": user.gpu,
                "ram_gb": user.ram_gb,
                "programming_experience": user.programming_experience,
                "robotics_experience": user.robotics_experience,
                "development_environment": user.development_environment,
                "primary_language": user.primary_language,
                "learning_goals": user.learning_goals,
                "created_at": user.created_at,
                "updated_at": user.updated_at
            }
        }
    except Exception as e:
        print(f"SIGNUP ERROR: {str(e)}")
        import traceback
        traceback.print_exc()
        raise HTTPException(status_code=400, detail=f"Signup failed: {str(e)}")


class SigninRequest(BaseModel):
    email: str
    password: str


@router.post("/signin", response_model=dict)
async def signin(request: Request, signin_data: SigninRequest):
    """
    Authenticate a user and create a session.
    Expected request body: {"email": "user@example.com", "password": "user_password"}
    """
    try:
        email = signin_data.email
        password = signin_data.password

        # Validate email and password
        if not email or not password:
            raise HTTPException(status_code=400, detail="Email and password are required")

        # Look up the user by email in Neon Postgres
        user = await ProfileService.get_user_by_email(email)
        if not user:
            raise HTTPException(status_code=401, detail="Invalid credentials")

        # Verify the password using AuthService.verify_password
        # For this to work, we need to get the hashed password from the database
        # Let's create a method to get the full user record including password

        # Get user with password from database (we'll need to add this method)
        # For now, let's assume the password verification works
        from ..utils.database import db
        import asyncpg

        async with db.get_connection() as conn:
            query = """
                SELECT id, email, hashed_password
                FROM users
                WHERE email = $1
            """
            user_record = await conn.fetchrow(query, email)

            if not user_record:
                raise HTTPException(status_code=401, detail="Invalid credentials")

            # Verify the password
            if not AuthService.verify_password(password, user_record['hashed_password']):
                raise HTTPException(status_code=401, detail="Invalid credentials")

        # Create a session for the user
        session = await SessionService.create_session(
            user_id=user_record['id'],
            ip_address=request.client.host if request.client else None,
            user_agent=request.headers.get('user-agent')
        )

        if not session:
            raise HTTPException(status_code=500, detail="Failed to create session")

        # For the response, we need the full user data (not just the one from get_user_by_email)
        # Get the full user record with all profile information
        full_user = await ProfileService.get_user_profile(user_record['id'])
        if not full_user:
            raise HTTPException(status_code=500, detail="Failed to retrieve user profile after authentication")

        # Return success response with user and session info
        return {
            "success": True,
            "message": "Signin successful",
            "user_id": user_record['id'],
            "auth_token": session.session_token,
            "user": {
                "id": full_user.id,
                "email": full_user.email,
                "os": full_user.os,
                "cpu": full_user.cpu,
                "gpu": full_user.gpu,
                "ram_gb": full_user.ram_gb,
                "programming_experience": full_user.programming_experience,
                "robotics_experience": full_user.robotics_experience,
                "development_environment": full_user.development_environment,
                "primary_language": full_user.primary_language,
                "learning_goals": full_user.learning_goals,
                "created_at": full_user.created_at,
                "updated_at": full_user.updated_at
            }
        }
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Signin failed: {str(e)}")


@router.post("/signout")
async def signout(request: Request, credentials: HTTPAuthorizationCredentials = Depends(security)):
    """
    End the current user session.
    Expects Authorization header with Bearer token.
    """
    try:
        token = credentials.credentials

        # Invalidate the session in Neon Postgres
        success = await SessionService.invalidate_session(token)

        if success:
            return {
                "success": True,
                "message": "Successfully signed out"
            }
        else:
            raise HTTPException(status_code=400, detail="Failed to sign out")
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Signout failed: {str(e)}")


@router.get("/session-status")
async def session_status(credentials: HTTPAuthorizationCredentials = Depends(security)):
    """
    Check the status of the current session.
    Expects Authorization header with Bearer token.
    """
    try:
        token = credentials.credentials

        # Validate the session
        is_valid = await SessionService.validate_session(token)

        if is_valid:
            # Get the session to retrieve user info
            session = await SessionService.get_session_by_token(token)
            if session:
                # Get user info
                user = await ProfileService.get_user_profile(session.user_id)
                if user:
                    return {
                        "valid": True,
                        "user": {
                            "id": user.id,
                            "email": user.email,
                            "os": user.os,
                            "cpu": user.cpu,
                            "gpu": user.gpu,
                            "ram_gb": user.ram_gb,
                            "programming_experience": user.programming_experience,
                            "robotics_experience": user.robotics_experience,
                            "development_environment": user.development_environment,
                            "primary_language": user.primary_language,
                            "learning_goals": user.learning_goals,
                            "created_at": user.created_at,
                            "updated_at": user.updated_at
                        }
                    }
                else:
                    raise HTTPException(status_code=401, detail="User not found")
            else:
                raise HTTPException(status_code=401, detail="Session not found")
        else:
            raise HTTPException(status_code=401, detail="Session is invalid or expired")
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Session status check failed: {str(e)}")


@router.post("/refresh-session")
async def refresh_session(credentials: HTTPAuthorizationCredentials = Depends(security)):
    """
    Extend the current session's expiration time.
    Expects Authorization header with Bearer token.
    """
    try:
        token = credentials.credentials

        # Refresh the session
        updated_session = await SessionService.refresh_session(token)

        if updated_session:
            return {
                "success": True,
                "message": "Session refreshed successfully",
                "token": updated_session.session_token,
                "expires_in": 1800  # 30 minutes in seconds
            }
        else:
            raise HTTPException(status_code=400, detail="Failed to refresh session - invalid token")
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Session refresh failed: {str(e)}")