from fastapi import APIRouter, HTTPException, Depends
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials

from ..models.user import UserCreate, User
from ..services.auth_service import AuthService
from ..services.profile_service import ProfileService
from ..services.session_service import SessionService

router = APIRouter()
security = HTTPBearer()


@router.post("/signup", response_model=User)
async def signup(user_data: UserCreate):
    """
    Create a new user account with optional profile information.
    """
    try:
        # Create user profile with hashed password
        user = await ProfileService.create_user_profile(user_data)

        if not user:
            raise HTTPException(status_code=400, detail="Failed to create user account")

        # In a real implementation, we would create a session for the new user
        # For now, we'll just return the created user
        return user
    except Exception as e:
        raise HTTPException(status_code=400, detail=f"Signup failed: {str(e)}")


@router.post("/signin")
async def signin(email: str, password: str):
    """
    Authenticate a user and create a session.
    Expected request body: {"email": "user@example.com", "password": "user_password"}
    """
    try:
        # Validate email and password
        if not email or not password:
            raise HTTPException(status_code=400, detail="Email and password are required")

        # In a real implementation, we would:
        # 1. Look up the user by email in Neon Postgres
        # 2. Verify the password using AuthService.verify_password
        # 3. Create a session using SessionService.create_session
        # 4. Return the session token and user information

        # Placeholder implementation - in real app would connect to Neon Postgres
        # user = await database.users.find_one({"email": email})
        # if not user:
        #     raise HTTPException(status_code=401, detail="Invalid credentials")
        #
        # if not AuthService.verify_password(password, user["hashed_password"]):
        #     raise HTTPException(status_code=401, detail="Invalid credentials")
        #
        # session = await SessionService.create_session(
        #     user_id=user["id"],
        #     ip_address=request.client.host,
        #     user_agent=request.headers.get('user-agent')
        # )

        # Return a mock response for now
        return {
            "message": "Signin successful",
            "user_id": "mock_user_id",
            "session_token": "mock_session_token"
        }
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Signin failed: {str(e)}")


@router.post("/signout")
async def signout(credentials: HTTPAuthorizationCredentials = Depends(security)):
    """
    End the current user session.
    Expects Authorization header with Bearer token.
    """
    try:
        token = credentials.credentials

        # In a real implementation, we would:
        # 1. Validate the session token
        # 2. Mark the session as inactive in Neon Postgres
        # 3. Return success confirmation

        # For now, just invalidate the session
        success = await SessionService.invalidate_session(token)

        if success:
            return {"message": "Successfully signed out"}
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
            # In a real implementation, we would return user info and session details
            return {
                "authenticated": True,
                "session_valid": True,
                "message": "Session is active"
            }
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
                "message": "Session refreshed successfully",
                "expires_at": updated_session.expires_at
            }
        else:
            raise HTTPException(status_code=400, detail="Failed to refresh session - invalid token")
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Session refresh failed: {str(e)}")