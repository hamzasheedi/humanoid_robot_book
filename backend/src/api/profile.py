from fastapi import APIRouter, HTTPException, Depends
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials

from ..models.user import User, UserUpdate
from ..services.profile_service import ProfileService

router = APIRouter()
security = HTTPBearer()


@router.get("/", response_model=User)
async def get_profile(credentials: HTTPAuthorizationCredentials = Depends(security)):
    """
    Retrieve the current user's profile information.
    Expects Authorization header with Bearer token containing user ID.
    """
    try:
        # In a real implementation, we would:
        # 1. Decode the token to get user ID
        # 2. Fetch user profile from Neon Postgres
        # 3. Return user profile information

        # Placeholder implementation
        # token_data = AuthService.verify_access_token(credentials.credentials)
        # if not token_data:
        #     raise HTTPException(status_code=401, detail="Invalid token")
        #
        # user_id = token_data.get("sub")
        # user = await ProfileService.get_user_profile(user_id)
        #
        # if not user:
        #     raise HTTPException(status_code=404, detail="User not found")

        # For now, return a mock user profile
        return {
            "id": "mock_user_id",
            "email": "mock@example.com",
            "created_at": "2023-01-01T00:00:00Z",
            "updated_at": "2023-01-01T00:00:00Z",
            "os": "Windows 10",
            "cpu": "Intel i7-12700K",
            "gpu": "NVIDIA RTX 3080",
            "ram_gb": 32,
            "programming_experience": "intermediate",
            "robotics_experience": "beginner",
            "development_environment": "VS Code",
            "primary_language": "Python",
            "learning_goals": ["robotics", "AI"]
        }
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to retrieve profile: {str(e)}")


@router.put("/", response_model=User)
async def update_profile(
    profile_update: UserUpdate,
    credentials: HTTPAuthorizationCredentials = Depends(security)
):
    """
    Update the current user's profile information.
    Expects Authorization header with Bearer token and profile update data in request body.
    """
    try:
        # In a real implementation, we would:
        # 1. Decode the token to get user ID
        # 2. Validate the profile update data
        # 3. Update user profile in Neon Postgres
        # 4. Return updated user profile information

        # Placeholder implementation
        # token_data = AuthService.verify_access_token(credentials.credentials)
        # if not token_data:
        #     raise HTTPException(status_code=401, detail="Invalid token")
        #
        # user_id = token_data.get("sub")
        # updated_user = await ProfileService.update_user_profile(user_id, profile_update)
        #
        # if not updated_user:
        #     raise HTTPException(status_code=404, detail="User not found")

        # For now, return a mock updated user profile
        return {
            "id": "mock_user_id",
            "email": "mock@example.com",
            "created_at": "2023-01-01T00:00:00Z",
            "updated_at": "2023-01-01T00:00:00Z",  # Updated timestamp
            "os": profile_update.os or "Windows 10",
            "cpu": profile_update.cpu or "Intel i7-12700K",
            "gpu": profile_update.gpu or "NVIDIA RTX 3080",
            "ram_gb": profile_update.ram_gb or 32,
            "programming_experience": profile_update.programming_experience or "intermediate",
            "robotics_experience": profile_update.robotics_experience or "beginner",
            "development_environment": profile_update.development_environment or "VS Code",
            "primary_language": profile_update.primary_language or "Python",
            "learning_goals": profile_update.learning_goals or ["robotics", "AI"]
        }
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to update profile: {str(e)}")