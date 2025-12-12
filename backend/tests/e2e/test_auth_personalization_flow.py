"""
End-to-End Integration Tests for Authentication and Personalization Flow
Tests the complete signup → save background → personalized book → personalized chatbot flow
"""

import pytest
import asyncio
from fastapi.testclient import TestClient
from unittest.mock import patch, MagicMock
import os

from src.main import app
from src.services.auth_service import AuthService
from src.services.profile_service import ProfileService
from src.services.personalization_service import PersonalizationService

client = TestClient(app)

class TestAuthPersonalizationFlow:
    """Test the complete authentication and personalization flow"""

    def setup_method(self):
        """Set up test dependencies"""
        self.test_email = "testuser@example.com"
        self.test_password = "SecurePassword123!"
        self.user_profile = {
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

    def test_complete_signup_flow(self):
        """Test complete signup flow with profile collection"""
        # Test signup with profile information
        response = client.post(
            "/api/auth/signup",
            json={
                "email": self.test_email,
                "password": self.test_password,
                "profile": self.user_profile
            }
        )

        assert response.status_code == 200
        data = response.json()
        assert data["success"] is True
        assert "user_id" in data
        assert "auth_token" in data

        # Verify profile was saved correctly
        token = data["auth_token"]
        profile_response = client.get(
            "/api/profile/",
            headers={"Authorization": f"Bearer {token}"}
        )

        assert profile_response.status_code == 200
        profile_data = profile_response.json()
        assert profile_data["success"] is True
        assert profile_data["profile"]["email"] == self.test_email
        assert profile_data["profile"]["os"] == self.user_profile["os"]
        assert profile_data["profile"]["programming_experience"] == self.user_profile["programming_experience"]

    def test_signin_flow(self):
        """Test signin flow and session management"""
        # First, create a user
        signup_response = client.post(
            "/api/auth/signup",
            json={
                "email": self.test_email,
                "password": self.test_password,
                "profile": self.user_profile
            }
        )

        assert signup_response.status_code == 200
        signup_data = signup_response.json()
        assert signup_data["success"] is True
        token = signup_data["auth_token"]

        # Test signin with correct credentials
        signin_response = client.post(
            "/api/auth/signin",
            json={
                "email": self.test_email,
                "password": self.test_password
            }
        )

        assert signin_response.status_code == 200
        signin_data = signin_response.json()
        assert signin_data["success"] is True
        assert "auth_token" in signin_data
        assert signin_data["user"]["email"] == self.test_email

    def test_personalization_flow(self):
        """Test personalization API endpoints"""
        # Create a user first
        signup_response = client.post(
            "/api/auth/signup",
            json={
                "email": self.test_email,
                "password": self.test_password,
                "profile": self.user_profile
            }
        )

        assert signup_response.status_code == 200
        signup_data = signup_response.json()
        assert signup_data["success"] is True
        token = signup_data["auth_token"]

        # Test getting personalization settings (should be created automatically)
        personalization_response = client.get(
            "/api/personalization/",
            headers={"Authorization": f"Bearer {token}"}
        )

        assert personalization_response.status_code == 200
        personalization_data = personalization_response.json()
        assert personalization_data["success"] is True
        assert "preferences" in personalization_data

        # Test updating personalization settings
        update_response = client.put(
            "/api/personalization/",
            json={
                "content_difficulty": "advanced",
                "preferred_examples": ["hardware-focused", "simulation"],
                "response_complexity": "detailed",
                "interaction_style": "problem-solving",
                "learning_pace": "fast"
            },
            headers={"Authorization": f"Bearer {token}"}
        )

        assert update_response.status_code == 200
        update_data = update_response.json()
        assert update_data["success"] is True
        assert update_data["preferences"]["content_difficulty"] == "advanced"

    def test_session_management(self):
        """Test session management functionality"""
        # Create a user
        signup_response = client.post(
            "/api/auth/signup",
            json={
                "email": self.test_email,
                "password": self.test_password,
                "profile": self.user_profile
            }
        )

        assert signup_response.status_code == 200
        signup_data = signup_response.json()
        assert signup_data["success"] is True
        token = signup_data["auth_token"]

        # Test session status
        session_response = client.get(
            "/api/session/status",
            headers={"Authorization": f"Bearer {token}"}
        )

        assert session_response.status_code == 200
        session_data = session_response.json()
        assert session_data["valid"] is True
        assert session_data["user"]["email"] == self.test_email

        # Test session refresh
        refresh_response = client.post(
            "/api/session/refresh",
            headers={"Authorization": f"Bearer {token}"}
        )

        assert refresh_response.status_code == 200
        refresh_data = refresh_response.json()
        assert refresh_data["success"] is True

    @pytest.mark.asyncio
    async def test_concurrent_user_sessions(self):
        """Test handling of multiple concurrent user sessions"""
        # Create multiple users
        users = []
        for i in range(3):
            email = f"testuser{i}@example.com"
            password = "SecurePassword123!"

            response = client.post(
                "/api/auth/signup",
                json={
                    "email": email,
                    "password": password,
                    "profile": self.user_profile
                }
            )

            assert response.status_code == 200
            data = response.json()
            assert data["success"] is True
            users.append({
                "email": email,
                "token": data["auth_token"]
            })

        # Test that each user can access their own profile
        for user in users:
            profile_response = client.get(
                "/api/profile/",
                headers={"Authorization": f"Bearer {user['token']}"}
            )

            assert profile_response.status_code == 200
            profile_data = profile_response.json()
            assert profile_data["success"] is True
            assert profile_data["profile"]["email"] == user["email"]

    def test_error_handling(self):
        """Test error handling for invalid inputs"""
        # Test signup with invalid email
        response = client.post(
            "/api/auth/signup",
            json={
                "email": "invalid-email",
                "password": self.test_password,
                "profile": self.user_profile
            }
        )

        assert response.status_code == 400

        # Test signup with weak password
        response = client.post(
            "/api/auth/signup",
            json={
                "email": self.test_email,
                "password": "weak",
                "profile": self.user_profile
            }
        )

        assert response.status_code == 400

        # Test signin with non-existent user
        response = client.post(
            "/api/auth/signin",
            json={
                "email": "nonexistent@example.com",
                "password": self.test_password
            }
        )

        assert response.status_code == 401

    def test_security_validation(self):
        """Test security validations"""
        # Test unauthorized access to protected endpoints
        response = client.get("/api/profile/")
        assert response.status_code == 401  # Unauthorized

        # Test with invalid token
        response = client.get(
            "/api/profile/",
            headers={"Authorization": "Bearer invalid-token"}
        )
        assert response.status_code == 401  # Unauthorized


if __name__ == "__main__":
    pytest.main([__file__])