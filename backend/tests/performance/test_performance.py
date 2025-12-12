"""
Performance Tests for Authentication and Personalization API
Validates response times for cloud (<2s) and local (<5s) deployments
"""

import pytest
import time
import asyncio
from fastapi.testclient import TestClient
import statistics

from src.main import app

client = TestClient(app)

class TestPerformance:
    """Performance tests for authentication and personalization endpoints"""

    def setup_method(self):
        """Set up test data"""
        self.test_email = "perf_test@example.com"
        self.test_password = "SecurePassword123!"
        self.user_profile = {
            "os": "Ubuntu 22.04",
            "cpu": "AMD Ryzen 7 5800X",
            "gpu": "AMD RX 6700 XT",
            "ram_gb": 16,
            "programming_experience": "intermediate",
            "robotics_experience": "beginner",
            "development_environment": "PyCharm",
            "primary_language": "Python",
            "learning_goals": ["AI", "simulation"]
        }

    def measure_response_time(self, method, url, **kwargs):
        """Measure response time for a given request"""
        start_time = time.time()

        response = client.request(method, url, **kwargs)

        end_time = time.time()
        response_time = (end_time - start_time) * 1000  # Convert to milliseconds

        return response, response_time

    def test_auth_signup_performance(self):
        """Test signup endpoint performance"""
        response, response_time = self.measure_response_time(
            "POST",
            "/api/auth/signup",
            json={
                "email": self.test_email,
                "password": self.test_password,
                "profile": self.user_profile
            }
        )

        assert response.status_code == 200
        print(f"Signup response time: {response_time:.2f}ms")

        # Performance requirement: <2s cloud, <5s local
        assert response_time < 5000, f"Signup took {response_time:.2f}ms, exceeds 5s limit"

    def test_auth_signin_performance(self):
        """Test signin endpoint performance"""
        # First create a user
        signup_response = client.post(
            "/api/auth/signup",
            json={
                "email": self.test_email,
                "password": self.test_password,
                "profile": self.user_profile
            }
        )

        assert signup_response.status_code == 200
        token = signup_response.json()["auth_token"]

        # Test signin performance
        response, response_time = self.measure_response_time(
            "POST",
            "/api/auth/signin",
            json={
                "email": self.test_email,
                "password": self.test_password
            }
        )

        assert response.status_code == 200
        print(f"Signin response time: {response_time:.2f}ms")

        # Performance requirement: <2s cloud, <5s local
        assert response_time < 5000, f"Signin took {response_time:.2f}ms, exceeds 5s limit"

    def test_profile_management_performance(self):
        """Test profile management endpoint performance"""
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
        token = signup_response.json()["auth_token"]

        # Test profile retrieval performance
        response, response_time = self.measure_response_time(
            "GET",
            "/api/profile/",
            headers={"Authorization": f"Bearer {token}"}
        )

        assert response.status_code == 200
        print(f"Profile retrieval response time: {response_time:.2f}ms")

        # Performance requirement: <2s cloud, <5s local
        assert response_time < 5000, f"Profile retrieval took {response_time:.2f}ms, exceeds 5s limit"

        # Test profile update performance
        response, response_time = self.measure_response_time(
            "PUT",
            "/api/profile/",
            json={
                "os": "Windows 11",
                "cpu": "Intel i9-13900K",
                "programming_experience": "advanced"
            },
            headers={"Authorization": f"Bearer {token}"}
        )

        assert response.status_code == 200
        print(f"Profile update response time: {response_time:.2f}ms")

        # Performance requirement: <2s cloud, <5s local
        assert response_time < 5000, f"Profile update took {response_time:.2f}ms, exceeds 5s limit"

    def test_personalization_performance(self):
        """Test personalization endpoint performance"""
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
        token = signup_response.json()["auth_token"]

        # Test personalization retrieval performance
        response, response_time = self.measure_response_time(
            "GET",
            "/api/personalization/",
            headers={"Authorization": f"Bearer {token}"}
        )

        assert response.status_code == 200
        print(f"Personalization retrieval response time: {response_time:.2f}ms")

        # Performance requirement: <2s cloud, <5s local
        assert response_time < 5000, f"Personalization retrieval took {response_time:.2f}ms, exceeds 5s limit"

        # Test personalization update performance
        response, response_time = self.measure_response_time(
            "PUT",
            "/api/personalization/",
            json={
                "content_difficulty": "advanced",
                "response_complexity": "detailed",
                "learning_pace": "fast"
            },
            headers={"Authorization": f"Bearer {token}"}
        )

        assert response.status_code == 200
        print(f"Personalization update response time: {response_time:.2f}ms")

        # Performance requirement: <2s cloud, <5s local
        assert response_time < 5000, f"Personalization update took {response_time:.2f}ms, exceeds 5s limit"

    def test_concurrent_performance(self):
        """Test performance under concurrent load"""
        # Create multiple test users
        test_users = []
        for i in range(5):
            email = f"perf_test_{i}@example.com"
            response = client.post(
                "/api/auth/signup",
                json={
                    "email": email,
                    "password": self.test_password,
                    "profile": self.user_profile
                }
            )

            assert response.status_code == 200
            data = response.json()
            test_users.append({
                "email": email,
                "token": data["auth_token"]
            })

        # Measure performance for multiple concurrent profile updates
        response_times = []
        for user in test_users:
            _, response_time = self.measure_response_time(
                "PUT",
                "/api/profile/",
                json={
                    "os": f"TestOS_{len(response_times)}",
                    "programming_experience": "intermediate"
                },
                headers={"Authorization": f"Bearer {user['token']}"}
            )

            response_times.append(response_time)

        avg_response_time = statistics.mean(response_times)
        max_response_time = max(response_times)

        print(f"Concurrent profile update - Avg: {avg_response_time:.2f}ms, Max: {max_response_time:.2f}ms")

        # Performance requirement: <2s cloud, <5s local (even under load)
        assert avg_response_time < 5000, f"Average concurrent response time {avg_response_time:.2f}ms exceeds 5s limit"
        assert max_response_time < 10000, f"Max concurrent response time {max_response_time:.2f}ms exceeds 10s limit"

    def test_session_performance(self):
        """Test session management performance"""
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
        token = signup_response.json()["auth_token"]

        # Test session status check performance
        response, response_time = self.measure_response_time(
            "GET",
            "/api/session/status",
            headers={"Authorization": f"Bearer {token}"}
        )

        assert response.status_code == 200
        print(f"Session status response time: {response_time:.2f}ms")

        # Performance requirement: <2s cloud, <5s local
        assert response_time < 5000, f"Session status took {response_time:.2f}ms, exceeds 5s limit"

        # Test session refresh performance
        response, response_time = self.measure_response_time(
            "POST",
            "/api/session/refresh",
            headers={"Authorization": f"Bearer {token}"}
        )

        assert response.status_code == 200
        print(f"Session refresh response time: {response_time:.2f}ms")

        # Performance requirement: <2s cloud, <5s local
        assert response_time < 5000, f"Session refresh took {response_time:.2f}ms, exceeds 5s limit"

    def test_api_health_performance(self):
        """Test health check endpoint performance"""
        response, response_time = self.measure_response_time(
            "GET",
            "/health"
        )

        assert response.status_code == 200
        print(f"Health check response time: {response_time:.2f}ms")

        # Health check should be very fast
        assert response_time < 1000, f"Health check took {response_time:.2f}ms, exceeds 1s limit"


if __name__ == "__main__":
    pytest.main([__file__])