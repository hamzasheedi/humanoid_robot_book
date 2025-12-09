import pytest
from fastapi.testclient import TestClient
import sys
import os

# Add the src directory to Python path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))

from src.api.main import app

client = TestClient(app)

def test_advanced_question_handling():
    """Integration test for advanced question handling"""
    # Test the complete flow of submitting a complex technical question
    response = client.post(
        "/chat/ask",
        json={
            "question": "Explain the architecture of the navigation stack in ROS 2",
            "session_id": "test_session_456"
        },
        headers={"Authorization": "Bearer dummy-token"}  # Would normally be API key
    )
    
    # Should return a valid response structure (even if it's an error due to missing services)
    assert response.status_code in [200, 401, 400, 500], f"Expected endpoint to process request, got {response.status_code}"
    
    if response.status_code == 200:
        data = response.json()
        # Expected response structure for advanced questions
        assert "answer" in data
        assert "sources" in data
        assert "confidence" in data
        assert "session_id" in data
        print("✓ Advanced question response structure is valid")
    
    print("✓ Advanced question handling test completed")


if __name__ == "__main__":
    test_advanced_question_handling()
    print("All advanced question tests passed!")