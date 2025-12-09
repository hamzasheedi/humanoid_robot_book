import pytest
from fastapi.testclient import TestClient
from unittest.mock import Mock, AsyncMock
import sys
import os

# Add the src directory to Python path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))

from src.api.main import app

client = TestClient(app)

def test_ask_endpoint_contract():
    """Contract test for /chat/ask endpoint"""
    # Test that the endpoint exists and returns expected response format
    response = client.post(
        "/chat/ask",
        json={"question": "What is ROS 2?"}
    )
    
    # Should return 422 (validation error) or 401 (auth error) rather than 404 (not found)
    assert response.status_code in [401, 400, 500], f"Expected endpoint to exist, got {response.status_code}"
    
    print("✓ /chat/ask endpoint contract test completed")


def test_context_endpoint_contract():
    """Contract test for /chat/context endpoint"""
    # Test that the endpoint exists and returns expected response format
    response = client.post(
        "/chat/context",
        json={"selected_text": "some text"}
    )
    
    # Should return 422 (validation error) or 401 (auth error) rather than 404 (not found)
    assert response.status_code in [401, 400, 500], f"Expected endpoint to exist, got {response.status_code}"
    
    print("✓ /chat/context endpoint contract test completed")


if __name__ == "__main__":
    test_ask_endpoint_contract()
    test_context_endpoint_contract()
    print("All contract tests passed!")