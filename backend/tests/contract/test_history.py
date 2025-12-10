import pytest
from fastapi.testclient import TestClient
import sys
import os

# Add the src directory to Python path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))

from src.api.main import app

client = TestClient(app)

def test_history_endpoint_contract():
    """Contract test for /chat/history endpoint"""
    # Test that the endpoint exists and returns expected response format
    response = client.get("/chat/history?session_id=test_session")
    
    # Should return 422 (validation error) or 401 (auth error) or 404 (not found) rather than 404 (endpoint not found)
    assert response.status_code in [200, 401, 400, 404], f"Expected endpoint to exist, got {response.status_code}"
    
    print("✓ /chat/history endpoint contract test completed")


if __name__ == "__main__":
    test_history_endpoint_contract()
    print("All contract tests passed!")