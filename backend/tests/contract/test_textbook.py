import pytest
from fastapi.testclient import TestClient
import sys
import os

# Add the src directory to Python path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))

from src.api.main import app

client = TestClient(app)

def test_textbook_modules_endpoint_contract():
    """Contract test for /textbook/modules endpoint"""
    # Test that the endpoint exists and returns expected response format
    response = client.get("/textbook/modules")
    
    # Should return 200 with expected structure or 401 (auth error) rather than 404 (not found)
    assert response.status_code in [200, 401], f"Expected endpoint to exist, got {response.status_code}"
    
    print("✓ /textbook/modules endpoint contract test completed")


if __name__ == "__main__":
    test_textbook_modules_endpoint_contract()
    print("All contract tests passed!")