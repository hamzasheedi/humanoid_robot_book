#!/usr/bin/env python3
"""
Better-Auth User Sign-In Script

This script performs user authentication via Better-Auth API.
"""

import json
import requests
from typing import Dict, Any

def signin_user(email: str, password: str, api_base_url: str = "http://localhost:8000") -> Dict[str, Any]:
    """
    Authenticate user via Better-Auth API.

    Args:
        email: User email address
        password: User password
        api_base_url: Base URL for the Better-Auth API

    Returns:
        Dictionary with authentication result including success status,
        message, token, and user profile
    """
    try:
        # Prepare the request payload
        payload = {
            "email": email,
            "password": password
        }

        # Make the API call to authenticate the user
        response = requests.post(
            f"{api_base_url}/api/auth/login",
            json=payload,
            headers={"Content-Type": "application/json"}
        )

        # Handle the response
        if response.status_code == 200:
            response_data = response.json()
            return {
                "success": True,
                "message": "Login successful",
                "auth_token": response_data.get("token", response_data.get("auth_token")),
                "user_profile": response_data.get("user", response_data.get("profile", {}))
            }
        elif response.status_code == 401:
            return {
                "success": False,
                "message": "Invalid credentials",
                "auth_token": None,
                "user_profile": {}
            }
        elif response.status_code == 404:
            return {
                "success": False,
                "message": "Account not found",
                "auth_token": None,
                "user_profile": {}
            }
        elif response.status_code == 422:
            error_data = response.json()
            return {
                "success": False,
                "message": f"Validation error: {error_data.get('message', 'Invalid input')}",
                "auth_token": None,
                "user_profile": {}
            }
        else:
            return {
                "success": False,
                "message": f"API error: {response.status_code} - {response.text}",
                "auth_token": None,
                "user_profile": {}
            }

    except requests.exceptions.ConnectionError:
        return {
            "success": False,
            "message": "Could not connect to Better-Auth API",
            "auth_token": None,
            "user_profile": {}
        }
    except requests.exceptions.RequestException as e:
        return {
            "success": False,
            "message": f"Request error: {str(e)}",
            "auth_token": None,
            "user_profile": {}
        }
    except Exception as e:
        return {
            "success": False,
            "message": f"Unexpected error: {str(e)}",
            "auth_token": None,
            "user_profile": {}
        }


def main():
    """
    Example usage of the signin_user function.
    This would typically be called by Claude when executing the skill.
    """
    # Example input (in practice, these would come from the skill execution)
    email = "user@example.com"
    password = "securePassword123"

    result = signin_user(email, password)
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()