#!/usr/bin/env python3
"""
Better-Auth User Creation Script

This script performs the actual user creation via Better-Auth API.
"""

import json
import requests
from typing import Dict, Any

def create_betterauth_user(email: str, password: str, validated_profile: Dict[str, Any], api_base_url: str = "http://localhost:8000") -> Dict[str, Any]:
    """
    Create a new user via Better-Auth API.

    Args:
        email: User email address
        password: User password
        validated_profile: Validated user profile data
        api_base_url: Base URL for the Better-Auth API

    Returns:
        Dictionary with success status, message, and user_id if successful
    """
    try:
        # Prepare the request payload
        payload = {
            "email": email,
            "password": password,
            "profile": validated_profile
        }

        # Make the API call to create the user
        response = requests.post(
            f"{api_base_url}/api/auth/register",
            json=payload,
            headers={"Content-Type": "application/json"}
        )

        # Handle the response
        if response.status_code == 200:
            response_data = response.json()
            return {
                "success": True,
                "message": "User created successfully",
                "user_id": response_data.get("user_id", response_data.get("id"))
            }
        elif response.status_code == 409:
            return {
                "success": False,
                "message": "Email already exists",
                "user_id": None
            }
        elif response.status_code == 400:
            error_data = response.json()
            return {
                "success": False,
                "message": f"Validation error: {error_data.get('message', 'Invalid input')}",
                "user_id": None
            }
        else:
            return {
                "success": False,
                "message": f"API error: {response.status_code} - {response.text}",
                "user_id": None
            }

    except requests.exceptions.ConnectionError:
        return {
            "success": False,
            "message": "Could not connect to Better-Auth API",
            "user_id": None
        }
    except requests.exceptions.RequestException as e:
        return {
            "success": False,
            "message": f"Request error: {str(e)}",
            "user_id": None
        }
    except Exception as e:
        return {
            "success": False,
            "message": f"Unexpected error: {str(e)}",
            "user_id": None
        }


def main():
    """
    Example usage of the create_betterauth_user function.
    This would typically be called by Claude when executing the skill.
    """
    # Example input (in practice, these would come from the skill execution)
    email = "user@example.com"
    password = "securePassword123"
    validated_profile = {
        "name": "John Doe",
        "age": 30,
        "role": "student"
    }

    result = create_betterauth_user(email, password, validated_profile)
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()