---
name: signin_user_skill
description: Handles user login via Better-Auth API and returns authentication token and session information.
---

# Sign-In User Skill

## Overview

This skill handles user login via Better-Auth API and returns authentication token and session information. It manages the authentication process, validates credentials, and provides the necessary data for personalization and course recommendation systems.

## Input

```json
{
  "email": "user email",
  "password": "user password"
}
```

## Output

```json
{
  "auth_result": {
    "success": "true/false",
    "message": "login confirmation or error message",
    "auth_token": "JWT or session token if success",
    "user_profile": "JSON profile associated with the user"
  }
}
```

## Resources

This skill includes resource directories that demonstrate how to organize different types of bundled resources:

### scripts/
Executable code (Python/Bash/etc.) that can be run directly to perform user authentication operations.

### references/
Documentation and reference material for Better-Auth API specifications, authentication requirements, and session management guidelines.

### assets/
Template files and examples of properly formatted API requests and responses for Better-Auth integration.

## Functionality

### Validate Input Parameters
- Ensure email and password are provided
- Verify email format is valid
- Check that password is not empty

### Authenticate via Better-Auth API
- Construct proper API request to Better-Auth login endpoint
- Include email and password in request
- Handle proper authentication headers if required

### Process Authentication Response
- Handle successful authentication responses
- Extract authentication token (JWT or session token)
- Retrieve user profile information
- Return success status with token and profile

### Handle Authentication Errors
- Handle invalid credentials errors
- Handle account not found errors
- Handle account disabled/blocked errors
- Handle API communication failures

### Prepare Output for Personalization
- Format user profile in a way that's usable by personalization agents
- Include relevant user information for course recommendation
- Ensure authentication token is properly formatted for subsequent API calls

### Response Formatting
- Format responses as structured JSON
- Include success status, message, token, and user profile
- Provide clear error messages for debugging