---
name: create_betterauth_user_skill
description: Calls the Better-Auth API to create a new user account using validated user profile and credentials.
---

# Better-Auth User Creation Skill

## Overview

This skill calls the Better-Auth API to create a new user account using validated user profile and credentials. It handles the API communication, error handling, and returns structured responses for user registration operations.

## Input

```json
{
  "email": "user email",
  "password": "user password",
  "validated_profile": "output of validate_user_input_skill"
}
```

## Output

```json
{
  "result": {
    "success": "true/false",
    "message": "confirmation or error message",
    "user_id": "Better-Auth ID if success"
  }
}
```

## Resources

This skill includes resource directories that demonstrate how to organize different types of bundled resources:

### scripts/
Executable code (Python/Bash/etc.) that can be run directly to perform Better-Auth user creation operations.

### references/
Documentation and reference material for Better-Auth API specifications, authentication requirements, and user management guidelines.

### assets/
Template files and examples of properly formatted API requests and responses for Better-Auth integration.

## Functionality

### Validate Input Parameters
- Ensure email, password, and validated_profile are provided
- Verify email format and password strength requirements
- Validate structure of validated_profile data

### Make Better-Auth API Call
- Construct proper API request to Better-Auth registration endpoint
- Include email, password, and profile information in request
- Handle proper authentication headers if required

### Handle API Response
- Process successful user creation responses
- Extract user_id from successful registration
- Return success status with confirmation message

### Error Handling
- Handle duplicate email errors from Better-Auth
- Handle weak password validation errors
- Handle API communication failures
- Return appropriate error messages for each failure case

### Response Formatting
- Format responses as structured JSON
- Include success status, message, and user_id when applicable
- Provide clear error messages for debugging