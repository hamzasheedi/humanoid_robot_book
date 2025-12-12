---
name: signup_agent
description: Handle new user registration and profile collection.
model: inherit
---

# Sign-Up Agent

## Overview

This agent handles new user registration and profile collection. It orchestrates the user creation process by collecting user information, creating a Better-Auth account, establishing a session, and returning confirmation with the session ID.

## Skills Used

- `betterauth_user` → Create Better-Auth account
- `markdown_format` → (Optional) Format any welcome content
- `session_state` → Create new session for user

## Input

```json
{
  "user_info": {
    "email": "user email",
    "password": "user password",
    "profile": {
      "name": "user name",
      "age": "user age",
      "role": "user role",
      "other_fields": "additional profile information"
    }
  }
}
```

## Output

```json
{
  "success": "true/false",
  "message": "registration confirmation or error message",
  "user_id": "Better-Auth user ID if successful",
  "session_id": "session ID for the new user",
  "auth_token": "authentication token for the new user"
}
```

## Resources

This agent includes resource directories that demonstrate how to organize different types of bundled resources:

### scripts/
Executable code (Python/Bash/etc.) that can be run directly to orchestrate the sign-up process.

### references/
Documentation and reference material for user registration workflows, profile validation requirements, and onboarding best practices.

### assets/
Template files and examples of properly formatted registration responses and welcome messages.

## Functionality

### Collect User Information
- Validate that required user information is provided
- Ensure email format is valid
- Verify password meets minimum requirements
- Validate profile information structure

### Create Better-Auth Account
- Call the `betterauth_user` skill to create the account
- Pass validated user information to the skill: `{"email": "...", "password": "...", "validated_profile": {...}}`
- Handle account creation success or failure
- Extract user ID from successful creation

### Create User Session
- Call the `session_state` skill to create a new session
- Use the user ID from account creation: `{"user_id": "...", "session_id": null, "context": "Welcome message..."}`
- Associate the session with the newly created user ID
- Initialize session with welcome context

### Format Welcome Content (Optional)
- Use `markdown_format` skill to format welcome messages if needed
- Format any onboarding content for the new user
- Prepare welcome materials in proper format

### Handle Registration Response
- Process the results from all skills
- Create a unified response with user ID and session ID
- Include authentication token for immediate use

### Error Handling
- Handle account creation failures from `betterauth_user` skill
- Handle session creation failures from `session_state` skill
- Provide clear error messages for different failure scenarios

### Response Formatting
- Format responses as structured JSON
- Include success status, message, user ID, session ID, and auth token
- Provide clear feedback for debugging if needed

## Execution Flow

1. Validate input user information
2. Execute `betterauth_user` skill with user credentials and profile
3. If successful, execute `session_state` skill to create new session
4. Optionally execute `markdown_format` skill for welcome content
5. Return unified response with all necessary information
