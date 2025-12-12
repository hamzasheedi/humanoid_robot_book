# API Documentation: Physical AI & Humanoid Robotics Textbook

## Overview

This document provides comprehensive documentation for the authentication and personalization API endpoints of the Physical AI & Humanoid Robotics Textbook platform.

## Base URL

```
http://localhost:8000/api/
```

For production deployments, replace with your production URL.

## Authentication

Most endpoints require authentication using a Bearer token. Include the token in the Authorization header:

```
Authorization: Bearer {token}
```

## API Endpoints

### Authentication Endpoints

#### POST `/auth/signup`

Create a new user account with optional profile information.

**Request Body:**
```json
{
  "email": "user@example.com",
  "password": "SecurePassword123!",
  "profile": {
    "os": "Windows 10",
    "cpu": "Intel i7-12700K",
    "gpu": "NVIDIA RTX 3080",
    "ram_gb": 32,
    "programming_experience": "intermediate",
    "robotics_experience": "beginner",
    "development_environment": "VS Code",
    "primary_language": "Python",
    "learning_goals": ["robotics", "AI"]
  }
}
```

**Response:**
```json
{
  "success": true,
  "message": "User created successfully",
  "user_id": "user_12345",
  "auth_token": "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...",
  "profile": {
    "id": "user_12345",
    "email": "user@example.com",
    "os": "Windows 10",
    "cpu": "Intel i7-12700K",
    "gpu": "NVIDIA RTX 3080",
    "ram_gb": 32,
    "programming_experience": "intermediate",
    "robotics_experience": "beginner",
    "development_environment": "VS Code",
    "primary_language": "Python",
    "learning_goals": ["robotics", "AI"],
    "created_at": "2023-01-01T00:00:00Z",
    "updated_at": "2023-01-01T00:00:00Z"
  }
}
```

**Status Codes:**
- 200: Success
- 400: Bad request (invalid input)
- 409: Conflict (email already exists)

---

#### POST `/auth/signin`

Authenticate an existing user.

**Request Body:**
```json
{
  "email": "user@example.com",
  "password": "SecurePassword123!"
}
```

**Response:**
```json
{
  "success": true,
  "message": "Signin successful",
  "user_id": "user_12345",
  "auth_token": "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...",
  "user": {
    "id": "user_12345",
    "email": "user@example.com",
    "os": "Windows 10",
    "cpu": "Intel i7-12700K",
    "programming_experience": "intermediate",
    "robotics_experience": "beginner",
    "created_at": "2023-01-01T00:00:00Z",
    "updated_at": "2023-01-01T00:00:00Z"
  }
}
```

**Status Codes:**
- 200: Success
- 400: Bad request (invalid input)
- 401: Unauthorized (invalid credentials)

---

#### POST `/auth/signout`

End the current user session.

**Headers:**
```
Authorization: Bearer {token}
```

**Response:**
```json
{
  "success": true,
  "message": "Signout successful"
}
```

**Status Codes:**
- 200: Success
- 401: Unauthorized (invalid token)

---

#### GET `/auth/session`

Check the current session status.

**Headers:**
```
Authorization: Bearer {token}
```

**Response:**
```json
{
  "valid": true,
  "user": {
    "id": "user_12345",
    "email": "user@example.com",
    "os": "Windows 10",
    "cpu": "Intel i7-12700K",
    "programming_experience": "intermediate",
    "robotics_experience": "beginner",
    "created_at": "2023-01-01T00:00:00Z",
    "updated_at": "2023-01-01T00:00:00Z"
  }
}
```

**Status Codes:**
- 200: Success
- 401: Unauthorized (invalid token)

---

### Profile Endpoints

#### GET `/profile/`

Retrieve the current user's profile information.

**Headers:**
```
Authorization: Bearer {token}
```

**Response:**
```json
{
  "success": true,
  "message": "Profile retrieved successfully",
  "profile": {
    "id": "user_12345",
    "email": "user@example.com",
    "os": "Windows 10",
    "cpu": "Intel i7-12700K",
    "gpu": "NVIDIA RTX 3080",
    "ram_gb": 32,
    "programming_experience": "intermediate",
    "robotics_experience": "beginner",
    "development_environment": "VS Code",
    "primary_language": "Python",
    "learning_goals": ["robotics", "AI"],
    "created_at": "2023-01-01T00:00:00Z",
    "updated_at": "2023-01-01T00:00:00Z"
  }
}
```

**Status Codes:**
- 200: Success
- 401: Unauthorized (invalid token)

---

#### PUT `/profile/`

Update the current user's profile information.

**Headers:**
```
Authorization: Bearer {token}
Content-Type: application/json
```

**Request Body:**
```json
{
  "os": "Windows 11",
  "cpu": "Intel i9-13900K",
  "programming_experience": "advanced",
  "development_environment": "PyCharm"
}
```

**Response:**
```json
{
  "success": true,
  "message": "Profile updated successfully",
  "profile": {
    "id": "user_12345",
    "email": "user@example.com",
    "os": "Windows 11",
    "cpu": "Intel i9-13900K",
    "gpu": "NVIDIA RTX 3080",
    "ram_gb": 32,
    "programming_experience": "advanced",
    "robotics_experience": "beginner",
    "development_environment": "PyCharm",
    "primary_language": "Python",
    "learning_goals": ["robotics", "AI"],
    "created_at": "2023-01-01T00:00:00Z",
    "updated_at": "2023-01-02T00:00:00Z"
  }
}
```

**Status Codes:**
- 200: Success
- 400: Bad request (invalid input)
- 401: Unauthorized (invalid token)

---

### Personalization Endpoints

#### GET `/personalization/`

Retrieve the current user's personalization preferences.

**Headers:**
```
Authorization: Bearer {token}
```

**Response:**
```json
{
  "success": true,
  "message": "Personalization preferences retrieved successfully",
  "preferences": {
    "id": "pref_12345",
    "user_id": "user_12345",
    "content_difficulty": "intermediate",
    "preferred_examples": ["hardware-focused", "simulation"],
    "response_complexity": "balanced",
    "interaction_style": "guided",
    "learning_pace": "moderate",
    "personalization_preferences": {
      "include_advanced_topics": true,
      "include_practical_exercises": true
    },
    "created_at": "2023-01-01T00:00:00Z",
    "updated_at": "2023-01-01T00:00:00Z"
  }
}
```

**Status Codes:**
- 200: Success
- 401: Unauthorized (invalid token)

---

#### PUT `/personalization/`

Update the current user's personalization preferences.

**Headers:**
```
Authorization: Bearer {token}
Content-Type: application/json
```

**Request Body:**
```json
{
  "content_difficulty": "advanced",
  "preferred_examples": ["AI-focused", "real-robot"],
  "response_complexity": "detailed",
  "interaction_style": "problem-solving",
  "learning_pace": "fast",
  "personalization_preferences": {
    "include_advanced_topics": true,
    "include_practical_exercises": false
  }
}
```

**Response:**
```json
{
  "success": true,
  "message": "Personalization preferences updated successfully",
  "preferences": {
    "id": "pref_12345",
    "user_id": "user_12345",
    "content_difficulty": "advanced",
    "preferred_examples": ["AI-focused", "real-robot"],
    "response_complexity": "detailed",
    "interaction_style": "problem-solving",
    "learning_pace": "fast",
    "personalization_preferences": {
      "include_advanced_topics": true,
      "include_practical_exercises": false
    },
    "created_at": "2023-01-01T00:00:00Z",
    "updated_at": "2023-01-02T00:00:00Z"
  }
}
```

**Status Codes:**
- 200: Success
- 400: Bad request (invalid input)
- 401: Unauthorized (invalid token)

---

### Session Management Endpoints

#### GET `/session/status`

Check the current session status.

**Headers:**
```
Authorization: Bearer {token}
```

**Response:**
```json
{
  "valid": true,
  "user": {
    "id": "user_12345",
    "email": "user@example.com",
    "os": "Windows 10",
    "cpu": "Intel i7-12700K",
    "programming_experience": "intermediate",
    "robotics_experience": "beginner",
    "created_at": "2023-01-01T00:00:00Z",
    "updated_at": "2023-01-01T00:00:00Z"
  }
}
```

**Status Codes:**
- 200: Success
- 401: Unauthorized (invalid token)

---

#### POST `/session/refresh`

Refresh the current session token.

**Headers:**
```
Authorization: Bearer {token}
```

**Response:**
```json
{
  "success": true,
  "message": "Session refreshed successfully",
  "token": "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...",
  "expires_in": 1800
}
```

**Status Codes:**
- 200: Success
- 401: Unauthorized (invalid token)

---

## Error Handling

All API responses follow a consistent error format:

```json
{
  "success": false,
  "message": "Error message describing the issue",
  "error_code": "ERROR_CODE",
  "details": {
    "field": "specific error details"
  }
}
```

## Rate Limiting

All endpoints are subject to rate limiting to prevent abuse. The default limits are:

- Authentication endpoints: 10 requests per minute per IP
- Profile and personalization endpoints: 100 requests per minute per user
- Other endpoints: 1000 requests per minute per IP

## Security

- All sensitive data is encrypted in transit using HTTPS
- Passwords are hashed using bcrypt
- JWT tokens have a default expiration of 30 minutes
- All user inputs are validated and sanitized

## Performance

- API response times should be under 2 seconds in cloud environments
- API response times should be under 5 seconds in local environments
- Database queries are optimized with appropriate indexing