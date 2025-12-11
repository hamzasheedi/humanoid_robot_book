# Better-Auth API Reference

## Overview
Better-Auth is an authentication library that provides user management functionality including registration, login, and session management.

## Registration API Endpoint
- **URL**: `POST /api/auth/register`
- **Content-Type**: `application/json`

## Request Format
```json
{
  "email": "user@example.com",
  "password": "securePassword123",
  "profile": {
    "name": "John Doe",
    "age": 30,
    "role": "student"
  }
}
```

## Response Formats

### Success Response (200)
```json
{
  "user_id": "unique-user-id",
  "email": "user@example.com",
  "created_at": "2023-01-01T00:00:00Z"
}
```

### Duplicate Email Error (409)
```json
{
  "error": "Email already exists"
}
```

### Validation Error (400)
```json
{
  "error": "Validation failed",
  "message": "Password must be at least 8 characters"
}
```

## Common Error Codes
- `200`: Success - User created
- `400`: Bad Request - Validation error
- `409`: Conflict - Email already exists
- `500`: Internal Server Error - API failure

## Password Requirements
- Minimum 8 characters
- Should contain uppercase, lowercase, number, and special character (if enforced)
- No common passwords