# Better-Auth Sign-In API Reference

## Overview
Better-Auth provides user authentication functionality including login with email and password.

## Login API Endpoint
- **URL**: `POST /api/auth/login`
- **Content-Type**: `application/json`

## Request Format
```json
{
  "email": "user@example.com",
  "password": "securePassword123"
}
```

## Response Formats

### Success Response (200)
```json
{
  "token": "jwt-authentication-token",
  "user": {
    "id": "user_abc123def456",
    "email": "user@example.com",
    "name": "John Doe",
    "role": "student",
    "created_at": "2023-01-01T00:00:00Z"
  }
}
```

### Invalid Credentials Error (401)
```json
{
  "error": "Invalid credentials"
}
```

### Account Not Found Error (404)
```json
{
  "error": "Account not found"
}
```

### Validation Error (422)
```json
{
  "error": "Validation failed",
  "message": "Email and password are required"
}
```

## Common Error Codes
- `200`: Success - User authenticated
- `401`: Unauthorized - Invalid credentials
- `404`: Not Found - Account doesn't exist
- `422`: Validation Error - Invalid input format
- `500`: Internal Server Error - API failure

## Token Usage
- The returned token should be used in Authorization header for subsequent API calls
- Format: `Authorization: Bearer <token>`
- Tokens typically expire after a set period (configurable in Better-Auth)