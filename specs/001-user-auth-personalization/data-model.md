# Data Model: Physical AI & Humanoid Robotics Textbook Signup/Signin & Personalization

## Overview
This document defines the data models for the signup/signin and personalization feature, including user profiles, session management, and personalization data structures.

## User Profile Model

### User Profile Entity
Represents a registered user with authentication credentials, hardware/software specifications, experience levels, and personalization preferences.

**Fields:**
- `user_id` (string, required): Unique identifier for the user (provided by Better-Auth)
- `email` (string, required): User's email address (validated format, unique)
- `created_at` (timestamp, required): Account creation timestamp
- `updated_at` (timestamp, required): Last profile update timestamp
- `os` (string, optional): Operating system (e.g., "Windows 10", "macOS 12", "Ubuntu 22.04")
- `cpu` (string, optional): CPU specifications (e.g., "Intel i7-12700K", "AMD Ryzen 7 5800X")
- `gpu` (string, optional): GPU specifications (e.g., "NVIDIA RTX 3080", "AMD RX 6700 XT")
- `ram_gb` (integer, optional): RAM in gigabytes (e.g., 16, 32, 64)
- `programming_experience` (string, optional): Programming experience level ("beginner", "intermediate", "advanced", "expert")
- `robotics_experience` (string, optional): Robotics experience level ("none", "beginner", "intermediate", "advanced")
- `development_environment` (string, optional): Preferred development environment (e.g., "Visual Studio Code", "PyCharm", "Vim")
- `primary_language` (string, optional): Primary programming language (e.g., "Python", "C++", "JavaScript")
- `learning_goals` (array, optional): Array of learning goals (e.g., ["robotics", "AI", "simulation"])

**Validation Rules:**
- Email must be valid email format
- RAM must be positive integer
- Experience levels must be from predefined enum values
- Learning goals must be from predefined list of valid values

**Relationships:**
- One-to-many with User Sessions
- One-to-many with Personalization Preferences

## Session Model

### Session Entity
Represents an authenticated user's active session with timeout management and security tokens.

**Fields:**
- `session_id` (string, required): Unique identifier for the session
- `user_id` (string, required): Reference to the associated user
- `created_at` (timestamp, required): Session creation timestamp
- `expires_at` (timestamp, required): Session expiration timestamp (30 minutes after last activity)
- `last_activity` (timestamp, required): Timestamp of last user activity
- `ip_address` (string, optional): IP address of the session origin
- `user_agent` (string, optional): User agent string of the client
- `is_active` (boolean, required): Whether the session is currently active

**Validation Rules:**
- Session must be associated with a valid user
- Expires_at must be after created_at
- Session must be renewed upon activity to extend timeout

**State Transitions:**
- Active → Expired: When timeout period elapses without activity
- Active → Revoked: When user explicitly logs out or session is invalidated
- Expired → Active: When user authenticates again

## Personalization Data Model

### Personalization Preference Entity
Represents user-specific parameters that influence content difficulty, examples, and chatbot response customization.

**Fields:**
- `preference_id` (string, required): Unique identifier for the preference record
- `user_id` (string, required): Reference to the associated user
- `content_difficulty` (string, required): Preferred content difficulty level ("adaptive", "beginner", "intermediate", "advanced")
- `preferred_examples` (array, optional): Array of preferred example types (e.g., ["hardware-focused", "simulation", "real-robot"])
- `response_complexity` (string, required): Preferred response complexity ("simple", "balanced", "detailed")
- `interaction_style` (string, required): Preferred interaction style ("guided", "exploratory", "problem-solving")
- `hardware_context` (object, optional): Context for hardware-relevant content based on user's hardware specs
- `learning_pace` (string, required): Preferred learning pace ("slow", "moderate", "fast")
- `created_at` (timestamp, required): Record creation timestamp
- `updated_at` (timestamp, required): Last update timestamp

**Validation Rules:**
- Content difficulty, response complexity, interaction style, and learning pace must be from predefined enum values
- Preferred examples must be from predefined list of valid values
- Associated user must exist

**Relationships:**
- Many-to-one with User Profile

## API Contract Models

### Authentication Request Model
**Endpoint**: POST `/api/auth/signup` and POST `/api/auth/signin`

**Fields:**
- `email` (string, required): User's email address
- `password` (string, required): User's password (minimum 8 characters with complexity)
- `profile` (object, optional): Profile information object with fields from User Profile Entity

### Authentication Response Model
**Endpoint**: POST `/api/auth/signup` and POST `/api/auth/signin`

**Fields:**
- `success` (boolean, required): Whether the operation was successful
- `message` (string, required): Status message
- `user_id` (string, required): User identifier
- `session_id` (string, required): Session identifier
- `auth_token` (string, required): Authentication token
- `profile` (object, optional): User profile information

### Profile Update Request Model
**Endpoint**: PUT `/api/profile/`

**Fields:**
- `os` (string, optional): Operating system
- `cpu` (string, optional): CPU specifications
- `gpu` (string, optional): GPU specifications
- `ram_gb` (integer, optional): RAM in gigabytes
- `programming_experience` (string, optional): Programming experience level
- `robotics_experience` (string, optional): Robotics experience level
- `development_environment` (string, optional): Preferred development environment
- `primary_language` (string, optional): Primary programming language
- `learning_goals` (array, optional): Array of learning goals

### Profile Response Model
**Endpoint**: GET `/api/profile/` and PUT `/api/profile/`

**Fields:**
- `success` (boolean, required): Whether the operation was successful
- `message` (string, required): Status message
- `profile` (object, required): User profile object with all profile fields

### Personalization Request Model
**Endpoint**: PUT `/api/personalization/`

**Fields:**
- `content_difficulty` (string, optional): Preferred content difficulty level
- `preferred_examples` (array, optional): Array of preferred example types
- `response_complexity` (string, optional): Preferred response complexity
- `interaction_style` (string, optional): Preferred interaction style
- `learning_pace` (string, optional): Preferred learning pace

### Personalization Response Model
**Endpoint**: GET `/api/personalization/` and PUT `/api/personalization/`

**Fields:**
- `success` (boolean, required): Whether the operation was successful
- `message` (string, required): Status message
- `preferences` (object, required): Personalization preferences object with all preference fields

## Security Considerations

### Data Encryption
- All sensitive user data stored in Neon Postgres must be encrypted at rest
- Authentication tokens must be encrypted in transit using HTTPS
- Passwords must be hashed using industry-standard algorithms (BCrypt, Argon2, etc.)

### Access Control
- User profiles should only be accessible by the owning user or authorized administrators
- Session data must be validated for ownership before access
- Personalization preferences should only be modifiable by the owning user

### Data Retention
- Session data should be automatically purged after expiration
- Inactive sessions should be cleaned up periodically
- User profile data should follow GDPR compliance for deletion rights

## Performance Considerations

### Indexing Strategy
- Index on `user_id` for all related entities
- Index on `session.expires_at` for efficient cleanup of expired sessions
- Index on `user.updated_at` for tracking active users

### Caching Strategy
- Frequently accessed personalization preferences should be cached
- Session validation results should be cached briefly to reduce database load
- User profile data for active users should be cached to improve response times

## Migration Considerations

### Schema Evolution
- Support for adding new optional profile fields without breaking existing functionality
- Backward compatibility for API endpoints during version transitions
- Safe migration of existing user data when schema changes occur