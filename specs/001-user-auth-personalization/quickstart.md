# Quickstart Guide: Physical AI & Humanoid Robotics Textbook Signup/Signin & Personalization

## Overview
This guide provides instructions for getting started with the signup/signin and personalization feature for the Physical AI & Humanoid Robotics Textbook platform.

## Prerequisites
- Node.js 18+ (for frontend development)
- Python 3.11+ (for backend development)
- Docker and Docker Compose (for local environment setup)
- Better-Auth account and API credentials
- Neon Postgres database setup
- Cohere API key for RAG integration

## Environment Setup

### 1. Clone the Repository
```bash
git clone <repository-url>
cd <repository-directory>
```

### 2. Backend Setup
```bash
# Navigate to backend directory
cd backend

# Create virtual environment
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Install dependencies
pip install -r requirements.txt

# Set up environment variables
cp .env.example .env
# Edit .env with your Better-Auth, Neon Postgres, and Cohere credentials
```

### 3. Frontend Setup
```bash
# Navigate to frontend directory
cd frontend

# Install dependencies
npm install

# Set up environment variables
cp .env.example .env
# Edit .env with your backend API URL
```

## Configuration

### Backend Configuration
1. Configure Better-Auth integration in `backend/src/config/auth.py`:
   ```python
   BETTER_AUTH_URL = os.getenv("BETTER_AUTH_URL")
   BETTER_AUTH_API_KEY = os.getenv("BETTER_AUTH_API_KEY")
   ```

2. Configure database connection in `backend/src/config/database.py`:
   ```python
   DATABASE_URL = os.getenv("DATABASE_URL")
   ```

3. Configure RAG integration in `backend/src/config/rag.py`:
   ```python
   COHERE_API_KEY = os.getenv("COHERE_API_KEY")
   ```

### Frontend Configuration
1. Update API endpoints in `frontend/src/config/api.js`:
   ```javascript
   const API_BASE_URL = process.env.REACT_APP_API_BASE_URL || 'http://localhost:8000';
   ```

## Running the Application

### Development Mode

#### Backend
```bash
# Activate virtual environment
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Run the backend server
cd backend
uvicorn src.main:app --reload --port 8000
```

#### Frontend
```bash
# Install dependencies if not done already
npm install

# Run the frontend development server
cd frontend
npm start
```

### Production Mode
```bash
# Build the frontend
cd frontend
npm run build

# Serve the application
cd ..
docker-compose up --build
```

## API Endpoints

### Authentication
- `POST /api/auth/signup` - Create a new user account
- `POST /api/auth/signin` - Authenticate an existing user
- `POST /api/auth/signout` - End the current session

### Profile Management
- `GET /api/profile/` - Retrieve user profile information
- `PUT /api/profile/` - Update user profile information

### Personalization
- `GET /api/personalization/` - Retrieve personalization preferences
- `PUT /api/personalization/` - Update personalization preferences

### Session Management
- `GET /api/session/status` - Check current session status
- `POST /api/session/refresh` - Refresh the current session

## User Workflow

### 1. Signup Process
1. User visits the signup page
2. User enters email and password
3. User optionally provides profile information (OS, CPU, GPU, RAM, experience level)
4. System validates input and creates Better-Auth account
5. System stores profile information in Neon Postgres
6. System creates session and returns authentication token

### 2. Signin Process
1. User visits the signin page
2. User enters email and password
3. System authenticates with Better-Auth
4. System retrieves profile information from Neon Postgres
5. System creates session and returns authentication token

### 3. Personalization Usage
1. System retrieves user profile information
2. System adapts textbook content based on experience level
3. System customizes RAG chatbot responses based on hardware and experience
4. System adjusts content complexity based on user preferences

## Testing

### Backend Tests
```bash
# Run all backend tests
cd backend
pytest

# Run specific test suite
pytest tests/unit/
pytest tests/integration/
pytest tests/security/
```

### Frontend Tests
```bash
# Run all frontend tests
cd frontend
npm test

# Run specific test suite
npm test -- --testPathPattern=auth
npm test -- --testPathPattern=profile
```

### End-to-End Tests
```bash
# Run integration tests
cd backend
pytest tests/e2e/
```

## Security Best Practices

### Authentication
- Always use HTTPS in production
- Implement rate limiting for authentication endpoints
- Use strong password requirements
- Implement secure session management with timeout

### Data Protection
- Encrypt sensitive data at rest and in transit
- Validate all user inputs
- Use parameterized queries to prevent SQL injection
- Implement proper access controls

### Session Management
- Use secure, HTTP-only cookies
- Implement CSRF protection
- Set appropriate session timeouts
- Invalidate sessions on logout

## Troubleshooting

### Common Issues

#### Authentication Failures
- Verify Better-Auth API credentials
- Check network connectivity to Better-Auth service
- Ensure proper CORS configuration

#### Database Connection Issues
- Verify Neon Postgres connection string
- Check database credentials
- Confirm database is running and accessible

#### RAG Integration Problems
- Validate Cohere API key
- Check RAG service availability
- Verify content indexing status

#### Session Problems
- Confirm session timeout settings
- Check cookie configuration
- Verify cross-origin settings

### Debugging Tips
1. Enable detailed logging for authentication services
2. Monitor API response times for performance issues
3. Check application logs for error details
4. Use browser developer tools to inspect API requests

## Deployment

### Cloud Deployment
1. Configure cloud infrastructure with required services
2. Set up environment variables with production credentials
3. Deploy backend and frontend applications
4. Configure SSL certificates and domain routing
5. Set up monitoring and alerting

### Local Deployment
1. Use Docker Compose for containerized deployment
2. Configure local environment variables
3. Run database migrations
4. Verify all services are accessible

## Next Steps

1. Complete the full authentication flow with your specific requirements
2. Customize personalization algorithms for your use case
3. Integrate with your existing RAG chatbot and textbook platform
4. Implement additional security measures as needed
5. Conduct thorough testing in your target environments