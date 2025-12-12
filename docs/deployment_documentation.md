# Deployment Documentation: Physical AI & Humanoid Robotics Textbook

## Overview

This document provides instructions for deploying the Physical AI & Humanoid Robotics Textbook platform with authentication and personalization features to both cloud and local environments.

## Prerequisites

### Cloud Deployment
- Cloud hosting account (AWS, Azure, GCP, or Vercel)
- Domain name (optional but recommended)
- SSL certificate (Let's Encrypt or provided by hosting service)
- Database service (Neon Postgres or equivalent)

### Local Deployment
- Docker and Docker Compose
- Python 3.11+
- Node.js 18+
- At least 4GB RAM available

## Environment Configuration

### Backend Environment Variables (.env)

Create a `.env` file in the `backend/` directory with the following variables:

```env
# Better-Auth Configuration
BETTER_AUTH_URL=https://your-domain.com
BETTER_AUTH_API_KEY=your_better_auth_api_key

# Neon Postgres Database
NEON_DB_URL=postgresql://username:password@ep-xxx.us-east-1.aws.neon.tech/dbname

# Cohere API for RAG
COHERE_API_KEY=your_cohere_api_key

# Qdrant Vector Database
QDRANT_URL=your_qdrant_cluster_url
QDRANT_API_KEY=your_qdrant_api_key

# OpenAI API
OPENAI_API_KEY=your_openai_api_key

# Backend API Configuration
API_KEY=your_backend_api_key
BACKEND_PORT=8000

# Session Configuration
SESSION_TIMEOUT_MINUTES=30

# JWT Secret
SECRET_KEY=your_secret_key_for_jwt_tokens
```

### Frontend Environment Variables (.env)

Create a `.env` file in the `frontend/` directory with the following variables:

```env
# Frontend Configuration
REACT_APP_BACKEND_URL=https://your-backend-domain.com
REACT_APP_API_KEY=your_api_key
REACT_APP_BETTER_AUTH_URL=https://your-better-auth-domain.com
```

## Cloud Deployment

### Vercel Deployment

1. **Prepare the frontend for Vercel deployment:**

```bash
cd frontend
npm run build
```

2. **Configure Vercel project:**
   - Go to [Vercel](https://vercel.com)
   - Create a new project
   - Link your repository
   - Set build command to `npm run build`
   - Set output directory to `build`
   - Add environment variables in the Vercel dashboard

3. **Deploy the backend:**
   - Use a service like Railway, Render, or Heroku for the backend
   - Set up your database and environment variables
   - Deploy the backend service

4. **Configure domain and SSL:**
   - Add your custom domain to both Vercel and backend hosting
   - Vercel will automatically provision SSL certificates

### AWS Deployment

1. **Using Elastic Beanstalk for backend:**
```bash
# Create application bundle
cd backend
zip -r app.zip . -x "*.git*" "venv/*" ".env" "__pycache__/*"

# Deploy to Elastic Beanstalk
aws elasticbeanstalk create-application-version --application-name textbook-backend --version-label v1.0.0 --source-bundle file://app.zip
```

2. **Using S3 + CloudFront for frontend:**
```bash
cd frontend
npm run build

# Upload to S3
aws s3 sync build/ s3://your-frontend-bucket --delete

# Invalidate CloudFront cache
aws cloudfront create-invalidation --distribution-id YOUR_DISTRIBUTION_ID --paths "/*"
```

3. **Using RDS for database:**
   - Create PostgreSQL RDS instance
   - Configure security groups to allow connections
   - Update database connection string

## Local Deployment

### Using Docker Compose

1. **Create docker-compose.yml:**

```yaml
version: '3.8'

services:
  backend:
    build:
      context: ./backend
      dockerfile: Dockerfile
    ports:
      - "8000:8000"
    environment:
      - NEON_DB_URL=${NEON_DB_URL}
      - COHERE_API_KEY=${COHERE_API_KEY}
      - QDRANT_URL=${QDRANT_URL}
      - QDRANT_API_KEY=${QDRANT_API_KEY}
      - OPENAI_API_KEY=${OPENAI_API_KEY}
      - API_KEY=${API_KEY}
      - SECRET_KEY=${SECRET_KEY}
    depends_on:
      - db
      - qdrant
    networks:
      - app-network

  frontend:
    build:
      context: ./frontend
      dockerfile: Dockerfile
    ports:
      - "3000:3000"
    environment:
      - REACT_APP_BACKEND_URL=http://localhost:8000
    depends_on:
      - backend
    networks:
      - app-network

  db:
    image: postgres:14
    environment:
      POSTGRES_DB: humanoid_robot_book
      POSTGRES_USER: postgres
      POSTGRES_PASSWORD: postgres
    ports:
      - "5432:5432"
    volumes:
      - postgres_data:/var/lib/postgresql/data
    networks:
      - app-network

  qdrant:
    image: qdrant/qdrant:latest
    ports:
      - "6333:6333"
    volumes:
      - qdrant_data:/qdrant/storage
    networks:
      - app-network

volumes:
  postgres_data:
  qdrant_data:

networks:
  app-network:
    driver: bridge
```

2. **Create Dockerfiles:**

Backend Dockerfile (`backend/Dockerfile`):
```Dockerfile
FROM python:3.11-slim

WORKDIR /app

COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

COPY . .

EXPOSE 8000

CMD ["uvicorn", "src.main:app", "--host", "0.0.0.0", "--port", "8000"]
```

Frontend Dockerfile (`frontend/Dockerfile`):
```Dockerfile
FROM node:18-alpine

WORKDIR /app

COPY package*.json ./
RUN npm ci --only=production

COPY . .

EXPOSE 3000

CMD ["npm", "start"]
```

3. **Run the local deployment:**
```bash
# From project root
docker-compose up --build
```

### Using Direct Installation

1. **Backend setup:**
```bash
cd backend
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
pip install -r requirements.txt
uvicorn src.main:app --reload --port 8000
```

2. **Frontend setup:**
```bash
cd frontend
npm install
npm start
```

## Configuration for Different Environments

### Development
```env
# Backend
BACKEND_PORT=8000
DEBUG=true
LOG_LEVEL=DEBUG

# Frontend
REACT_APP_BACKEND_URL=http://localhost:8000
```

### Staging
```env
# Backend
BACKEND_PORT=8000
DEBUG=false
LOG_LEVEL=INFO

# Frontend
REACT_APP_BACKEND_URL=https://staging.yourdomain.com
```

### Production
```env
# Backend
BACKEND_PORT=8000
DEBUG=false
LOG_LEVEL=WARNING
SESSION_TIMEOUT_MINUTES=30

# Frontend
REACT_APP_BACKEND_URL=https://yourdomain.com
```

## Monitoring and Logging

### Backend Monitoring
- Use the logging configuration in `src/utils/logging_config.py`
- Monitor API response times and error rates
- Set up alerts for authentication failures
- Track user engagement with personalization features

### Frontend Monitoring
- Implement client-side error tracking
- Monitor page load times
- Track user journey completion rates
- Monitor API call success/failure rates

## Performance Optimization

### Backend Optimization
- Database connection pooling
- Query optimization with proper indexing
- Caching for frequently accessed data
- Asynchronous processing for heavy operations

### Frontend Optimization
- Code splitting for faster initial load
- Image optimization
- Caching strategies
- Bundle size optimization

## Security Considerations

### Authentication Security
- Implement rate limiting on authentication endpoints
- Use secure JWT tokens with appropriate expiration
- Validate all user inputs
- Implement CSRF protection
- Use HTTPS for all communications

### Data Protection
- Encrypt sensitive data at rest
- Use parameterized queries to prevent SQL injection
- Implement proper access controls
- Regular security audits

## Troubleshooting

### Common Issues

#### Backend Not Starting
- Check that all environment variables are set
- Verify database connection
- Check port availability

#### Frontend Cannot Connect to Backend
- Verify backend is running and accessible
- Check CORS configuration
- Verify environment variables

#### Authentication Not Working
- Check Better-Auth configuration
- Verify API keys
- Check network connectivity

### Debugging Steps

1. Check logs for error messages
2. Verify environment variables
3. Test database connectivity
4. Check API endpoint accessibility
5. Review authentication flow

## Rollback Procedures

### Cloud Deployment Rollback
1. Use version control in your hosting platform
2. Maintain previous deployment artifacts
3. Use blue-green deployment strategy when possible

### Local Deployment Rollback
1. Keep previous Docker images
2. Use Git for code versioning
3. Maintain database migration history

## Maintenance

### Regular Maintenance Tasks
- Database cleanup of expired sessions
- Log rotation
- Security updates
- Performance monitoring
- Backup verification

### Backup Strategy
- Database backups (daily recommended)
- Configuration backups
- Code repository backups
- Environment variable backups

## Scaling Considerations

### Horizontal Scaling
- Load balancer configuration
- Database read replicas
- Caching layer implementation
- Microservice architecture considerations

### Vertical Scaling
- Server resource upgrades
- Database performance tuning
- Memory and CPU optimization