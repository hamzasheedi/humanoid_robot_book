# ADR-003: API Gateway Between Docusaurus, FastAPI, and Better-Auth

## Status
Accepted

## Date
2025-12-12

## Context
The system needs to integrate a Docusaurus-based frontend textbook interface with a FastAPI backend that handles authentication, personalization, and integration with Better-Auth. This requires a clear API gateway pattern to manage communication between different technology stacks and external services.

## Decision
Implement a clear API gateway pattern with FastAPI backend serving as the primary API server. Docusaurus frontend will communicate with FastAPI backend via HTTPS API calls, with FastAPI handling integration with Better-Auth and other external services (Cohere, Qdrant). The backend will serve as a unified interface to all external services.

## Alternatives Considered
1. **Direct frontend to external services**: Would expose API keys and credentials to client-side, creating security vulnerabilities.
2. **Multiple independent API gateways**: Would increase complexity and potential failure points; harder to maintain consistent security policies.
3. **Server-side rendering with backend-driven UI**: Would reduce flexibility of Docusaurus platform and increase server load.
4. **Event-driven microservices**: Would add unnecessary complexity for this scale of application; premature architectural sophistication.

## Rationale
The chosen approach provides:
- Centralized security controls and authentication validation
- Unified error handling and logging
- Consistent data formatting between services
- Clear separation of concerns between frontend and backend
- Maintains Docusaurus flexibility while securing external service access

## Consequences
### Positive
- Centralized security and authentication validation
- Simplified frontend development with consistent API contracts
- Better control over external service access and rate limiting
- Clear separation of concerns between components
- Easier monitoring and debugging of API traffic

### Negative
- Additional network hop between frontend and external services
- Potential bottleneck at the backend API layer
- Increased coupling between frontend and backend API contracts
- Single point of failure if backend API goes down

## References
- plan.md: Architecture sketch and component interactions
- research.md: Integration strategies and security considerations
- contracts/: API contract definitions for auth, profile, personalization