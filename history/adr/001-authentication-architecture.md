# ADR-001: Authentication Architecture Using Better-Auth

## Status
Accepted

## Date
2025-12-12

## Context
The Physical AI & Humanoid Robotics Textbook platform requires a secure authentication system for users (students grades 10-12, early college students, instructors). We need to implement signup/signin functionality that integrates with the existing RAG chatbot and textbook platform while maintaining security best practices and ease of use for the educational audience.

## Decision
We will use Better-Auth as the authentication provider with email/password authentication only for the initial implementation. This decision centralizes authentication concerns, provides industry-standard security practices, and offers a simple onboarding experience for students.

## Alternatives Considered
1. **Custom JWT-based authentication**: Would require implementing all authentication logic, security measures, and best practices from scratch, increasing development time and security risks.
2. **Auth.js (NextAuth.js)**: Would be tightly coupled to Next.js ecosystem, but our frontend is integrated with Docusaurus which may not have optimal integration.
3. **Clerk**: Would introduce an external dependency with potential costs and vendor lock-in, though with excellent developer experience.
4. **OAuth providers only (Google/Microsoft)**: Would create barriers for students who may not have institutional accounts, and privacy concerns for educational use.

## Rationale
Better-Auth was selected because it provides:
- Proven security practices and regular updates
- Easy integration with our FastAPI backend
- Educational-friendly onboarding (email/password)
- Self-hosting capability for privacy compliance
- Good documentation and community support
- Alignment with project constitution security requirements

## Consequences
### Positive
- Reduced development time for authentication features
- Industry-standard security practices out-of-the-box
- Self-hosting capability for data privacy compliance
- Centralized authentication management
- Easier maintenance and security updates

### Negative
- Additional dependency to manage
- Potential vendor lock-in if we extend beyond basic features
- Need to maintain Better-Auth infrastructure
- Limited to Better-Auth's feature roadmap

## References
- plan.md: Authentication section and technical context
- research.md: Security considerations and technology evaluation
- data-model.md: User profile and session models