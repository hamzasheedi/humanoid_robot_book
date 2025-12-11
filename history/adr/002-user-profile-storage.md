# ADR-002: User Profile Storage in Neon Postgres

## Status
Accepted

## Date
2025-12-12

## Context
The platform needs to store user profile information (hardware specs, software environment, experience levels) that will be used for personalization of textbook content and RAG chatbot responses. This data needs to be persisted securely and accessed efficiently for personalization algorithms while maintaining privacy compliance.

## Decision
Store user profile data in Neon Postgres database with encryption at rest and in transit. Profile data will be stored as optional fields alongside user authentication references, with all sensitive data encrypted using industry-standard encryption.

## Alternatives Considered
1. **Local JSON storage per user**: Would complicate synchronization, backup, and querying across users; difficult to implement personalization algorithms that compare user profiles.
2. **Separate document database (MongoDB)**: Would add complexity with additional database system to manage; Neon Postgres already handles our other data needs.
3. **Storage in Qdrant vector database**: Would be inappropriate as profile data is structured metadata, not vector embeddings for RAG retrieval.
4. **Hybrid approach (Postgres + Redis cache)**: Would add complexity prematurely; caching can be added later if needed.

## Rationale
Neon Postgres was selected because it provides:
- ACID compliance for data integrity
- Strong security features with encryption capabilities
- Integration with existing backend stack
- Efficient querying for personalization algorithms
- Compliance with privacy regulations (GDPR, etc.)
- Consistency with existing data storage approach

## Consequences
### Positive
- Consistent data storage architecture with existing system
- Strong data integrity and security guarantees
- Efficient querying for personalization algorithms
- Simplified operations with single database system
- Compliance-ready with encryption and access controls

### Negative
- Additional database load for profile queries
- Schema evolution complexity if profile fields change frequently
- Need to manage encryption keys securely
- Potential performance bottlenecks if personalization queries become complex

## References
- plan.md: User profile storage section and technical context
- research.md: Data storage and security considerations
- data-model.md: User profile entity schema