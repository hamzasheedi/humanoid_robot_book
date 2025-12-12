# ADR-006: Dual Deployment Architecture (Cloud + Local)

## Status
Accepted

## Date
2025-12-12

## Context
The Physical AI & Humanoid Robotics Textbook platform needs to support both cloud-based and local deployments to accommodate students with varying technical infrastructure. The system must function effectively in both environments while maintaining consistent performance and security standards.

## Decision
Implement a dual deployment architecture that supports both cloud and local environments with the same codebase and configuration. All services will be designed to work in both environments with configurable parameters for different deployment scenarios. Authentication, personalization, and RAG features will be available in both environments with appropriate performance optimizations.

## Alternatives Considered
1. **Cloud-only deployment**: Would exclude students with limited internet access or institutional restrictions on cloud services.
2. **Local-only deployment**: Would limit scalability and maintenance, require individual updates on each deployment.
3. **Separate codebases for cloud/local**: Would increase maintenance burden and create inconsistency between environments.
4. **Hybrid with cloud-based personalization only**: Would create inconsistent user experience between environments.

## Rationale
Dual deployment was chosen because it provides:
- Maximum accessibility for students with different infrastructure
- Flexibility for educational institutions with varying policies
- Consistent functionality across environments
- Ability to support offline use cases
- Scalability of cloud with reliability of local deployment
- Aligns with project constitution accessibility requirements

## Consequences
### Positive
- Maximum accessibility across different user environments
- Flexibility for different educational institution policies
- Consistent user experience across deployment types
- Ability to function in low-connectivity situations
- Scalable cloud option with local reliability backup

### Negative
- Increased complexity in testing and deployment processes
- Need to maintain performance in resource-constrained local environments
- Challenge of synchronizing data between environments if needed
- Increased infrastructure costs for cloud deployment
- Potential for subtle differences between environments

## References
- plan.md: Performance goals and deployment considerations
- research.md: Deployment strategies and scalability considerations
- spec.md: Performance requirements for cloud and local deployments