# ADR-007: Integration of Claude Code + Spec-Kit Plus

**Status**: Accepted  
**Date**: 2025-12-08  
**Author**: Qwen Code

## Context

The AI Robotics Textbook project must be developed efficiently while maintaining structure and ensuring reproducible code examples. The textbook needs to be "AI-native," meaning it should leverage AI tools for creation and maintenance. The project also needs to be well-structured with proper documentation and deployment capabilities. Selecting the right tools and workflows will significantly impact development speed and quality.

## Decision

We will use Claude Code for content creation and Spec-Kit Plus for project management, structure, and Docusaurus deployment. Claude Code will accelerate content creation with AI assistance, while Spec-Kit Plus will enforce plan adherence and enable atomic task execution. This combination provides AI-native content creation with structured project management and streamlined deployment to Docusaurus on GitHub Pages.

## Consequences

Positive:
- Accelerated writing with AI suggestions and assistance
- Structured project management with enforced workflow
- Streamlined deployment to Docusaurus on GitHub Pages
- Atomic task execution for better progress tracking
- AI-native features integrated into the development process

Negative/Risks:
- Authors must follow Spec-Kit Plus workflow strictly
- Checkpoints and tasks must be completed in sequence for consistency
- Dependency on specific tools that may need updates or maintenance
- Potential learning curve for team members unfamiliar with these tools
- Risk of over-dependence on AI tools for content creation

## Alternatives Considered

- **Manual Markdown writing + GitHub**: More error-prone, slower integration of AI-native features. Rejected due to increased manual effort and reduced AI-native capabilities.

- **Alternative AI-assisted editors**: May lack seamless integration with Docusaurus and Spec-Kit Plus. Rejected due to potentially reduced integration quality.

- **Claude Code with Spec-Kit Plus** (selected): Accelerates writing with AI suggestions while enforcing plan adherence and atomic task execution.

## References

- plan.md: Technical Context and Project Structure sections
- research.md: Key Findings on AI-native pedagogy
- spec.md: Success Criteria for deployment and AI-native features