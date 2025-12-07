# ADR-004: Research Approach — Research-Concurrent vs Research-First

**Status**: Accepted  
**Date**: 2025-12-08  
**Author**: Qwen Code

## Context

The AI Robotics Textbook project requires extensive research on Physical AI, ROS 2, Gazebo, Unity, NVIDIA Isaac, humanoid robotics, and Vision-Language-Action systems. The approach to integrating research with writing will significantly impact project timeline and quality. Waiting for complete research before beginning writing may delay discovery of gaps between research and narrative, potentially leading to misalignment between research findings and textbook content.

## Decision

We will adopt a research-concurrent workflow where research and writing happen in parallel. Each module will complete its research phase before starting the writing phase, with checkpoints to validate alignment between research findings and narrative content. This approach allows immediate integration of research findings into the textbook, provides iterative improvements based on research discoveries, and enables early feedback on content quality.

## Consequences

Positive:
- Immediate integration of research findings into the narrative
- Iterative improvements based on research discoveries
- Early feedback on content quality and alignment
- Reduced risk of misalignment between research and writing
- Faster identification of gaps or issues in research

Negative/Risks:
- Requires careful checkpoint discipline to prevent cascading errors
- Research and writing phases may still have dependencies that need careful coordination
- Risk of needing rework if research findings significantly change the narrative direction
- More complex project management to track both research and writing progress

## Alternatives Considered

- **Research-First**: Complete all research before beginning writing. Pros: Full coverage before writing, more comprehensive research. Cons: Delays writing, discovery of gaps happens late, potential misalignment between research and narrative.

- **Research-Concurrent**: Research and write in parallel with checkpoints (selected). Pros: Immediate integration, iterative improvements, early feedback. Cons: Requires careful checkpoint discipline to prevent cascading errors.

## References

- plan.md: Summary and Module Sequencing sections
- research.md: Key Findings sections on research methodology
- spec.md: Constraint section on research requirements