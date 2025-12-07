# ADR-005: Section and Module Ordering

**Status**: Accepted  
**Date**: 2025-12-08  
**Author**: Qwen Code

## Context

The AI Robotics Textbook must be structured to support student learning, with prerequisite knowledge properly established before advanced topics are introduced. The ordering of modules directly impacts the student's ability to build on previous knowledge and understand the relationships between different concepts. The course needs to progress logically from fundamentals to advanced integrated applications.

## Decision

We will order textbook modules in the following sequence:

1. Robotic Nervous System (ROS 2) - Foundational concepts for robot communication
2. Digital Twin (Gazebo & Unity) - Simulation environments for testing concepts
3. AI-Robot Brain (NVIDIA Isaac) - AI perception and navigation capabilities
4. Vision-Language-Action (VLA) - Advanced integration of perception and command
5. Capstone Project - Integration of all previous modules

This ordering ensures smooth knowledge build-up, aligns with week-by-week course breakdown, and supports checkpoint evaluation for each module before proceeding to the next.

## Consequences

Positive:
- Smooth knowledge build-up with proper prerequisite establishment
- Clear alignment with week-by-week course structure
- Supports checkpoint evaluation and phased development
- Students can gradually build understanding from simple to complex concepts
- Enables effective assessment and progress tracking

Negative/Risks:
- Each module's tasks must be completed sequentially, creating dependency chains
- Early delays in module completion will propagate downstream
- Students cannot skip ahead to advanced topics without completing prerequisites
- May limit flexibility for instructors with different pedagogical approaches

## Alternatives Considered

- **Logical/chronological order** (selected): Follows natural progression from fundamentals to advanced integration, ensuring proper prerequisite knowledge.

- **Randomized order for flexibility**: Allow modules to be taught in any sequence. Rejected because students would lack necessary prerequisite knowledge, leading to poor learning outcomes.

- **Topic-based ordering**: Group by robot subsystems rather than development flow. Rejected because it would interrupt the logical progression of complexity.

## References

- plan.md: Implementation Phases and Dependencies sections
- research.md: Key Findings on educational content best practices
- spec.md: User Story ordering and module dependencies