# ADR-002: Multi-Tier Hardware Strategy for Educational Accessibility

**Status**: Accepted  
**Date**: 2025-12-08  
**Author**: Qwen Code

## Context

The AI Robotics Textbook must accommodate students with varying levels of access to hardware resources. Some students will have access to high-end workstations suitable for simulation and development, others will need cloud-based solutions, and some will only have access to entry-level hardware like the Jetson Student Kit. This decision ensures equitable access to the educational content regardless of economic circumstances.

## Decision

We will implement a three-tier hardware strategy supporting:

- **Tier 1 (On-Premise Lab)**: High-end workstations with dedicated GPUs for intensive simulation and development
- **Tier 2 (Ether Lab - Cloud)**: Cloud-based environments for students without powerful local hardware
- **Tier 3 (Economy Jetson Student Kit)**: Entry-level hardware for students with limited resources

Each tier will have specific instructions, configuration guides, and performance expectations tailored to its capabilities.

## Consequences

Positive:
- Maximum accessibility for students with different economic backgrounds
- Flexibility for different learning environments (classroom, remote, individual)
- Allows progression from simple to complex hardware as students advance
- Supports both simulation-first and hardware-first learning approaches

Negative/Risks:
- Increased testing and validation complexity across multiple configurations
- Need to maintain different instruction sets for each tier
- Potential performance differences could affect learning consistency
- Increased documentation overhead to support all hardware configurations

## Alternatives Considered

- **High-end only approach**: Supporting only powerful hardware but excluding many students
- **Cloud-only approach**: Simplifying deployment but requiring reliable internet access
- **Single tier approach**: Supporting only one type of hardware but limiting accessibility
- **Two-tier approach**: Reducing complexity but potentially excluding some students

## References

- plan.md: Technical Context and Constraints sections
- research.md: Decision on Hardware Tier Approach
- spec.md: Functional Requirements sections on hardware support