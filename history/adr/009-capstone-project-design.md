# ADR-009: Capstone Project Design

**Status**: Accepted  
**Date**: 2025-12-08  
**Author**: Qwen Code

## Context

The AI Robotics Textbook needs a capstone project that demonstrates the integration of all modules and reinforces embodied intelligence concepts. The capstone must serve as a culminating experience that validates student learning across all covered technologies (ROS 2, Digital Twin, NVIDIA Isaac, VLA). It should demonstrate end-to-end application of the learned concepts and provide a tangible project students can showcase.

## Decision

The capstone project will integrate Vision-Language-Action (VLA), NVIDIA Isaac for perception and navigation, and ROS 2 for control systems in a simulated humanoid robot that performs autonomous tasks. Students will implement a simulated humanoid agent capable of receiving voice commands, processing them using AI models, planning navigation paths, perceiving its environment, and executing physical manipulation tasks. The project will be designed to be executable both in simulation and with optional deployment to physical robots where available.

## Consequences

Positive:
- Full integration reinforces embodied intelligence principles
- Demonstrates module interdependencies and system integration
- Provides a tangible project for students to showcase
- Validates learning across all covered technologies
- Supports both simulation and physical deployment approaches
- Creates a comprehensive example of AI-robotics integration

Negative/Risks:
- Capstone must be staged in textbook tasks sequentially after all modules
- Code examples must be compatible with both simulation and edge devices
- Complexity may be challenging for some students
- Requires integration of all previously learned concepts
- Testing and validation become more complex with full integration
- Potential hardware compatibility issues during deployment

## Alternatives Considered

- **Smaller isolated projects per module**: Would not demonstrate system integration. Rejected because it doesn't provide a comprehensive demonstration of all learned concepts working together.

- **Cloud-only execution**: Would introduce latency issues and disconnect from physical AI principles. Rejected due to potential real-time performance issues and reduced educational value.

- **Integrated VLA, Isaac, and ROS 2 capstone** (selected): Full integration reinforces embodied intelligence principles and demonstrates module interdependencies.

## References

- plan.md: Summary and Implementation Phases sections
- research.md: Key Findings on hardware integration guidelines
- spec.md: User Story 5 on Capstone Project and Functional Requirement FR-005
- constitution.md: Success Criteria for Capstone project