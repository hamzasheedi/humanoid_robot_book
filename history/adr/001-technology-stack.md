# ADR-001: Technology Stack for AI Robotics Textbook

**Status**: Accepted  
**Date**: 2025-12-08  
**Author**: Qwen Code

## Context

The AI Robotics Textbook project requires a technology stack that supports both educational content delivery and hands-on robotics development. The system needs to support multiple modules: ROS 2 fundamentals, Digital Twin simulation (Gazebo/Unity), NVIDIA Isaac perception, and Vision-Language-Action integration. The chosen technologies must be accessible to high school and early college students while maintaining professional-grade capabilities for advanced concepts.

## Decision

We will use the following integrated technology stack:

- **Content Management**: Docusaurus for textbook deployment on GitHub Pages
- **Primary Programming Language**: Python 3.10+ for ROS 2 components and educational examples
- **Robotics Framework**: ROS 2 Humble Hawksbill (LTS version)
- **Simulation Environments**: Gazebo and Unity 2022.3 LTS
- **AI Integration**: NVIDIA Isaac Sim, OpenAI Whisper for speech recognition
- **Frontend/Deployment**: JavaScript/TypeScript for interactive elements, deployed via GitHub Pages

## Consequences

Positive:
- Python's readability makes it ideal for educational purposes
- ROS 2 Humble Hawksbill's long-term support ensures stability for the textbook lifecycle
- Supporting both Gazebo (open-source) and Unity (commercial) maximizes accessibility
- Docusaurus enables rich documentation with interactive elements
- The combination provides a complete educational pipeline from simulation to deployment

Negative/Risks:
- Multiple simulation environments increase testing and maintenance complexity
- Some components require commercial licenses (Unity, NVIDIA Isaac)
- Students may face complex setup processes across multiple platforms
- Dependency on multiple external frameworks increases integration challenges

## Alternatives Considered

- **Single simulation approach**: Using only Gazebo or Unity to simplify maintenance but limiting accessibility
- **Multiple programming languages**: Using C++ for performance but reducing educational accessibility
- **Custom content management**: Building a custom system but increasing development time and complexity
- **Different ROS distribution**: Using Rolling Ridley for newest features but sacrificing stability

## References

- plan.md: Technical Context and Project Structure sections
- research.md: Research Summary sections on technology choices
- spec.md: Functional Requirements sections