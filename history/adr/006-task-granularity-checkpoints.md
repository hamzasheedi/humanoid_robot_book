# ADR-006: Task Granularity and Checkpoints

**Status**: Accepted  
**Date**: 2025-12-08  
**Author**: Qwen Code

## Context

The AI Robotics Textbook project involves multiple complex components and requires coordination between research, writing, validation, and technical implementation. Large tasks with unclear acceptance criteria reduce visibility into progress, slow feedback loops, and risk accumulating errors that become expensive to fix. The project needs a systematic approach to break down work into manageable pieces while ensuring quality through checkpoints.

## Decision

We will break all work into atomic tasks of 15-30 minutes duration with explicit acceptance criteria and human review checkpoints. Each task will have clear, measurable outcomes that can be validated independently. Checkpoints will be inserted at critical integration points and before major milestones to ensure alignment and quality before proceeding. This approach balances progress tracking granularity with overhead minimization.

## Consequences

Positive:
- Clear visibility into project progress at granular level
- Rapid feedback loops for corrections and improvements
- Risk mitigation through early detection of issues
- Clear accountability for task completion
- Systematic quality validation through checkpoint reviews

Negative/Risks:
- Workflow requires strict discipline to review and approve each checkpoint
- Failure to adhere to checkpoint process can cause misalignment between research and writing
- Potential overhead from managing many small tasks
- Risk of losing sight of bigger picture due to focus on small tasks
- Coordination complexity may increase with many small deliverables

## Alternatives Considered

- **Larger tasks** (>45 minutes, multi-step): Rejected due to difficulty in review and validation, unclear progress tracking, and slower feedback loops.

- **Micro-tasks** (<10 minutes): Rejected due to excessive overhead from task management and review processes.

- **Atomic tasks with checkpoints** (selected): Balance between progress tracking and overhead; allows checkpoint control while maintaining reasonable progress.

## References

- plan.md: Implementation Phases and Dependencies sections
- research.md: Key Findings on project management best practices
- spec.md: Success Criteria for measurable outcomes