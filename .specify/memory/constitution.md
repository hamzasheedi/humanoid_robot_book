<!-- 
Sync Impact Report:
- Version change: 1.0.0 → 1.1.0
- Modified principles: Added 5 principles from user input
- Added sections: Core Principles (5), Key Standards, Constraints, Success Criteria, Validation and Cascading Rules
- Templates requiring updates: [.specify/templates/plan-template.md] ⚠ pending, [.specify/templates/spec-template.md] ⚠ pending, [.specify/templates/tasks-template.md] ⚠ pending
- Follow-up TODOs: None
-->
# Textbook for Physical AI & Humanoid Robotics Constitution

## Core Principles

### I. Accuracy
All technical claims, algorithms, and instructions must be verified against official documentation, primary sources (ROS 2, NVIDIA Isaac, Gazebo, Unity), and peer-reviewed robotics literature.

### II. Clarity
Content must be understandable to advanced high school to early college students (grades 10–12), with computer science or engineering background.

### III. Reproducibility
Code examples, simulations, and exercises must run as described in the instructions on supported hardware or cloud setups.

### IV. Rigor
Preference for peer-reviewed or official technical sources; industry-standard practices must be followed.

### V. Engagement
Include visual diagrams, step-by-step tutorials, and practical exercises to reinforce embodied intelligence concepts.

## Key Standards

### Technical Accuracy
All ROS 2, Gazebo, Unity, and NVIDIA Isaac commands or scripts must be tested and verified on the stated hardware or cloud setup.

### Citation
Minimum 50% of references must be peer-reviewed publications, official SDK/API documentation, or authoritative textbooks. Citation format: IEEE or APA style.

### Plagiarism
Zero tolerance. All content must be original or properly cited.

### Code & Simulation Clarity
Code snippets must include explanations of input/output, expected behavior, and error handling.

### Hardware/Software Specifications
Explicitly state requirements for each module (GPU, CPU, RAM, OS, Edge AI kits, sensors, robot models).

### Learning Outcomes Alignment
Each chapter/module must clearly map to specific learning outcomes and weekly objectives.

## Constraints

### Book Length
250–350 pages, including diagrams, tutorials, and exercises.

### Modules
Must cover ROS 2, Digital Twin (Gazebo & Unity), NVIDIA Isaac, VLA integration, and Capstone project.

### Hardware/Cloud
Provide separate instructions for On-Premise Lab, Ether Lab (Cloud-native), and Economy Jetson Student Kit.

### Simulation & Deployment
All examples must include steps for both simulated and physical deployment.

### Accessibility
Include alternative instructions for students without high-end RTX GPUs (cloud options).

## Success Criteria

- All technical examples reproduce results on test hardware or cloud setup.
- Citations meet minimum standards, plagiarism = 0%.
- Students following the book can complete the Capstone project (simulated humanoid with conversational AI).
- Diagrams and tutorials enhance clarity and engagement (tested via peer/student review).
- Book deployed on Docusaurus and GitHub Pages using Spec-Kit Plus, fully navigable.

## Validation and Cascading Rules

### Specification Alignment
All module specifications must include: required hardware, simulation instructions, ROS 2 package templates, sensor integration, and expected outcomes.

### Plan Alignment
Lesson plans, weekly breakdowns, and exercises must reflect hardware and cloud constraints, including VLA latency issues.

### Implementation Alignment
Code examples and diagrams must strictly follow tested pipelines; deviation requires explicit note and justification.

## Governance

This constitution governs all project decisions and supersedes any conflicting practices. Amendment requires explicit documentation of rationale and approval by project leads. All specifications, plans, and implementations must verify compliance with these principles. Complexity must be justified with clear benefits to the learning experience.

**Version**: 1.1.0 | **Ratified**: TODO(RATIFICATION_DATE): Date of original adoption | **Last Amended**: 2025-12-08