<!--
Sync Impact Report:
- Version change: 2.1.0 → 2.1.1
- Added sections: Amendment: Signup & Personalization (new section with 9 subsections)
- Templates requiring updates: [.specify/templates/plan-template.md] ⚠ pending, [.specify/templates/spec-template.md] ⚠ pending, [.specify/templates/tasks-template.md] ⚠ pending
- Follow-up TODOs: None
-->
# Constitution: Integrated RAG Chatbot and Physical AI & Humanoid Robotics Textbook

## Preamble

This constitution serves as the governing document for the development of the Physical AI & Humanoid Robotics Textbook with an integrated Retrieval-Augmented Generation (RAG) chatbot. The constitution ensures that all decisions related to development, integration, testing, and deployment align with the project's mission to provide an interactive, accurate, and engaging learning experience.

The target audience includes students in grades 10–12, early college students, and instructors. Our primary objectives are to deliver interactive learning experiences, ensure technical accuracy, maintain clarity for the intended audience, guarantee reproducible results, and promote engagement through innovative pedagogical methods.

## Core Principles

### I. Accuracy
All technical claims, algorithms, and instructions must be verified against official documentation, primary sources (ROS 2, NVIDIA Isaac, Gazebo, Unity), and peer-reviewed robotics literature. All answers provided by the RAG chatbot must be grounded strictly in textbook content and verified through official sources. The RAG pipeline must retrieve and synthesize information accurately from the vector store without hallucinations.

### II. Clarity
Content and user interfaces must be understandable to advanced high school to early college students (grades 10–12), with computer science or engineering background. All code snippets, explanations, and interactions must be clearly articulated. The chatbot must respond in a manner that is comprehensible and pedagogically sound for the target audience, providing clear explanations rather than overly technical jargon.

### III. Reproducibility
Code examples, simulations, exercises, and RAG chatbot responses must run and function as described in the instructions on supported hardware and cloud setups. All backend services (FastAPI, Qdrant, Neon Postgres) must be consistently deployable across environments. Students must be able to reproduce all results and interactions successfully.

### IV. Rigor
Preference for peer-reviewed or official technical sources; industry-standard practices must be followed. All RAG implementations must use official SDKs and APIs (Cohere, Qdrant, Neon Postgres) with proper error handling and validation. Backend services must follow security best practices and performance standards.

### V. Engagement
Include visual diagrams, step-by-step tutorials, practical exercises, and interactive chatbot responses to reinforce embodied intelligence concepts. The RAG chatbot must provide contextual, helpful responses that maintain conversation flow and encourage deeper learning. UI elements must be intuitive and promote continued engagement.

### VI. Performance Excellence
Implementation must meet specified performance requirements of ≤2 seconds response time for cloud deployment and ≤5 seconds for local deployment with ≥95% accuracy on textbook content.

### VII. Accessibility & Setup Simplicity
Support both cloud and local deployments to accommodate students with varying technical infrastructure. The system must be accessible to students with different levels of technical expertise and different hardware capabilities.

## Key Standards

### Content Verification
All ROS 2, Gazebo, Unity, NVIDIA Isaac, VLA, and RAG chatbot implementations must be verified against official documentation and primary sources. Textbook content must be validated by subject matter experts.

### Citations & References
Minimum 50% of references must be peer-reviewed publications, official SDK/API documentation, or authoritative textbooks. Citation format: IEEE or APA style. Chatbot responses must include appropriate citations to textbook sections.

### Plagiarism
Zero tolerance. All content must be original or properly cited. AI-generated responses must be clearly attributed as such but grounded in original textbook content.

### Code & UI Clarity
Code snippets must include explanations of input/output, expected behavior, and error handling. UI must be intuitive, responsive, and accessible (WCAG 2.1 AA). The chatbot interface must provide clear feedback and error states.

### Hardware/Software Specifications
Explicitly state requirements for each module (GPU, CPU, RAM, OS, Edge AI kits, sensors, robot models, cloud resources for RAG processing).

### Learning Outcome Alignment
Each chapter/module and chatbot interaction must clearly map to specific learning outcomes and weekly objectives. The RAG system must reinforce these learning objectives through contextual responses.

## Constraints

### Scope Limitations
The chatbot must answer only based on textbook content; no external knowledge unless explicitly allowed. The RAG system must not fabricate information or provide general knowledge outside the textbook.

### Latency & Performance
Define acceptable response times: ≤2s for cloud deployment, ≤5s for local deployment. Backend services must maintain 99% uptime during academic hours.

### Deployment Environments
Specify supported browsers, OS, cloud platforms, and hardware compatibility. Include options for both cloud and local deployment to accommodate different student environments.

### Accessibility
Include fallback options for students without high-end GPUs (cloud options). The UI must comply with WCAG 2.1 AA standards and support screen readers and keyboard navigation.

### Security
Protect all API keys, database credentials, user data, and textbook content. Server-side storage of credentials, encrypted transmission, and proper authentication protocols.

## Success Criteria

- Chatbot answers ≥95% of textbook-based questions accurately
- Contextual responses are provided for user-selected text
- Interactive UI integrates seamlessly into Docusaurus and Spec-Kit Plus
- Peer/student review >85% satisfaction
- All backend services (FastAPI, Neon DB, Qdrant) operate securely and reliably
- Deployed textbook is fully functional on GitHub Pages with the integrated chatbot
- Response times are ≤2s for cloud and ≤5s for local deployment
- All technical examples reproduce results on test hardware or cloud setup
- Citations meet minimum standards, plagiarism = 0%
- Students following the book can complete the Capstone project (simulated humanoid with conversational AI)
- Diagrams and tutorials enhance clarity and engagement (tested via peer/student review)
- Book deployed on Docusaurus and GitHub Pages using Spec-Kit Plus, fully navigable
- Interactive RAG chatbot responds to queries with ≥95% accuracy on textbook content
- Chatbot provides context-aware answers maintaining conversation flow
- Students can successfully interact with the embedded chatbot interface

## Backend & Chatbot-Specific Additions

### Environment Variables Management
All API keys for FastAPI, Neon Postgres, Qdrant, and Cohere must be stored securely in environment variables. No credentials should be hardcoded. Implement proper secret management for deployment environments.

### RAG Chatbot Workflow
Define the complete workflow: text selection → Cohere embeddings generation → vector retrieval (Qdrant) → context extraction → answer generation (Cohere) → citation attribution. Each step must be logged and monitored for quality and accuracy.

### Logging Requirements
Log all queries, Cohere embeddings generation, response accuracy, system errors, and user interactions. Maintain logs for performance monitoring, accuracy validation, and system troubleshooting. Implement proper privacy controls on user data.

### Content Update Procedures
Establish procedures for updating textbook content and regenerating RAG embeddings via Cohere. Create automated workflows to index new content and invalidate outdated embeddings while maintaining system availability.

### Error Handling
Implement graceful error handling for unanswerable queries, RAG system failures, and backend service outages. Provide helpful error messages to users and fallback mechanisms when the primary RAG system is unavailable.

## UI & Accessibility

### Interface Requirements
The chatbot UI must provide responsive design, text selection highlight capabilities, and comprehensive feedback mechanisms. Include clear indicators for loading, error, or no-answer states. The UI must explicitly interact with the Cohere backend for responses.

### Accessibility Standards
Ensure compliance with WCAG 2.1 AA standards: keyboard navigation, screen reader support, appropriate contrast ratios, and alternative text for visual elements. The UI must be usable by students with disabilities.

### User Experience Design
Provide intuitive navigation, clear visual hierarchy, and consistent interaction patterns. Include features like history, bookmarking, and session management to enhance learning continuity.

## Validation & Cascading Rules

### Specification Alignment
Verify all modules, code snippets, and UI elements match the defined standards. All RAG implementations must align with the accuracy and performance requirements outlined in this constitution, specifically using Cohere for embeddings and answer generation.

### Plan Alignment
Weekly plans, exercises, and chatbot scenarios must reflect hardware, cloud, and latency constraints. Development plans must account for the performance requirements and security measures for Cohere API integration.

### Implementation Alignment
All backend, embeddings, and retrieval workflows must follow tested procedures; deviations require documented justification. Code implementations must pass all validation checks before merging, with special attention to Cohere API usage and response handling.

## Amendment: Signup & Personalization

### Preamble

This amendment establishes the foundational principles, standards, and governance for user authentication, profile collection, and personalization features within the Physical AI & Humanoid Robotics Textbook platform. The amendment ensures secure, privacy-compliant user onboarding while enabling personalized educational experiences through profile-based content adaptation and chatbot interactions.

### Core Principles

- **Secure Authentication**: All user authentication must be handled through Better-Auth with industry-standard security practices, including secure password storage, session management, and protection against common vulnerabilities.
- **Privacy-First Profile Collection**: User profile data collection must be minimal, purposeful, and transparent, with clear consent mechanisms and data usage policies.
- **Personalization with Purpose**: Personalization features must enhance the educational experience without compromising user privacy or creating filter bubbles that limit learning opportunities.
- **Data Sovereignty**: Users must retain control over their personalization data, with clear opt-out mechanisms and data portability options.

### Key Standards

- **Data Collection**: Collect only hardware/software profile information relevant to educational content adaptation (OS, CPU, GPU, RAM, programming experience, robotics experience).
- **User Privacy**: Implement end-to-end encryption for profile data transmission and storage, with compliance to GDPR and applicable privacy regulations.
- **Integration Standards**: All authentication and personalization features must integrate seamlessly with existing RAG chatbot, session management, and master_agent orchestration systems.

### Constraints

- **Security**: All user data must be stored in Neon Postgres with appropriate encryption at rest and in transit.
- **Scope**: Personalization features are limited to textbook content adaptation and chatbot response customization; no external data sharing without explicit consent.
- **Performance**: Authentication and personalization features must not significantly impact platform performance or increase response latency beyond acceptable thresholds.

### Success Criteria

- User can successfully sign up/sign in through Better-Auth integration with proper validation
- User profile collected and validated during signup process with clear consent
- Textbook content personalized based on user's technical background and experience level
- Chatbot responds using personalization data to provide relevant examples and explanations
- All backend services (Better-Auth, Neon, Cohere, Qdrant) work securely and efficiently
- User profile data is stored and accessed with appropriate security measures
- Personalization features enhance rather than limit educational experience

### Backend & Chatbot Additions

- **API Integration**: Better-Auth endpoints must be properly integrated with FastAPI backend, including proper error handling and response formatting
- **Session Handling**: Session state management through Neon Postgres with proper timeout and security measures
- **Profile Persistence**: User profile data stored in Neon Postgres with appropriate indexing for efficient retrieval
- **RAG Integration**: Personalization data must be accessible to RAG system for context-aware responses
- **Master Agent Coordination**: Authentication and personalization flows must be properly orchestrated through master_agent

### UI & Accessibility

- **Signup Form Standards**: Accessible signup forms compliant with WCAG 2.1 AA standards with proper labeling and error messaging
- **Personalization Settings**: Clear UI controls for users to view, edit, or opt out of personalization features
- **Accessibility Compliance**: All personalized content must maintain accessibility standards for users with disabilities
- **Progressive Enhancement**: Core functionality must be available without JavaScript, with personalization as an enhancement

### Validation & Cascading Rules

- **Data Validation**: All profile data must be validated for format, range, and reasonableness before storage
- **Session Consistency**: Authentication tokens and session data must remain consistent across all platform components
- **Personalized Content Correctness**: Personalized content must maintain educational accuracy and not compromise learning objectives
- **Security Validation**: All authentication and personalization features must pass security validation before deployment

### Governance

- **Review Schedule**: Personalization features and privacy practices must be reviewed quarterly
- **Update Process**: Changes to personalization algorithms require stakeholder approval and user notification
- **Compliance Monitoring**: Ongoing monitoring of privacy regulation compliance and security best practices
- **User Feedback**: Regular collection and analysis of user feedback on personalization features
- **Data Retention**: Clear policies for data retention and deletion with automated enforcement

## Governance

This constitution governs all project decisions and supersedes any conflicting practices. Amendment requires explicit documentation of rationale and approval by project leads. All specifications, plans, and implementations must verify compliance with these principles. Complexity must be justified with clear benefits to the learning experience.

Changes to core principles, standards, and deployment procedures must be tracked with version history. The project team must conduct regular reviews to ensure continued compliance with this constitution.

**Version**: 2.1.1 | **Ratified**: 2025-12-12 | **Last Amended**: 2025-12-12