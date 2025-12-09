# Research Summary: RAG Chatbot Integration

## Decision: Architecture Pattern
**Rationale**: Selected a micro-frontend approach with separate backend and frontend components to maintain clear separation of concerns. The backend (FastAPI) handles RAG processing, database operations, and security, while the frontend (Docusaurus/React) manages the user interface and textbook integration.
**Alternatives considered**: 
- Monolithic application: Would have mixed concerns and made deployment more complex
- Serverless approach: Would have increased latency and made vector operations more complex

## Decision: Vector Database
**Rationale**: Qdrant selected as the vector database for its efficient similarity search capabilities, active development, and good Python integration for the RAG pipeline. It's also suitable for the free tier as specified in requirements.
**Alternatives considered**: 
- Pinecone: More expensive and potentially over-engineered for this use case
- Weaviate: Good alternative but Qdrant has better performance characteristics for this scale
- FAISS: Requires more manual management of vector storage and retrieval

## Decision: Database for Logs
**Rationale**: Neon Postgres chosen for storing chat logs and accuracy data due to its compatibility with the existing project constitution, good performance, and serverless capabilities that work well for both cloud and local deployments.
**Alternatives considered**: 
- SQLite: Simpler for local deployment but lacks features needed for analytics
- MongoDB: More complex setup and less suitable for structured log data

## Decision: Textbook Content Structure
**Rationale**: Docusaurus is already specified in the project requirements and integrates well with React components. Textbook content will be structured in a modular way to facilitate RAG chunking and retrieval.
**Alternatives considered**: 
- Custom CMS: Would require additional development time
- Markdown-only approach: Less flexible for rich content integration

## Decision: Embedding Model
**Rationale**: OpenAI embeddings chosen for their proven quality and ease of integration. The project already mentions using OpenAI agents/ChatKit SDK.
**Alternatives considered**: 
- Sentence transformers: Free but potentially less accurate for technical content
- Cohere embeddings: Good alternative but OpenAI has better documentation and integration

## Decision: UI Integration Method
**Rationale**: React component embedded within Docusaurus pages provides seamless integration without requiring navigation away from the textbook content. This approach maintains the "embedded" requirement from the specification.
**Alternatives considered**: 
- Separate modal window: Would create context switching
- iframe integration: Would limit interaction and styling capabilities