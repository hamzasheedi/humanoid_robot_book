---
id: 001
title: enhance-constitution-rag-chatbot
stage: constitution
date: 2025-12-09
surface: agent
model: Qwen
feature: "none"
branch: "main"
user: Hamza
command: /sp.constitution
labels:
  - constitution
  - rag-chatbot
  - ui
  - textbook
links:
  - spec: "null"
  - ticket: "null"
  - adr: "null"
  - pr: "null"
files:
  - .specify/memory/constitution.md
tests: []
---

# Enhance Constitution with RAG Chatbot and UI Requirements

## PROMPT_TEXT

```
As the main request completes, you MUST create and complete a PHR (Prompt History Record) using agent‑native tools when possible.

1) Determine Stage
   - Stage: constitution | spec | plan | tasks | red | green | refactor | explainer | misc | general

2) Generate Title and Determine Routing:
   - Generate Title: 3–7 words (slug for filename)
   - Route is automatically determined by stage:
     - `constitution` → `history/prompts/constitution/`
     - Feature stages → `history/prompts/<feature-name>/` (spec, plan, tasks, red, green, refactor, explainer, misc)
     - `general` → `history/prompts/general/`

3) Create and Fill PHR (Shell first; fallback agent‑native)
   - Run: `.specify/scripts/bash/create-phr.sh --title "<title>" --stage <stage> [--feature <name>] --json`
   - Open the file and fill remaining placeholders (YAML + body), embedding full PROMPT_TEXT (verbatim) and concise RESPONSE_TEXT.
   - If the script fails:
     - Read `.specify/templates/phr-template.prompt.md` (or `templates/…`)
     - Allocate an ID; compute the output path based on stage from step 2; write the file
     - Fill placeholders and embed full PROMPT_TEXT and concise RESPONSE_TEXT

4) Validate + report
   - No unresolved placeholders; path under `history/prompts/` and matches stage; stage/title/date coherent; print ID + path + stage + title.
   - On failure: warn, don't block. Skip only for `/sp.phr`.

Enhance my existing constitution.md by adding only the parts needed for the Integrated RAG Chatbot and its UI. Do not rewrite or remove the existing Core Principles, Key Standards, Constraints, Success Criteria, Validation Rules, or Governance. Just:

Add a short section or bullet points mentioning the RAG Chatbot, including: FastAPI backend, Qdrant vector store, Neon Postgres database.

Include success criteria for the chatbot: context-aware answers from selected text, ≥95% accuracy, ≤2s response time for cloud, ≤5s for local.

Add a short section about Chatbot UI, emphasizing:

Simple and intuitive interface for students

Embedded inside the textbook (Docusaurus integration)

Supports text selection for context-aware answers

Visual clarity and accessibility (WCAG 2.1 AA)

Make all additions beginner-friendly—explain RAG and UI concepts in simple terms for students who may not know advanced AI.

Keep the document consistent with the existing style, formatting, and tone.
```

## RESPONSE_TEXT

Updated the project constitution to include RAG Chatbot and UI requirements while preserving existing content. Added sections for:
- RAG Chatbot Implementation with technology stack (FastAPI, Qdrant, Neon Postgres)
- Chatbot UI Requirements with accessibility and design principles
- Success criteria with specific performance metrics (≥95% accuracy, ≤2s cloud/≤5s local response time)
- Updated version to 1.2.0 with appropriate change log

## OUTCOME

Successfully enhanced the constitution with RAG chatbot and UI requirements while maintaining all existing sections. The additions are beginner-friendly and consistent with the document's style and tone.

## EVALUATION

The changes were successfully implemented as requested, adding the RAG chatbot and UI sections without modifying existing content. The new sections explain the technology stack and requirements in beginner-friendly terms appropriate for high school/early college students.