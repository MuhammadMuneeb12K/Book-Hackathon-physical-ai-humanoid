---
id: 0001
title: Initial project constitution setup
stage: constitution
date: 2025-12-13
surface: agent
model: gemini-1.5-flash
feature: none
branch: main
user: user
command: /sp.constitution
labels: ["constitution", "setup"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
- specs/constitution/code-quality.md
- specs/constitution/testing-validation.md
- specs/constitution/security-authentication.md
- specs/constitution/performance-scalability.md
- specs/constitution/ai-rag-safety.md
- specs/constitution/reusability-modularity.md
tests:
- null
---

## Prompt

/sp.constitution SYSTEM INSTRUCTIONS (MANDATORY)

You are running inside Gemini CLI with filesystem access.

STRICT RULES:
- This is a WRITE-ONCE operation
- Do NOT re-read any files you generate
- Do NOT modify existing files unless explicitly told
- Do NOT reference previous specs or folders
- Treat generated files as final artifacts

PERSISTENCE REQUIREMENTS:
1. Save THIS ENTIRE PROMPT to:
   /prompts/sp.constitution.md
2. Log this execution summary to:
   /history/sp.constitution.log

OUTPUT REQUIREMENTS:
- Create a folder: /specs/constitution
- Split the constitution into logical markdown files
- Use professional, enforceable language
- Make files commit-ready (no placeholders)
- Do NOT include implementation code
- Do NOT create checklists or tasks here

NOW EXECUTE THE FOLLOWING COMMAND:

/sp.constitution
📁 Required Folder Structure (Must Be Created)
bash
Copy code
/specs/constitution
 ├─ code-quality.md
 ├─ testing-validation.md
 ├─ security-authentication.md
 ├─ performance-scalability.md
 ├─ ai-rag-safety.md
 └─ reusability-modularity.md
📄 Content to Distribute Across Files
🔹 code-quality.md
md
Copy code
# Code Quality Standards

- TypeScript-first development with strict mode enabled
- Single Responsibility Principle enforced across components
- All async operations must include structured try/catch handling
- Naming conventions:
  - camelCase: variables and functions
  - PascalCase: classes and components
  - UPPER_SNAKE_CASE: constants
- JSDoc required for exported functions and complex logic
- Console logging prohibited in production environments
🔹 testing-validation.md
md
Copy code
# Testing & Validation Standards

- Unit tests required for critical business logic (target ≥80% coverage)
- Integration tests required for API endpoints and RAG pipelines
- Type safety enforced; `any` only allowed for untyped external libraries
- Manual testing required in light and dark modes before commits
- Edge cases must cover loading, empty, and error states
🔹 security-authentication.md
md
Copy code
# Security & Authentication

- BetterAuth must follow official session management best practices
- Secrets must be stored in environment variables only
- Rate limiting required on all public endpoints
- All user input must be validated and sanitized
- HTTPS enforcement mandatory in production
- XSS protection required for all user-generated content
🔹 performance-scalability.md
md
Copy code
# Performance & Scalability

- RAG responses must complete within 3 seconds
- Vector search scope must be restricted using Qdrant filters
- Lazy loading required for routes and heavy components
- Embeddings and frequent queries must be cached
- Frequently queried database fields must be indexed
- Initial JavaScript bundle size must remain under 300KB
🔹 ai-rag-safety.md
md
Copy code
# AI / RAG Reliability & Safety

- Context window limits must never be exceeded
- Graceful fallback responses required when retrieval fails
- All AI responses must include source citations
- Low-confidence retrievals must be filtered out
- Inappropriate queries and outputs must be blocked
- User input must be sanitized to prevent prompt injection
🔹 reusability-modularity.md
md
Copy code
# Reusability & Modularity

- DRY principles enforced via shared utilities
- Configuration must be externalized from code
- Feature modules must be independently deployable
- Clear API contracts required between modules
- Reusable UI components should be centralized
- Monorepo structure must support future extraction

## Response snapshot

Successfully created constitution files in `specs/constitution`.

## Outcome

- ✅ Impact: Created initial constitution files.
- 🧪 Tests: No tests were run for this action.
- 📁 Files: 6 new constitution files created.
- 🔁 Next prompts: None.
- 🧠 Reflection: PHR creation via agent tools due to script execution failure on Windows.

## Evaluation notes (flywheel)

- Failure modes observed: Bash script execution failed on Windows.
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A
