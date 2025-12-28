SYSTEM INSTRUCTIONS (MANDATORY)

You are running inside Gemini CLI with filesystem access.

Your task is to DEFINE and CREATE the COMPLETE PROJECT STRUCTURE only.
This is a STRUCTURE-DESIGN step, not content generation.

STRICT RULES:
- Do NOT generate implementation code
- Do NOT generate long documentation
- Do NOT create plans, tasks, or logic
- Do NOT repeat or loop
- Do NOT ask follow-up questions
- Do NOT modify existing files if they exist
- Treat this as a ONE-TIME scaffold operation

PERSISTENCE REQUIREMENTS:
1. Save THIS ENTIRE PROMPT to:
   /prompts/project-structure.md
2. Log a brief summary to:
   /history/project-structure.log

OUTPUT REQUIREMENTS:
- Create folders and placeholder markdown files ONLY
- Each file may contain a 1–2 line description of its purpose
- Keep content minimal and professional
- Everything must be commit-ready

PROJECT CONTEXT:
This project is an Intelligent Documentation Assistant for a humanoid robotics book.
It uses RAG, conversational AI, authentication, and personalized learning.
Specs are created using sp.constitution, sp.specify, sp.plan, sp.tasks.

REQUIRED FINAL STRUCTURE:

/specs
 ├─ constitution/
 │   └─ principles.md
 │
 ├─ specification/
 │   ├─ purpose.md
 │   ├─ user-problems.md
 │   ├─ features.md
 │   ├─ user-experience.md
 │   └─ justification.md
 │
 ├─ plan/
 │   ├─ tech-stack.md
 │   ├─ architecture-overview.md
 │   ├─ authentication.md
 │   ├─ rag-pipeline.md
 │   ├─ data-storage.md
 │   ├─ security.md
 │   └─ performance-scalability.md
 │
 └─ tasks/
     └─ README.md

/prompts
 ├─ sp.constitution.md
 ├─ sp.specify.md
 ├─ sp.plan.md
 └─ project-structure.md

/history
 ├─ sp.constitution.log
 ├─ sp.specify.log
 ├─ sp.plan.log
 └─ project-structure.log

NOW EXECUTE:
Create this directory structure and minimal placeholder files.