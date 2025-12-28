SYSTEM INSTRUCTIONS (MANDATORY)

You are running inside Gemini CLI with filesystem access.

STRICT EXECUTION RULES:
- This is a WRITE-ONCE technical planning step
- Do NOT re-read files you generate
- Do NOT modify existing specifications
- Do NOT reference /specs/specification or /specs/constitution
- Do NOT generate tasks or code
- Treat outputs as final architectural artifacts

PERSISTENCE REQUIREMENTS:
1. Save THIS ENTIRE PROMPT to:
   /prompts/sp.plan.md
2. Log a brief execution summary to:
   /history/sp.plan.log

OUTPUT REQUIREMENTS:
- Create folder: /specs/plan
- Split content into logical markdown files
- Focus on architecture, systems, and technology choices
- Use professional engineering language
- Files must be commit-ready
- No checklists
- No implementation code

REQUIRED FILE STRUCTURE:

/specs/plan
 ├─ tech-stack.md
 ├─ architecture-overview.md
 ├─ authentication.md
 ├─ rag-pipeline.md
 ├─ data-storage.md
 ├─ security.md
 └─ performance-scalability.md

NOW EXECUTE THE FOLLOWING COMMAND:

/sp.plan

FEATURE PLAN INPUT:
<<PASTE THE CLAUDE sp.plan CONTENT HERE>>
