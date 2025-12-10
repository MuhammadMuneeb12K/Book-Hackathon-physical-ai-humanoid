---
id: 6
title: Sp.Analyze Prerequisite Failure
stage: general
date: 2025-12-09
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: main
user: user
command: /sp.analyze
labels: ["error", "prerequisite"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
- null
tests:
- null
---

## Prompt

/sp.analyze

## Response snapshot

Feature directory not found. Please run `/sp.specify` first to create the feature structure before running `/sp.analyze`.

## Outcome

- ✅ Impact: Command failed due to missing prerequisites.
- 🧪 Tests: N/A
- 📁 Files: N/A
- 🔁 Next prompts: User needs to run /sp.specify
- 🧠 Reflection: The /sp.analyze command requires a feature directory to be present with spec.md, plan.md, and tasks.md. The initial check-prerequisites.ps1 script correctly identified the missing directory.

## Evaluation notes (flywheel)

- Failure modes observed: Prerequisite check failure: missing feature directory.
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Ensure /sp.analyze command checks for feature directory existence and guides user to /sp.specify if missing.