---
id: 0001
title: Clarifying Work Cessation
stage: general
date: 2025-12-07
surface: agent
model: claude-sonnet-4-5-20251101
feature: none
branch: feat/update-constitution
user: user
command: user_query
labels: ["process", "guidance"]
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

where we stop our work

## Response snapshot

As Claude Code, I continue working until a task is completed, I require further input or clarification from you (the user), or when a specific workflow like "plan mode" is explicitly exited. I also proactively create a Prompt History Record for every interaction to track our progress.

## Outcome

- ✅ Impact: Clarified process for ending work.
- 🧪 Tests: No tests run for this type of query.
- 📁 Files: No files modified or created in response to the user's query.
- 🔁 Next prompts: User to provide next task.
- 🧠 Reflection: PHR successfully created for a general process question.

## Evaluation notes (flywheel)

- Failure modes observed: null
- Graders run and results (PASS/FAIL): null
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): null
