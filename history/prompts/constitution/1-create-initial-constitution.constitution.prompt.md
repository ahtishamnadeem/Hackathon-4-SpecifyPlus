---
id: 1
title: Create Initial Constitution
stage: constitution
date: 2025-12-16
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: main
user: user
command: /sp.constitution
labels: ["constitution", "governance", "spec-first"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - .specify/memory/constitution.md
 - .specify/templates/plan-template.md
 - .specify/templates/spec-template.md
 - .specify/templates/tasks-template.md
tests:
 - none
---

## Prompt

```text
/sp.constitution

Project: AI-Spec-Driven Technical Book with Embedded RAG Chatbot

Core principles:
- Spec-first, AI-native development
- Technical accuracy via official documentation
- Clarity for developers (CS background)
- Full reproducibility

Key standards:
- All claims and APIs must be verifiable
- Code must be complete, executable, and tested
- Book content and codebase must stay in sync
- RAG answers must be grounded only in indexed book content
- Selected-text QA must use only user-highlighted text

Technology stack:
- Authoring: Claude Code + Spec-Kit Plus
- Book: Docusaurus
- Hosting: GitHub Pages
- Backend: FastAPI
- RAG: OpenAI Agents / ChatKit SDKs
- Vector DB: Qdrant Cloud (Free Tier)
- SQL DB: Neon Serverless Postgres

Constraints:
- Public, reproducible GitHub repo
- No hard-coded secrets
- Modular ingestion, embedding, retrieval, and agent layers
- Successful GitHub Pages deployment
- Chatbot embedded in the book UI

Content requirements:
- Markdown chapters via Docusaurus
- Step-by-step a
```

## Response snapshot

Created the initial project constitution for the AI-Spec-Driven Technical Book with Embedded RAG Chatbot project. The constitution includes six core principles:

1. Spec-First, AI-Native Development
2. Technical Accuracy via Official Documentation
3. Full Reproducibility (NON-NEGOTIABLE)
4. RAG Grounding and Content Integrity
5. Modularity and Separation of Concerns
6. Public Reproducibility and Security

Also updated template files to align with the new constitution:
- Updated plan-template.md to include constitution check gates
- Updated spec-template.md to include constitution alignment in requirements
- Updated tasks-template.md to include constitution alignment in format

## Outcome

- ✅ Impact: Created foundational governance document for the project with 6 core principles
- 🧪 Tests: No tests required for constitution creation
- 📁 Files: Updated 4 files (.specify/memory/constitution.md and 3 templates)
- 🔁 Next prompts: Can proceed with feature specification using the new constitution
- 🧠 Reflection: Constitution provides clear governance framework for future development decisions

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Begin feature specification using the new constitution