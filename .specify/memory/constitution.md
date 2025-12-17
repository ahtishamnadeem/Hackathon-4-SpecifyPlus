<!--
Sync Impact Report:
Version change: N/A -> 1.0.0
Added sections: Core Principles (6), Additional Constraints, Development Workflow, Governance
Removed sections: None
Modified principles: None (new constitution)
Templates requiring updates: ⚠ pending - plan-template.md, spec-template.md, tasks-template.md, command files
Follow-up TODOs: None
-->
# AI-Spec-Driven Technical Book with Embedded RAG Chatbot Constitution

## Core Principles

### I. Spec-First, AI-Native Development
Every feature and implementation begins with a clear specification; All development follows AI-assisted patterns using Claude Code and Spec-Kit Plus; Development process must be spec-driven with clear acceptance criteria before implementation.

### II. Technical Accuracy via Official Documentation
All claims, APIs, and technical details must be verifiable against official documentation; Code examples must be complete, executable, and tested against actual implementations; Third-party integrations must reference authoritative sources and version-specific documentation.

### III. Full Reproducibility (NON-NEGOTIABLE)
All code must be complete, executable, and tested in isolation; Book content and codebase must stay in sync through automated validation; Environment setup must be fully documented with reproducible steps and dependencies.

### IV. RAG Grounding and Content Integrity
RAG answers must be grounded only in indexed book content; Selected-text QA must use only user-highlighted text without hallucination; Vector database content must be synchronized with published book chapters and updates.

### V. Modularity and Separation of Concerns
Modular ingestion, embedding, retrieval, and agent layers must be independently deployable; Backend services (FastAPI) must maintain clean separation from frontend (Docusaurus); Database interactions must be abstracted through well-defined interfaces.

### VI. Public Reproducibility and Security
All code must be suitable for public GitHub repository without hard-coded secrets; Environment variables and configuration must be properly abstracted; Deployment processes must be documented for public hosting on GitHub Pages.

## Additional Constraints
Technology stack requirements: Claude Code + Spec-Kit Plus for authoring, Docusaurus for book, GitHub Pages for hosting, FastAPI backend, OpenAI Agents / ChatKit SDKs for RAG, Qdrant Cloud for vector database, Neon Serverless Postgres for SQL.
Deployment policies: Successful GitHub Pages deployment required, chatbot embedded in book UI, all components must be publicly accessible and reproducible.

## Development Workflow
Code review requirements: All claims must be verifiable, code must be tested, book-content synchronization verified; Testing gates: Unit tests for backend components, integration tests for RAG functionality, UI tests for chatbot integration; Deployment approval: Successful CI/CD pipeline including security scanning and content validation.

## Governance
This constitution supersedes all other development practices; All PRs and reviews must verify compliance with spec-first methodology and technical accuracy requirements; Amendments require documentation of changes, approval from project maintainers, and migration plan for existing artifacts.

**Version**: 1.0.0 | **Ratified**: 2025-12-16 | **Last Amended**: 2025-12-16
