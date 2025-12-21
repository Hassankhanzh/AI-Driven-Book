---
id: 1
title: Create Physical AI Robotics Constitution
stage: constitution
date: 2025-12-18
surface: agent
model: claude-haiku-4-5-20251001
feature: none
branch: main
user: Claude User
command: /sp.constitution
labels: [constitution,physical-ai,robotics,education]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - .specify/memory/constitution.md
tests:
 - none
---

## Prompt

```text
Project: Textbook for Teaching Physical AI & Humanoid Robotics

Core Principles

Technical Accuracy: All Physical AI and robotics concepts must align with real-world physics, robotics standards, and industry practices.

Spec-Driven Authoring: Content must follow a Spec-Kit Plus workflow using Claude Code with no undocumented deviations.

Educational Clarity: Clear, progressive instruction for senior undergraduate / graduate learners.

Reproducibility: All code, simulations, and workflows must be reproducible using documented tools.

User-Centric Design: Optimize readability, navigation, and learning experience.

Standards

Writing level: Flesch-Kincaid 11–13

No hallucinated APIs, tools, or research

Modular chapters with learning objectives

Framework: Docusaurus, deployed to GitHub Pages

Topics include Physical AI, ROS 2, Gazebo, Isaac, humanoid control, perception, and deployment

RAG Chatbot Requirements

Embedded in the book

Stack: OpenAI Agents/ChatKit, FastAPI, Neon Postgres, Qdrant Cloud (Free Tier)

Answers limited strictly to book content

Supports questions based only on user-selected text

Constraints

Single unified repository

Open-source only

No placeholder or incomplete sections

Fully documented and reproducible

Success Criteria

Published textbook on GitHub Pages

Verified, spec-compliant content

Functional embedded RAG chatbot

Reusable Spec-Kit Plus workflow
```

## Response snapshot

Created the project constitution for the Textbook for Teaching Physical AI & Humanoid Robotics with the following sections:

- Core Principles: Technical Accuracy, Spec-Driven Authoring, Educational Clarity, Reproducibility, User-Centric Design
- Technical Standards: Writing level, tech stack, topics, no hallucinations, modular structure
- Development Workflow: Content creation, review process, quality gates, documentation
- RAG Chatbot Requirements: Embedding, tech stack, scope limitation, functionality
- Constraints: Repository structure, completeness, licensing, documentation
- Success Criteria: Publication, content verification, functionality, workflow reusability
- Governance: Constitution authority, amendment process, compliance reviews, versioning policy, quality assurance

Version: 1.0.0 | Ratified: 2025-12-18 | Last Amended: 2025-12-18

## Outcome

- ✅ Impact: Created foundational project constitution with all specified principles and requirements
- 🧪 Tests: none
- 📁 Files: Updated .specify/memory/constitution.md with complete constitution
- 🔁 Next prompts: Continue with spec, plan, and tasks creation for the textbook project
- 🧠 Reflection: Constitution properly captures all required principles and constraints for the Physical AI & Robotics textbook project

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A