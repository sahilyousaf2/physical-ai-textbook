---
id: 0002
title: Fix Docusaurus Route Conflicts and Navigation Issues
stage: general
date: 2025-12-09
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: 001-physical-ai-textbook
user: homie
command: /sp.phr
labels: ["docusaurus", "routing", "navigation", "textbook", "fix"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - C:/Users/homie/Desktop/new-ai-book/physical-ai-textbook/my-website/docusaurus.config.ts
 - C:/Users/homie/Desktop/new-ai-book/physical-ai-textbook/my-website/docs/tutorial-basics
 - C:/Users/homie/Desktop/new-ai-book/physical-ai-textbook/my-website/docs/tutorial-extras
tests:
- none
---

## Prompt

Fix Docusaurus Route Conflicts and Navigation Issues --stage general

## Response snapshot

Fixed duplicate route warnings by changing routeBasePath from '/' to '/docs' in docusaurus.config.ts and removing conflicting tutorial directories that were causing route conflicts.

## Outcome

- ✅ Impact: Resolved duplicate route warnings and fixed navigation issues in the Physical AI textbook
- 🧪 Tests: Verified site compiles and runs without duplicate route warnings
- 📁 Files: Updated docusaurus.config.ts and removed tutorial directories
- 🔁 Next prompts: None needed - issue resolved
- 🧠 Reflection: Route conflicts in Docusaurus often occur when routeBasePath settings conflict with file structure

## Evaluation notes (flywheel)

- Failure modes observed: Duplicate route warnings were preventing proper navigation
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A
