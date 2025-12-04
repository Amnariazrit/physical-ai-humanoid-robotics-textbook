<!-- Sync Impact Report
Version change: 0.0.0 (initial) -> 1.0.0
Modified principles: All principles significantly modified/defined.
Added sections: All sections and principles are new as this is an initial constitution.
Removed sections: None.
Templates requiring updates:
- .specify/templates/plan-template.md: ⚠ pending
- .specify/templates/spec-template.md: ⚠ pending
- .specify/templates/tasks-template.md: ⚠ pending
- .specify/templates/commands/sp.constitution.md: ✅ updated
- .specify/templates/commands/sp.phr.md: ⚠ pending
- .specify/templates/commands/sp.specify.md: ⚠ pending
- .specify/templates/commands/sp.plan.md: ⚠ pending
- .specify/templates/commands/sp.tasks.md: ⚠ pending
- .specify/templates/commands/sp.implement.md: ⚠ pending
- .specify/templates/commands/sp.git.commit_pr.md: ⚠ pending
- .specify/templates/commands/sp.clarify.md: ⚠ pending
- .specify/templates/commands/sp.checklist.md: ⚠ pending
- .specify/templates/commands/sp.analyze.md: ⚠ pending
- .specify/templates/commands/sp.adr.md: ⚠ pending
Follow-up TODOs: None.
-->
# AI-Native Textbook — Physical AI & Humanoid Robotics Constitution

## CORE PRINCIPLES

### 1.1 Accuracy
- All factual claims must be correct, verifiable, and based on primary sources.
- Robotics facts must be validated against ROS 2, Gazebo, Unity, NVIDIA Isaac, and hardware vendor documentation.
- Hardware specifications must be exact and reflect real models.

### 1.2 Rigor
- All technical explanations must be precise and suitable for an upper-undergraduate computer science audience.
- Mathematical, algorithmic, and architectural descriptions must be technically correct.
- Provide citations whenever introducing facts, statistics, or performance claims.

### 1.3 Reproducibility
- All instructions must be executable as written.
- Code examples must run without modification unless stated.
- Version numbers must be specified for ROS 2, Isaac Sim, Gazebo, and other tools.

### 1.4 Clarity
- Writing level must be Flesch-Kincaid Grade 10–12.
- Explanations must be concise, structured, and technically accessible.

## SOURCE & CITATION RULES

### 2.1 Citations
- All factual statements require APA-style citations.
- Include in-text citations and a reference list for each chapter.
- Use peer-reviewed sources whenever possible (target ≥ 50%).

### 2.2 Source Types
Allowed:
- Peer-reviewed papers (ICRA, IROS, RSS, IJRR, Nature Machine Intelligence)
- Official documentation (ROS, Gazebo, Unity, NVIDIA Isaac, Unitree)
- Academic books
- Research datasets or benchmarks

Not allowed:
- Blogs without strong authority
- Social media posts
- Unverified user comments

### 2.3 Plagiarism
- All writing must be original.
- Paraphrasing must remain technically correct and cite original sources.
- 0% plagiarism tolerance.

## TEXTBOOK CONTENT REQUIREMENTS

### 3.1 Chapter Structure
Each chapter must include:
- Introduction
- Technical explanation sections
- Diagrams or tables where appropriate
- Code examples (ROS 2, Gazebo, Isaac)
- Exercises or practical tasks
- APA-formatted references

### 3.2 Word Count
- Total book: 50,000–80,000 words
- Each module chapter: approx. 5,000–7,000 words

### 3.3 Forbidden Output
Claude must NOT produce:
- Uncited facts
- Fictional references
- Placeholder citations
- Inaccurate technical shortcuts
- Unsupported claims

## TECHNICAL IMPLEMENTATION RULES

### 4.1 Docusaurus
- All content must be structured for Docusaurus v3.x.
- Use correct frontmatter for each markdown file.
- Maintain consistent directory structure and sidebar definitions.

### 4.2 Spec-Kit Plus
- All chapters must follow spec requirements.
- Claude must generate machine-readable spec files on request.
- Traceability must be maintained: requirement → content → verification.

### 4.3 AI Enhancements
Claude must support generating, designing, or debugging:
- RAG chatbot implementation (OpenAI Agents, FastAPI, Qdrant, Neon)
- Personalization buttons and logic
- Urdu translation toggles
- BetterAuth signup/signin flows
- Claude Code Subagents and Skills definitions

## STYLE & FORMATTING RULES

### 5.1 Markdown
- Use clean markdown compatible with Docusaurus.
- Do not use HTML except when unavoidable.
- Code blocks must specify language.

### 5.2 Figures & Tables
- Provide captions and sources.
- Do not embed copyrighted images without permission.

### 5.3 Tone
- Professional
- Precise
- Engineering-focused
- No conversational filler

## WORKFLOW & GOVERNANCE RULES

### 6.1 Responsiveness
- Claude must always follow this constitution before generating content.
- If a user request conflicts with the constitution, respond with a correction and acceptable alternatives.

### 6.2 Versioning
- When generating code or configs, respect typical git best practices.
- Provide clear commit messages if asked.

### 6.3 Validation
Claude must ensure:
- All claims have supporting citations.
- No broken references.
- Code builds (unless explicitly hypothetical).

## SUCCESS CRITERIA

Claude must produce content that meets all of the following:

✔ Accurate, source-verified claims
✔ APA citations with ≥ 50% peer-reviewed sources
✔ Original writing with 0% plagiarism
✔ Clear technical explanations (Grade 10–12)
✔ Reproducible examples + correct versioning
✔ Docusaurus-ready markdown structure
✔ Spec-Kit Plus compliant specs
✔ AI-feature support (RAG, personalization, Urdu translation)

If any success criteria are unmet, Claude must revise the output.

## Governance

This constitution supersedes all other practices. Amendments require documentation, approval, and a migration plan. All PRs/reviews must verify compliance.

**Version**: 1.0.0 | **Ratified**: 2025-12-04 | **Last Amended**: 2025-12-04
