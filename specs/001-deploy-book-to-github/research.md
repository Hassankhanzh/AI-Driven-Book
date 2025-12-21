# Research: Deployment & GitHub Pages Publishing

## Decision: Docusaurus GitHub Pages Configuration
**Rationale**: To deploy the Docusaurus site to GitHub Pages, we need to configure the docusaurus.config.ts file with the appropriate settings for GitHub Pages deployment.

**Configuration parameters identified from user requirements:**
- url: https://hassankhanzh.github.io
- baseUrl: /AI-Book/
- organizationName: Hassankhanzh
- projectName: AI-Book

## Decision: Repository Setup
**Rationale**: The repository needs to be set up at https://github.com/Hassankhanzh/AI-Book as specified in the requirements.

**Key requirements identified:**
- Repository: AI-Book (public)
- Branch: main
- All content in Markdown (.md)
- No secrets, temp files, or drafts committed

## Decision: Build and Deployment Process
**Rationale**: Following standard Docusaurus GitHub Pages deployment practices.

**Process identified:**
- npm run build passes locally (verification step)
- npm run deploy (or equivalent) completes successfully
- GitHub Pages source set correctly (Docusaurus deploy branch)
- Site loads correctly at GitHub Pages URL

## Decision: Content Structure
**Rationale**: The site contains the Physical AI & Humanoid Robotics textbook content across 4 modules.

**Content requirements identified:**
- Modules 1-4 chapters in Markdown format
- Spec-Kit Plus artifacts (/sp.constitution, /sp.specify, /sp.plan, Docusaurus site)
- Clean commit history with clear messages

## Alternatives Considered

1. **Alternative deployment methods**:
   - GitHub Actions workflow vs. Docusaurus deploy command
   - Chosen: Standard Docusaurus deploy command (npm run deploy) as it's simpler and follows Docusaurus conventions

2. **Different base URLs**:
   - Using baseUrl: / vs. baseUrl: /AI-Book/
   - Chosen: baseUrl: /AI-Book/ as specified in requirements for GitHub Pages

3. **Content formats**:
   - Markdown vs. other formats
   - Chosen: Markdown (.md) as specified in requirements