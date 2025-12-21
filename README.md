# AI-Driven Book: Physical AI & Humanoid Robotics Textbook

This repository contains the complete Physical AI & Humanoid Robotics textbook created by Hassan Khan (GIAIC). It is built using [Docusaurus](https://docusaurus.io/), a modern static website generator.

## Table of Contents
- Module 1: The Robotic Nervous System (ROS 2)
- Module 2: The Digital Twin (Gazebo & Unity)
- Module 3: The AI-Robot Brain (NVIDIA Isaac™)
- Module 4: Vision-Language-Action (VLA)

## Installation

```bash
npm install
```

## Local Development

```bash
npm start
```

This command starts a local development server and opens up a browser window. Most changes are reflected live without having to restart the server.

## Build

```bash
npm run build
```

This command generates static content into the `build` directory and can be served using any static contents hosting service.

## Deployment to GitHub Pages

```bash
npm run deploy
```

This command builds the website and deploys it to GitHub Pages using the `gh-pages` branch.

## Project Structure

- `docs/` - Contains all textbook content in Markdown format
- `src/` - Custom source files for the Docusaurus site
- `.specify/` - Spec-Kit Plus configuration and templates
- `specs/` - Feature specifications and plans
- `docusaurus.config.ts` - Main Docusaurus configuration
- `sidebars.ts` - Navigation sidebar configuration
