# Quickstart: Module 1 - The Robotic Nervous System (ROS 2)

## Prerequisites
- Node.js (v16 or higher)
- npm or yarn
- Git
- Basic Python knowledge
- Basic AI/robotics concepts

## Setup Docusaurus Environment

1. **Install Node.js dependencies**:
   ```bash
   npm install
   ```

2. **Start local development server**:
   ```bash
   npm start
   ```
   This command starts a local development server and opens the documentation in your browser. Most changes are reflected live without restarting the server.

3. **Create the module directory**:
   ```bash
   mkdir docs/module-1
   ```

## Create Module Content

1. **Module index page**:
   Create `docs/module-1/index.md` with overview content for Module 1

2. **Chapter pages**:
   Create the three required chapters:
   - `docs/module-1/ros2-foundations.md`
   - `docs/module-1/python-agents.md`
   - `docs/module-1/urdf-modeling.md`

## Build for Production

```bash
npm run build
```

This command generates static content in the `build/` directory, which can be served using any static hosting service.

## Deploy to GitHub Pages

1. **Configure deployment** in `docusaurus.config.js` with your project details
2. **Run the deployment command**:
   ```bash
   GIT_USER=<Your GitHub username> USE_SSH=true npm run deploy
   ```

## Local Testing

- Use `npm start` to run a local development server
- Verify all links and content render correctly
- Test navigation between chapters
- Ensure examples are clear and functional