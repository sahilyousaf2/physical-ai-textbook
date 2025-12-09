# Physical AI & Humanoid Robotics Textbook

This website is built using [Docusaurus](https://docusaurus.io/), a modern static website generator.

## Installation

```bash
npm install
```

## Local Development

```bash
npm run start
```

This command starts a local development server and opens up a browser window. Most changes are reflected live without having to restart the server.

## Build

```bash
npm run build
```

This command generates static content into the `build` directory and can be served using any static contents hosting service.

## Deployment

### GitHub Pages Deployment (Automatic)

This project is configured for automatic deployment to GitHub Pages using GitHub Actions. When you push to the `main` branch, the workflow in `.github/workflows/deploy.yml` will automatically build and deploy your site.

### Manual GitHub Pages Deployment

If you prefer to deploy manually:

```bash
GIT_USER=<Your GitHub username> npm run deploy
```

This command is a convenient way to build the website and push to the `gh-pages` branch.

Before deploying, make sure to update the `url`, `baseUrl`, `organizationName`, and `projectName` fields in `docusaurus.config.ts` to match your GitHub username and repository name.
