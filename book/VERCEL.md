# Physical AI & Humanoid Robotics Textbook

This is a [Docusaurus](https://docusaurus.io/) website for the Physical AI & Humanoid Robotics Textbook, deployed on [Vercel](https://vercel.com).

## Deploying to Vercel

This project is ready to deploy to Vercel. Here's how:

### Method 1: One-Click Deploy (Recommended)

[![Deploy with Vercel](https://vercel.com/button)](https://vercel.com/new/clone?repository-url=https://github.com/sahilyousaf2/physical-ai-textbook&project-name=physical-ai-textbook&repository-name=physical-ai-textbook)

### Method 2: Manual Deployment

1. Push your code to a GitHub repository
2. Go to [vercel.com](https://vercel.com) and create an account
3. Import your GitHub repository
4. Vercel will automatically detect this is a Docusaurus project and use the build settings from `vercel.json`
5. The build command will be `npm run build` and the output directory will be `build`

### Local Development

```bash
cd book
npm install
npm start
```

This command starts a local development server and opens up a browser window. Most changes are reflected live without having to restart the server.

### Build for Production

```bash
cd book
npm run build
```

This command generates static content into the `build` directory and can be served using any static contents hosting service.

## Configuration

The Vercel configuration is in `vercel.json` and is set up to:
- Use the `@vercel/static-build` builder
- Output files to the `build` directory
- Handle client-side routing with the catch-all route