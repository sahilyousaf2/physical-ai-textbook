# Physical AI & Humanoid Robotics Textbook

This repository contains a comprehensive textbook on Physical AI & Humanoid Robotics, built with Docusaurus and deployed on GitHub Pages.

## Project Structure

- `book/` - The Docusaurus website source code
- `specs/` - Project specifications and planning documents
- `history/` - Prompt history records
- `.github/workflows/` - GitHub Actions workflows for deployment

## Deployment Options

### GitHub Pages (Automatic)

The site is configured for automatic deployment to GitHub Pages:
1. The GitHub Actions workflow in `.github/workflows/deploy.yml` will automatically build and deploy on pushes to main
2. Site will be available at `https://<your-username>.github.io/physical-ai-textbook/`

### Vercel (Manual)

The site is also configured for Vercel deployment:
1. Connect your GitHub repository to Vercel
2. Set the Root Directory to `book/` during import
3. Site will be available at a Vercel URL

## Local Development

To run the site locally:

```bash
cd book
npm install
npm run start
```

## Configuration

Before deploying, update the following in `book/docusaurus.config.ts`:
- `url` - Your GitHub Pages URL (e.g., `https://your-username.github.io`)
- `organizationName` - Your GitHub username
- `projectName` - Your repository name

## About

This textbook covers:
- Module 1: The Robotic Nervous System (ROS 2)
- Module 2: The Digital Twin (Gazebo & Unity)
- Module 3: The AI-Robot Brain (NVIDIA Isaac)
- Module 4: Vision-Language-Action (VLA) Systems

---

## GitHub Pages Deployment Status
✅ GitHub Actions workflow is configured and will automatically deploy the site to GitHub Pages when changes are pushed to the main branch.
