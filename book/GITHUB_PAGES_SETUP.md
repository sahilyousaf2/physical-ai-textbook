# GitHub Pages Setup Instructions

To properly deploy your Docusaurus site to GitHub Pages, follow these steps:

## 1. Configure GitHub Actions Workflow
The workflow is already configured in `.github/workflows/deploy.yml` and should automatically build and deploy your site when you push to the `main` branch.

## 2. Configure GitHub Pages Settings
1. Go to your repository on GitHub
2. Click on the "Settings" tab
3. Scroll down to the "Pages" section in the left sidebar
4. Under "Source", select "Deploy from a branch"
5. Select "gh-pages" as the branch
6. Select "/" as the folder
7. Click "Save"

## 3. Verify GitHub Actions
1. Go to the "Actions" tab in your repository
2. Make sure the "Deploy to GitHub Pages" workflow is enabled
3. Check that the workflow has run successfully after your last commit

## 4. Troubleshooting
If you're still seeing the "Dependencies lock file is not found" error:

- Make sure GitHub Pages is set to use the `gh-pages` branch, not the `main` branch
- The GitHub Actions workflow should automatically build the site and push it to the `gh-pages` branch
- Your site will be available at: https://sahilyousaf2.github.io/physical-ai-textbook/

## 5. Important Notes
- Do NOT enable GitHub Pages to build from the main branch, as it will try to build the root directory which doesn't contain the necessary package files
- The Docusaurus site is in the `book` directory and is built using the GitHub Actions workflow
- The workflow automatically pushes the built site to the `gh-pages` branch for hosting

## 6. Verification
After setting up GitHub Pages:
- The workflow should run automatically when you push to `main`
- The `gh-pages` branch will be updated with the built site
- Your site will be accessible via the GitHub Pages URL