# GitHub Actions Troubleshooting

## Issue
GitHub Pages is showing a 404 error because the `gh-pages` branch doesn't exist or doesn't contain the built site.

## Solution Steps

### 1. Check GitHub Actions Status
- Go to your repository on GitHub
- Click on the "Actions" tab
- Look for the "Deploy to GitHub Pages" workflow
- Check if it ran successfully after your latest commits
- If it failed, review the logs to see why

### 2. Enable GitHub Actions (if needed)
- If the Actions tab shows "This repository has no workflows yet" or they're disabled:
  - Go to Settings → Actions → General
  - Under "Workflow permissions", make sure "Allow GitHub Actions to create and approve pull requests" is checked
  - Under "Actions can read and write org tokens", ensure appropriate permissions are set

### 3. Manually Trigger the Workflow
- If the workflow exists but didn't run:
  - Go to the Actions tab
  - Find the "Deploy to GitHub Pages" workflow
  - Click "Run workflow" to manually trigger it

### 4. Verify the gh-pages Branch
- After the workflow runs successfully, check if a `gh-pages` branch was created
- Go to the dropdown that shows "main" branch, type "gh-pages" to see if it exists

### 5. Common Issues and Solutions
- **Workflow not running**: Make sure Actions are enabled for the repository
- **Permission errors**: Check that the workflow has proper permissions to push to gh-pages branch
- **Build failures**: Check the workflow logs for specific errors in the build process
- **Missing secrets**: The workflow should use `${{ secrets.GITHUB_TOKEN }}` which is automatically available

### 6. Expected Result
After the workflow runs successfully:
- A `gh-pages` branch will be created/updated with the built site
- The site will be available at https://sahilyousaf2.github.io/physical-ai-textbook/
- GitHub Pages should be configured to use the `gh-pages` branch as source