---
title: Deployment Guide for GitHub Pages
---

# Deployment Guide for GitHub Pages

This guide provides detailed steps for deploying the AI Robotics Textbook to GitHub Pages using Docusaurus v3.

## Overview

This project uses Docusaurus v3 for documentation generation and GitHub Pages for deployment. The site has already been configured with the correct settings in `docusaurus.config.js` for GitHub Pages deployment.

## GitHub Pages Deployment Guide

### Prerequisites
- GitHub repository set up with your Docusaurus project
- Admin access to repository settings
- Properly configured `docusaurus.config.js` with correct `url`, `baseUrl`, `organizationName`, and `projectName` values
- Git user credentials set up (either via HTTPS with token or SSH key)

### Configuration in docusaurus.config.js

The deployment settings have already been configured in this project:

```javascript
const config = {
  // Set the production url of your site here
  url: 'https://hamzasheedi.github.io',
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: '/humanoid_robot_book/',
  
  // GitHub pages deployment config
  organizationName: 'hamzasheedi', // Usually your GitHub org/user name.
  projectName: 'humanoid_robot_book', // Usually your repo name.
  
  // Specify trailing slash behavior for GitHub Pages deployment
  trailingSlash: true,
  // ...
};
```

### Deployment Steps

1. **Build the static site**:
   ```bash
   npm run build
   ```
   This creates a `build` directory with statically generated HTML files optimized for deployment.

2. **Deploy to GitHub Pages**:
   ```bash
   GIT_USER=hamzasheedi npm run deploy
   ```
   This command does the following:
   - Builds the static site (if not already built)
   - Creates or updates the `gh-pages` branch
   - Pushes the built site to the `gh-pages` branch on GitHub

### GitHub Repository Settings

1. Navigate to your GitHub repository
2. Go to **Settings** tab
3. Scroll down to the **Pages** section
4. Under **Source**, ensure it's set to:
   - **Branch:** `gh-pages`
   - **Folder:** `/ (root)` or `/docs` depending on your configuration
5. Click **Save** if changes are needed

### Verification Steps

1. After deployment, check the GitHub repository:
   - A `gh-pages` branch should exist
   - The branch should contain the built static files

2. Visit the site at: `https://hamzasheedi.github.io/humanoid_robot_book/`
   
3. Verify that:
   - All pages load correctly
   - Navigation works properly
   - Links to modules and sections resolve correctly
   - Images and assets display properly
   - Search functionality (if enabled) works

### Troubleshooting

#### Common Deployment Issues

**Issue**: Deployment fails with authentication error
**Solution**: Ensure you have proper credentials set up for git operations, either:
- Personal Access Token for HTTPS authentication
- SSH key properly configured

**Issue**: Site doesn't load after deployment
**Solution**: 
- Check the GitHub Pages settings in repository settings
- Verify the correct branch and folder are selected
- Ensure the baseUrl in `docusaurus.config.js` matches your repository name

**Issue**: Links are broken in deployed site
**Solution**:
- Verify that all internal links are relative (not absolute paths)
- Ensure the site is browsed via the correct URL (https://username.github.io/repo-name/)

#### Verification Commands

- Check if the build was successful: `npm run build`
- Manually check the generated site: `npm run serve` and visit `http://localhost:3000`
- Verify git remote: `git remote -v`
- Check current branch: `git branch`

### Manual Deployment (Alternative Method)

If the automatic deployment fails, you can manually deploy:

1. Build the site: `npm run build`
2. Create a new orphan branch: `git checkout --orphan gh-pages`
3. Remove all files: `git rm -rf .`
4. Copy the build folder content (from the `build` directory)
5. Commit and push: `git add . && git commit -m "Deploy website" && git push origin gh-pages`
6. Switch back to your main branch: `git checkout -`

### Deployment Schedule

For regular updates, follow this workflow:
1. Make changes to documentation
2. Test locally: `npm start`
3. Build: `npm run build`
4. Deploy: `GIT_USER=hamzasheedi npm run deploy`
5. Verify deployment at the site URL

## Post-Deployment Validation

After deployment:

1. **Accessibility**: Verify the site is accessible at the configured URL
2. **Functionality**: Test navigation, search, and interactive elements
3. **Performance**: Check page load times and responsiveness
4. **Content Accuracy**: Verify all content appears as intended

## Updating Content Post-Deployment

To update content after initial deployment:

1. Make changes to source files
2. Commit changes to the main branch
3. Run deployment command again: `GIT_USER=hamzasheedi npm run deploy`
4. The site will update automatically

## Security Considerations

- Ensure that sensitive information is not included in the documentation
- Review all content before deployment to GitHub Pages (which is public by default)
- Use appropriate access controls for the GitHub repository

---

The AI Robotics Textbook is now ready for deployment to GitHub Pages. The configuration is set up correctly to ensure the site appears at: https://hamzasheedi.github.io/humanoid_robot_book/