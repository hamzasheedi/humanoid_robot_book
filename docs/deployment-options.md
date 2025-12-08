---
title: Deployment Options: GitHub Pages & Vercel
---

# Deployment Options: GitHub Pages & Vercel

This guide provides detailed steps for deploying the AI Robotics Textbook to GitHub Pages or Vercel using Docusaurus v3.

## Overview

This project uses Docusaurus v3 for documentation generation. You can deploy to either GitHub Pages or Vercel depending on your needs and preferences. Both deployment methods are fully supported, though this project was primarily configured for GitHub Pages.

---

## PART 1: Deploying to GitHub Pages

### Prerequisites
- GitHub repository set up with your Docusaurus project
- Admin access to repository settings
- Properly configured `docusaurus.config.js` with correct `url`, `baseUrl`, `organizationName`, and `projectName` values
- Git user credentials set up (either via HTTPS with token or SSH key)

### Configuration in docusaurus.config.js

The GitHub Pages deployment settings are configured in this project:

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

---

## PART 2: Deploying to Vercel

### Prerequisites
- Vercel account (linked with GitHub)
- Access to the repository in both GitHub and Vercel
- A `vercel.json` configuration file in the project root

### Configuration Overview

The project includes a `vercel.json` file that enables Vercel deployment:

```json
{
  "root": ".",
  "builds": [
    {
      "src": "package.json",
      "use": "@vercel/static-build",
      "config": {
        "distDir": "build"
      }
    }
  ],
  "routes": [
    {
      "src": "/(.*)",
      "dest": "/index.html"
    }
  ],
  "cleanUrls": true,
  "trailingSlash": true
}
```

### Deployment Steps for Vercel

#### Option A: Git Integration (Recommended)
1. Go to https://vercel.com/
2. Sign in with your GitHub account
3. Click "New Project" → "Import Git Repository"
4. Select your `humanoid_robot_book` repository
5. Configure the project settings:
   - **FRAMEWORK PRESET**: Select "Other" (since Docusaurus has its own build process)
   - **BUILD COMMAND**: `npm run build`
   - **OUTPUT DIRECTORY**: `build` (this is the directory Docusaurus outputs to)
   - **ROOT DIRECTORY**: Leave as default (root)
6. Add environment variables if needed:
   - NODE_VERSION: 20.x or your preferred version
7. Deploy the project

#### Option B: Manual Deployment (Using Vercel CLI)
1. Install Vercel CLI:
   ```bash
   npm install -g vercel
   ```
2. Build the static site:
   ```bash
   npm run build
   ```
3. Deploy to Vercel:
   ```bash
   vercel --prod
   ```

### Vercel Configuration Details

The `vercel.json` file specifies how Vercel should build and deploy your project:
- **root**: Points to the project root directory
- **builds**: Defines how to build the project using the package.json script
- **routes**: Sets up catch-all routing for SPA functionality (needed for client-side routing)
- **cleanUrls**: Removes .html extensions from URLs for cleaner paths
- **trailingSlash**: Controls trailing slash behavior to match Docusaurus configuration

### Environment Variables for Vercel

For Vercel-specific configuration without changing the main codebase, you can define environment variables during project import in the Vercel dashboard:

- `NODE_VERSION`: Node.js version to use (e.g., "20.x")
- Any other build or runtime environment variables

### Custom Domain Setup (Optional)

1. In your Vercel dashboard, navigate to your project
2. Go to Settings → Domains
3. Add your custom domain
4. Follow Vercel's instructions to update DNS settings
5. Vercel will automatically provision an SSL certificate

### Verification Steps for Vercel

1. After deployment, check that:
   - The site is accessible at the provided Vercel URL (e.g., `https://humanoid-robot-book-hamzasheedi.vercel.app`)
   - If using custom domain, verify it works at your domain
   - All pages load correctly
   - Navigation works properly
   - Assets and interactive elements function properly

### Troubleshooting Vercel Deployment

#### Common Issues:
- **Build failures**: Check NODE_VERSION is appropriate for Docusaurus 3.x (needs Node 18+)
- **Asset loading problems**: If deploying to subdirectory instead of root, adjust baseUrl accordingly
- **Routing issues**: The `vercel.json` routes section handles SPA routing by redirecting all routes to index.html

#### For GitHub Pages vs Vercel Configuration:
If you need to deploy the same codebase to both platforms with different base URLs, you can use environment variables:

```js
// In docusaurus.config.js
const isVercel = process.env.VERCEL_ENV !== undefined;

const config = {
  url: isVercel 
    ? 'https://humanoid-robot-book.hamzasheedi.vercel.app' 
    : 'https://hamzasheedi.github.io',
  baseUrl: isVercel ? '/' : '/humanoid_robot_book/',
  // ... rest of config
};
```

---

## Comparing GitHub Pages vs Vercel

| Feature | GitHub Pages | Vercel |
|---------|--------------|--------|
| **Cost** | Free for public repos | Free tier with generous limits |
| **Setup** | Requires gh-pages branch | Git integration or CLI |
| **Build Time** | Limited | More generous (30 mins) |
| **Global CDN** | Basic | Excellent with edge locations |
| **SSL Certificates** | Automatic for GitHub domains | Automatic for all deployments |
| **Custom Domains** | Supported | Supported with easy setup |
| **Preview Deployments** | No native support | Native support for PR previews |
| **Analytics** | Third-party required | Built-in analytics |
| **Caching** | Standard | Advanced edge caching |
| **Performance** | Good | Excellent with global edge network |

### When to Choose GitHub Pages:
- If you're already using GitHub for source control
- For completely free hosting of open-source documentation
- For simple deployment that stays within the GitHub ecosystem
- If you don't need advanced features like preview deployments

### When to Choose Vercel:
- For better global performance with worldwide CDN
- When you need preview deployments for PRs
- For advanced caching and optimization features
- If you want built-in analytics and performance monitoring

## Notes on Dual Platform Configuration

The current configuration supports GitHub Pages deployment with the URL `https://hamzasheedi.github.io/humanoid_robot_book/`. If you want to deploy to both platforms:

1. **For Vercel as primary**: You would need to modify the baseUrl to `/` in docusaurus.config.js and ensure all paths are relative to root
2. **For both platforms**: Use environment variables to dynamically set the baseUrl based on the deployment platform
3. **For platform-specific builds**: Create separate build scripts or branches for each platform with appropriate configurations

## Troubleshooting

### Common Deployment Issues for Both Platforms:

1. **Broken Links**: 
   - Verify all internal links are correct relative to your URL structure
   - For GitHub Pages, ensure links account for the subdirectory (e.g., `/humanoid_robot_book/docs/`)
   - For Vercel, ensure links work relative to root or configured base path

2. **Missing Assets**:
   - Check that the CSS, JS, and image files are loading correctly
   - Verify that your baseUrl setting is correct for the deployment platform

3. **Navigation Problems**:
   - Ensure client-side routing is properly configured
   - Test navigation between all sections of your site

4. **Build Failures**:
   - Check that all dependencies are properly defined in package.json
   - Verify the Node.js version is compatible with your Docusaurus version
   - Look for any platform-specific configuration differences needed

The AI Robotics Textbook is now configured for deployment on both GitHub Pages and Vercel. Choose the platform that best fits your deployment requirements and audience accessibility needs.