---
title: Vercel Deployment Guide
---

# Vercel Deployment Guide

This guide provides instructions for deploying the AI Robotics Textbook to Vercel, an alternative to GitHub Pages.

## Overview

Vercel is a cloud platform for static sites and serverless functions that provides excellent performance with its global edge network. While this project was initially configured for GitHub Pages, it can also be deployed to Vercel with minimal changes.

## Prerequisites

- Vercel account (sign up at https://vercel.com)
- Git repository connected to your Vercel account
- Node.js environment for building the site
- Admin access to your repository settings (if using Git integration)

## Configuration for Vercel

### 1. Using the Existing vercel.json File

This project already includes a `vercel.json` file configured for deployment:

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

This configuration tells Vercel:
- The project root is the current directory
- Use the static build package to build the site
- The output directory is `build` (where Docusaurus generates its files)
- All routes should go to `index.html` for client-side routing
- Clean URLs (no .html extensions) and trailing slashes as configured

### 2. Docusaurus Config for Vercel Deployment

For deployment to Vercel, if you want to host at the root domain, you may need to modify your `docusaurus.config.js`:

**For deployment to root of domain (e.g. https://your-site.vercel.app/)**:
```javascript
const config = {
  // Set the production url of your site here
  url: 'https://your-site.vercel.app', // Replace with your actual Vercel URL or custom domain
  // Set the /<baseUrl>/ pathname under which your site is served
  // For Vercel root deployment, use '/'
  baseUrl: '/',
  
  // Other configurations...
};
```

**For deployment to GitHub Pages (as currently configured)**:
```javascript
const config = {
  // Set the production url of your site here
  url: 'https://hamzasheedi.github.io',
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is '/<projectName>/'
  baseUrl: '/humanoid_robot_book/',
  
  // Other configurations...
};
```

If you want to maintain compatibility with both platforms, see the section on dual platform configuration below.

## Deployment Methods

### Method 1: Git Integration (Recommended)

1. **Link Your GitHub Repository**
   - Go to https://vercel.com/
   - Sign in with your GitHub account
   - Click "New Project" → "Import Git Repository"
   - Select your humanoid robot textbook repository

2. **Configure the Project**
   - **FRAMEWORK PRESET**: Select "Other"
   - **BUILD COMMAND**: `npm run build`
   - **OUTPUT DIRECTORY**: `build`
   - **ROOT DIRECTORY**: Leave as default (root)

3. **Deploy**
   - Click "Deploy" and Vercel will automatically build and deploy your site
   - On subsequent pushes to main branch, Vercel will redeploy automatically

### Method 2: Vercel CLI

1. **Install Vercel CLI**
   ```bash
   npm install -g vercel
   ```

2. **Build Your Site**
   ```bash
   npm run build
   ```

3. **Deploy**
   ```bash
   vercel --prod
   ```
   This will deploy to production directly. Use `vercel` without `--prod` for preview deployments.

## Dual Platform Configuration

To support deployment on both GitHub Pages and Vercel with a single codebase, use environment variables in your configuration:

```javascript
// docusaurus.config.js
const isVercel = !!process.env.VERCEL_ENV;
const isGHPages = !isVercel; // Default assumption when not on Vercel

const config = {
  // For Vercel deployments, use the Vercel URL; for GitHub Pages, use GitHub URL
  url: isVercel ? 'https://[project-name].[username].vercel.app' : 'https://[username].github.io',
  
  // For Vercel, deploy to root (/); for GitHub Pages, use project subdirectory (/[projectName]/)
  baseUrl: isVercel ? '/' : '/[projectName]/',
  
  // GitHub Pages specific config (only needed for GitHub deployment)
  organizationName: isGHPages ? '[username]' : undefined,
  projectName: isGHPages ? '[projectName]' : undefined,
  
  // Other configurations remain the same
  onBrokenLinks: 'throw',
  onBrokenMarkdownLinks: 'warn',
  
  // The rest of your configuration...
};
```

Replace `[project-name]`, `[username]`, and `[projectName]` with your actual values.

## Environment Variables

### During Build Time
Set these in your Vercel project settings:
- `NODE_VERSION`: 20.x (or your preferred Node version)
- `GENERATE_SOURCEMAP`: false (if you want to reduce bundle size)

### For Custom Domains
If you use a custom domain:
- Add your domain in the Vercel dashboard
- Update DNS settings as instructed by Vercel
- Vercel will automatically handle SSL certificates

## Verification

After deployment:

1. **Check Site Accessibility**: Visit your Vercel deployment URL
2. **Verify All Pages**: Navigate through all modules to ensure links work
3. **Test Functionality**: Verify search and interactive elements are working
4. **Validate Assets**: Ensure all images, CSS, and JS files load properly

## Performance Optimization

### Leverage Vercel's Global Edge Network
- Your site will automatically be served from edge locations worldwide
- This provides faster load times for users globally

### Image Optimization
Vercel automatically optimizes images, but ensure your Docusaurus config enables this:
```javascript
module.exports = {
  // ... other config
  themeConfig: {
    // ... other theme config
    image: 'img/docusaurus-social-card.jpg', // This helps with social cards
  },
};
```

### Caching
- Vercel provides excellent caching by default
- Ensure your `vercel.json` has appropriate caching headers if needed

## Troubleshooting Common Issues

### Issue: Site Not Loading Correctly
- **Cause**: Client-side routing not configured properly
- **Solution**: Ensure your `vercel.json` includes the catch-all route:
  ```json
  {
    "routes": [
      {
        "src": "/(.*)",
        "dest": "/index.html"
      }
    ]
  }
  ```

### Issue: Assets Not Loading
- **Cause**: Incorrect baseUrl in docusaurus.config.js
- **Solution**: For Vercel root deployment, use baseUrl: '/'

### Issue: Build Errors
- **Cause**: Incorrect Node.js version or missing dependencies
- **Solution**: Check that your Node.js version supports Docusaurus 3.x (Node 18+ required)

## Custom Domain Setup

1. **In Vercel Dashboard**:
   - Go to your project settings
   - Click "Domains"
   - Add your custom domain

2. **Update DNS Settings**:
   - Follow Vercel's instructions to update your DNS provider
   - Usually involves adding a CNAME record pointing to Vercel

3. **Wait for Propagation**:
   - DNS changes can take up to 48 hours to propagate
   - Vercel dashboard will show when the domain is ready

## Monitoring and Analytics

Vercel provides built-in analytics for your deployed site:
- Traffic statistics
- Geographic distribution of visitors
- Performance metrics
- Error monitoring

Access these in your Vercel dashboard under your project.

## Next Steps

After successfully deploying to Vercel:
1. Set up custom domain if desired
2. Configure analytics and monitoring
3. Set up preview deployments for pull requests
4. Implement any additional Vercel features (edge functions, serverless APIs, etc.)

Both GitHub Pages and Vercel are viable deployment options for your AI Robotics Textbook, each with their own advantages. Choose the one that best fits your hosting needs and audience requirements.