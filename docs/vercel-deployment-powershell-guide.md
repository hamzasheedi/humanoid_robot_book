---
title: Deploying to Vercel with PowerShell
---

# Deploying Your Web Project to Vercel Using PowerShell

This guide provides step-by-step instructions for deploying your Docusaurus-based AI Robotics Textbook to Vercel using PowerShell, assuming the Vercel CLI is already installed and authenticated.

## Prerequisites

- Vercel CLI installed and authenticated (`vercel login`)
- PowerShell terminal
- Valid `package.json` with build script
- Git repository initialized (optional but recommended)

## Step-by-Step Instructions

### Step 1: Verify Your Project Structure

First, check that your project has the necessary files:

```powershell
# Navigate to your project directory
Set-Location "C:\Users\Hamza\Desktop\qwen_ai_book\humanoid_robot_book"

# Verify package.json exists and check its contents
Get-Content package.json
```

Look for the build script in the `scripts` section. It should look something like:
```json
{
  "scripts": {
    "build": "docusaurus build",
    "start": "docusaurus start",
    "deploy": "docusaurus deploy"
  }
}
```

### Step 2: Test Your Build Script Locally

Before deploying, verify that your build script works correctly:

```powershell
# Test the build script locally to ensure it works
npm run build

# If successful, you should see a "build" directory created
# Check if the build directory exists
Test-Path build
```

**Important**: Make sure your build completes successfully before proceeding. Fix any build errors locally first to avoid deployment issues.

### Step 3: Run the Vercel Deployment Command

Start the deployment process with the Vercel CLI:

```powershell
# Initialize deployment
vercel
```

The Vercel CLI will prompt you with several configuration questions.

### Step 4: Configure Project Settings

You'll be prompted with several questions. Here's how to respond for a Docusaurus project:

**Question 1**: `? Set up and deploy “~/path-to-your-project”? [Y/n]`
```powershell
# Answer: Y
Y
```

**Question 2**: `? Which scope do you want to use?`
```powershell
# Select your account from the list
# For example: hamzasheedi (Hamza Sheedi)
# Use arrow keys to navigate and press Enter
```

**Question 3**: `? Link to existing project? [y/N]`
```powershell
# If this is a new project, answer: N
N
```

**Question 4**: `? What's your project's name?`
```powershell
# Enter a project name (or press Enter to accept default based on directory name)
# For example: humanoid-robot-book
humanoid-robot-book
```

**Question 5**: `? In which directory is your code located?`
```powershell
# Press Enter to use the current directory (.)
# Or specify if your code is in a subdirectory
.
```

### Step 5: Configure Build Settings

Vercel will now analyze your project and suggest default settings:

- **Detected Project Settings**:
  - **Framework**: Detected as "Docusaurus" or "Create React App"
  - **Root Directory**: Usually the project root
  - **Build Command**: Should detect `npm run build`
  - **Development Command**: Often empty for static sites

**Accept the suggested settings** by pressing Enter when prompted.

### Step 6: Configure Output Directory

```powershell
# Vercel will ask: "? Build Command (npm run build) [Enter to confirm]"
# Press Enter to confirm the detected build command

# Then it will ask: "? Output Directory (usually "public") [Enter to confirm]"
# For Docusaurus, this is usually "build" (not "public")
# Type "build" and press Enter
build
```

### Step 7: Deploy the Project

After configuration, Vercel will:

1. Upload your project files
2. Install dependencies using npm/yarn/pnpm
3. Run the build command (`npm run build`)
4. Deploy the built assets

Watch the PowerShell output for progress:
```powershell
# The deployment process will show:
- Installing dependencies
- Running build command
- Bundling output
- Uploading files
- Providing deployment URL
```

### Step 8: Review Deployment Results

The deployment should complete with a success message like:
```
✅  Success! Deployment complete.
🔗  URL: https://humanoid-robot-book-[unique-hash].vercel.app
⚡️  Prove: https://humanoid-robot-book-git-[branch]-[username].vercel.app
```

### Step 9: Deploy to Production (Optional)

If you want to deploy directly to production without preview:

```powershell
# Deploy directly to production
vercel --prod
```

This will skip the preview deployment and deploy directly to your project's main URL.

### Step 10: Verify Your Deployment

1. Open the provided URL in a web browser
2. Verify all pages load correctly
3. Test navigation and interactive elements
4. Ensure all assets (images, CSS, JS) are loading properly

### Step 11: Set Up Custom Domain (Optional)

If you have a custom domain you'd like to use:

```powershell
# Add a custom domain using PowerShell
vercel domains add yourdomain.com
```

Then follow the instructions to update your DNS settings.

### Step 12: Future Updates and Redeployment

To redeploy updates to your project:

```powershell
# For preview deployment (recommended for testing)
vercel

# For production deployment
vercel --prod

# To force a fresh production deployment
vercel --prod --force
```

## Important Notes

### Before Deploying
- **Always test locally**: Run `npm run build` to ensure your project builds without errors
- **Check your package.json**: Verify the build script matches your project framework
- **Review dependencies**: Ensure all dependencies are properly declared in package.json

### Troubleshooting Common Issues

**Build fails during deployment**:
```powershell
# Check build script locally first
npm run build
# If this fails, fix issues before deploying
```

**Wrong output directory**:
- For Docusaurus: `build` directory (not `public`)
- Update in Vercel settings if needed

**Environment variables**:
```powershell
# If your project needs environment variables, add them:
vercel env add NAME value --development
vercel env add NAME value --production
```

### Deployment Configuration Verification

The `vercel.json` file in this project is already configured for Docusaurus:

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

This configuration:
- Points to the project root
- Uses the static build package to build from package.json
- Specifies the `build` directory as the output (distDir)
- Sets up catch-all routing for client-side navigation
- Maintains clean URLs and trailing slash behavior

## Post-Deployment Validation

After deployment, verify the following:

1. **Accessibility**: Site loads at the provided Vercel URL
2. **Functionality**: Navigation works and all links resolve
3. **Assets**: Images, CSS, and JavaScript files load properly
4. **Content**: All pages display content as expected
5. **Performance**: Pages load within reasonable timeframes

Your Docusaurus-based AI Robotics Textbook is now deployed to Vercel and accessible via the provided URL. For future updates, run `vercel --prod` in PowerShell from your project directory to deploy updates to production.