---
title: GitHub Pages Deployment and Docusaurus Validation
---

# GitHub Pages Deployment and Docusaurus Interface Validation

This document provides comprehensive instructions for deploying the AI Robotics Textbook to GitHub Pages using Docusaurus and validating that the interface meets the project requirements.

## Deployment Prerequisites

Before deploying to GitHub Pages, ensure you have:

- GitHub repository set up with the textbook content
- Docusaurus project configured and tested locally
- GitHub Actions enabled for the repository
- Administrative access to repository settings

## Docusaurus Configuration for GitHub Pages

### Step 1: Configure docusaurus.config.js

Update your `docusaurus.config.js` file for GitHub Pages deployment:

```javascript
// @ts-check
// `@type` JSDoc annotations allow editor autocompletion and type checking
// (when paired with `@ts-check`).
// There are various equivalent ways to declare your Docusaurus config.
// See: https://docusaurus.io/docs/api/docusaurus-config

import { themes as prismThemes } from 'prism-react-renderer';

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'AI Robotics Textbook',
  tagline: 'Physical AI & Humanoid Robotics Course',
  favicon: 'img/favicon.ico',

  // Set the production URL of your site here
  url: 'https://hamzasheedi.github.io',  // Your GitHub org/user name
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: '/humanoid_robot_book/',  // Your repo name

  // GitHub pages deployment config.
  // If you aren't using GitHub pages, you don't need these.
  organizationName: 'hamzasheedi', // Usually your GitHub org/user name.
  projectName: 'humanoid_robot_book', // Usually your repo name.
  deploymentBranch: 'gh-pages', // Branch that GitHub Pages will deploy from.

  onBrokenLinks: 'throw',
  onBrokenMarkdownLinks: 'warn',

  // Even if you don't use internationalization, you can use this field to set
  // useful metadata like html lang. For example, if your site is Chinese, you
  // may want to replace "en" with "zh-Hans".
  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          sidebarPath: './sidebars.js',
          // Please change this to your repo.
          editUrl:
            'https://github.com/hamzasheedi/humanoid_robot_book/edit/main/',
        },
        blog: false, // Disable blog if not needed
        theme: {
          customCss: './src/css/custom.css',
        },
      }),
    ],
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      // Replace with your project's social card
      image: 'img/docusaurus-social-card.jpg',
      navbar: {
        title: 'AI Robotics Textbook',
        logo: {
          alt: 'Robotics Textbook Logo',
          src: 'img/robotics-logo.svg',
        },
        items: [
          {
            type: 'docSidebar',
            sidebarId: 'tutorialSidebar',
            position: 'left',
            label: 'Textbook',
          },
          {
            href: 'https://github.com/hamzasheedi/humanoid_robot_book',
            label: 'GitHub',
            position: 'right',
          },
        ],
      },
      footer: {
        style: 'dark',
        links: [
          {
            title: 'Docs',
            items: [
              {
                label: 'Textbook',
                to: '/docs/modules/ros2/introduction',
              },
            ],
          },
          {
            title: 'Community',
            items: [
              {
                label: 'Stack Overflow',
                href: 'https://stackoverflow.com/questions/tagged/docusaurus',
              },
              {
                label: 'Discord',
                href: 'https://discordapp.com/invite/docusaurus',
              },
            ],
          },
          {
            title: 'More',
            items: [
              {
                label: 'GitHub',
                href: 'https://github.com/hamzasheedi/humanoid_robot_book',
              },
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} AI Robotics Textbook. Built with Docusaurus.`,
      },
      prism: {
        theme: prismThemes.github,
        darkTheme: prismThemes.dracula,
      },
    }),
};

export default config;
```

### Step 2: Create GitHub Actions Workflow

Create `.github/workflows/deploy.yml`:

```yaml
name: Deploy to GitHub Pages

on:
  push:
    branches: [main]
  pull_request:
    branches: [main]

jobs:
  deploy:
    name: Deploy to GitHub Pages
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v3
      - uses: actions/setup-node@v3
        with:
          node-version: 18
          cache: yarn

      - name: Install dependencies
        run: yarn install --frozen-lockfile
      - name: Build website
        run: yarn build

      - name: Deploy to GitHub Pages
        uses: peaceiris/actions-gh-pages@v3
        with:
          github_token: ${{ secrets.GITHUB_TOKEN }}
          publish_dir: ./build
          # The following lines assign commit authorship to the official
          # GH-Actions bot for deploys to `gh-pages` branch:
          # https://github.com/actions/checkout/issues/13#issuecomment-724415212
          # The GH actions bot is used by default if you didn't specify the two fields.
          # You can swap them out with your own user credentials.
          user_name: github-actions[bot]
          user_email: 41898282+github-actions[bot]@users.noreply.github.com
```

## Deployment Process

### Step 1: Local Testing
Before deploying, test the site locally:

```bash
# Install dependencies
npm install

# Start local development server
npm run start

# Verify all pages load correctly
# Check navigation and links
# Verify content displays properly
```

### Step 2: Build for Production
Build the static site for production:

```bash
# Build the static files
npm run build
```

### Step 3: Deploy to GitHub Pages
Deploy by pushing to the main branch, which triggers the GitHub Actions workflow:

```bash
# Commit and push all changes to main branch
git add .
git commit -m "Deploy textbook site to GitHub Pages"
git push origin main
```

### Step 4: Configure GitHub Repository Settings
1. In your GitHub repository, navigate to Settings → Pages
2. Ensure the source is set to "Deploy from a branch"
3. Select branch `gh-pages` and root directory `/`
4. Click "Save"
5. Wait for the page to be built and deployed

## Interface Validation Checklist

### Visual Interface Validation

#### Layout and Structure
- [ ] Responsive design works on mobile, tablet, and desktop
- [ ] Navigation menu is accessible and functional
- [ ] Sidebar navigation displays correctly
- [ ] Footer appears on all pages
- [ ] Logo displays correctly in header
- [ ] Table of Contents renders properly for longer pages

#### Typography and Readability
- [ ] Font sizes are appropriate and readable
- [ ] Line spacing and paragraph spacing are comfortable for reading
- [ ] Text contrast meets accessibility requirements (WCAG AA minimum)
- [ ] Code blocks are clearly displayed with syntax highlighting
- [ ] Headings are hierarchically structured correctly
- [ ] Lists (ordered and unordered) display properly

#### Color Scheme and Theme
- [ ] Theme follows the configured color scheme (set in docusaurus.config.js)
- [ ] Dark/light mode toggles work correctly
- [ ] Colors maintain good contrast ratios
- [ ] Links are visually distinct and clearly clickable
- [ ] Buttons have appropriate hover and focus states

### Functional Interface Validation

#### Navigation
- [ ] Top navigation menu items work correctly
- [ ] Sidebar navigation links navigate to correct pages
- [ ] Previous/Next buttons at bottom of pages work correctly
- [ ] Breadcrumb navigation displays properly
- [ ] Search functionality works (if enabled)
- [ ] Table of Contents scrolls to sections in longer documents

#### Content Display
- [ ] All modules are accessible through navigation
- [ ] Images and diagrams display correctly
- [ ] Code snippets are properly formatted and syntax highlighted
- [ ] Mathematical formulas render correctly (if using Katex)
- [ ] Tables display properly
- [ ] Embedded media (if any) functions correctly

#### User Experience
- [ ] Pages load quickly (under 3 seconds)
- [ ] Site functions without JavaScript errors
- [ ] All external links open correctly
- [ ] Accessibility features are available (skip to main content, etc.)
- [ ] Print styles are appropriate for content

### Content Validation

#### Module-Specific Validation
- [ ] Module 1 (ROS 2) content is properly organized and linked
- [ ] Module 2 (Digital Twin) content is properly organized and linked
- [ ] Module 3 (NVIDIA Isaac) content is properly organized and linked
- [ ] Module 4 (VLA) content is properly organized and linked
- [ ] Module 5 (Capstone) content is properly organized and linked

#### Exercise and Assessment Content
- [ ] Exercise sections are properly formatted
- [ ] Solutions or hints are appropriately hidden if required
- [ ] Interactive elements function correctly (if any)

#### Reference Materials
- [ ] Glossary displays correctly
- [ ] Bibliography is properly formatted
- [ ] External links are functional
- [ ] Index (if present) links work properly

## Automated Validation

### Lighthouse Audit
Run a Lighthouse audit to validate performance, accessibility, SEO, and best practices:

```bash
# Install Lighthouse CLI
npm install -g @lhci/cli

# Run audit against deployed site
lhci collect --url=https://hamzasheedi.github.io/humanoid_robot_book/
lhci assert --preset=desktop --assertions='{"categories:performance":[">0.9"],"categories:accessibility":[">0.9"],"categories:best-practices":[">0.9"],"categories:seo":[">0.9]}'
```

### Link Validation
Use a link checker to ensure all internal and external links are functional:

```bash
# Install linkinator
npm install -g linkinator

# Check links on deployed site
linkinator https://hamzasheedi.github.io/humanoid_robot_book/
```

## Performance Validation

### Speed Testing
- [ ] Use tools like Google PageSpeed Insights to validate load times
- [ ] Ensure pages load under 3 seconds on 3G connections
- [ ] Optimize images if page speed is slow
- [ ] Minimize JavaScript/CSS bundles if needed

### Mobile Responsiveness
- [ ] Test on various screen sizes
- [ ] Verify navigation works on mobile
- [ ] Check that content is readable without horizontal scrolling

## Accessibility Validation

### WCAG Compliance
- [ ] Use automated tools like axe-core to test compliance
- [ ] Verify proper heading structure (h1, h2, h3, etc.)
- [ ] Check proper alt text on images
- [ ] Ensure form elements have proper labels
- [ ] Verify proper color contrast ratios

### Keyboard Navigation
- [ ] Test navigation using only keyboard
- [ ] Verify focus indicators are visible
- [ ] Check that skip navigation links work

## Cross-Browser Validation

Test the deployed site in multiple browsers:

- [ ] Chrome (latest version)
- [ ] Firefox (latest version)
- [ ] Safari (latest version)
- [ ] Microsoft Edge (latest version)
- [ ] Mobile browsers (Chrome for Android, Safari for iOS)

## Success Criteria Validation

### SC-006 Verification
Ensure all requirements from success criteria SC-006 are met:

- [ ] Textbook deployed on Docusaurus 
- [ ] Available on GitHub Pages
- [ ] Has functional navigation that works across modules
- [ ] Interface is user-friendly with intuitive design
- [ ] All links and navigation elements function correctly
- [ ] Content is properly organized by modules
- [ ] Visual appearance is professional and appealing
- [ ] No broken links or missing content

## Troubleshooting Common Issues

### Build Failures
- Check for syntax errors in config or MDX files
- Verify all required dependencies are installed
- Ensure all image files exist and are properly referenced

### Deployment Failures
- Verify GitHub Actions workflow permissions
- Check repository settings for GitHub Pages
- Ensure the gh-pages branch is properly configured

### Content Display Issues
- Check for improperly nested markdown elements
- Verify all images are in the correct directory
- Ensure all cross-references are correct

### Performance Issues
- Optimize images for web
- Minimize use of heavy JavaScript libraries
- Ensure efficient code snippet formatting

## Maintenance and Updates

### Process for Content Updates
1. Make changes to content locally
2. Test changes using `npm run start`
3. Build using `npm run build` to verify no errors
4. Commit and push changes to main branch
5. Monitor GitHub Actions for successful deployment
6. Verify changes on deployed site

### Monitoring Site Health
- Set up monitoring tools to alert if site is down
- Regular checks of link integrity
- Performance monitoring over time
- User feedback collection for usability issues

By following this deployment and validation process, the AI Robotics Textbook will be successfully deployed to GitHub Pages with a functional, user-friendly, and accessible interface that meets all specified requirements.