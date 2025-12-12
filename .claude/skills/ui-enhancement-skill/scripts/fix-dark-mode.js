const fs = require('fs');
const path = require('path');

/**
 * Fixes dark mode problems by adjusting text/background colors,
 * ensuring borders remain visible, improving card shadows, and optimizing typography
 */
function fixDarkMode(frontendDir = './frontend') {
  const customCssPath = path.join(frontendDir, 'src', 'css', 'custom.css');
  const cssDir = path.dirname(customCssPath);

  // Create the css directory if it doesn't exist
  if (!fs.existsSync(cssDir)) {
    fs.mkdirSync(cssDir, { recursive: true });
  }

  // Create comprehensive dark mode CSS fixes
  const darkModeCss = `/**
 * Custom CSS for dark mode fixes and improvements
 */

/* Base dark mode variables */
:root {
  --ifm-color-primary: #25c2a0;
  --ifm-color-primary-dark: #21af90;
  --ifm-color-primary-darker: #1fa588;
  --ifm-color-primary-darkest: #1a8870;
  --ifm-color-primary-light: #29d5b0;
  --ifm-color-primary-lighter: #32d8b4;
  --ifm-color-primary-lightest: #4fddbf;
  --ifm-code-font-size: 95%;

  /* Background colors */
  --ifm-background-color: #1a1a1a;
  --ifm-background-surface-color: #121212;

  /* Text colors */
  --ifm-font-color-base: #e0e0e0;
  --ifm-font-color-base-inverse: #282828;
  --ifm-heading-color: #ffffff;
  --ifm-color-content: #e0e0e0;
  --ifm-color-content-secondary: #b0b0b0;

  /* Border colors */
  --ifm-toc-border-color: #333333;
  --ifm-color-emphasis-0: rgba(255, 255, 255, 1);
  --ifm-color-emphasis-100: rgba(245, 245, 245, 0.1);
  --ifm-color-emphasis-200: rgba(235, 235, 235, 0.2);
  --ifm-color-emphasis-300: rgba(225, 225, 225, 0.3);
  --ifm-color-emphasis-400: rgba(215, 215, 215, 0.4);
  --ifm-color-emphasis-500: rgba(205, 205, 205, 0.5);
  --ifm-color-emphasis-600: rgba(195, 195, 195, 0.6);
  --ifm-color-emphasis-700: rgba(185, 185, 185, 0.7);
  --ifm-color-emphasis-800: rgba(175, 175, 175, 0.8);
  --ifm-color-emphasis-900: rgba(165, 165, 165, 0.9);
  --ifm-color-emphasis-1000: rgba(155, 155, 155, 1);

  /* Card and surface colors */
  --ifm-card-background-color: #1e1e1e;
  --ifm-card-border-color: #333333;

  /* Shadow colors */
  --ifm-global-shadow-lw: 0 1px 2px 0 rgba(255, 255, 255, 0.1);
  --ifm-global-shadow-md: 0 4px 6px -1px rgba(255, 255, 255, 0.1), 0 2px 4px -1px rgba(255, 255, 255, 0.06);
  --ifm-global-shadow-xl: 0 20px 25px -5px rgba(255, 255, 255, 0.1), 0 10px 10px -5px rgba(255, 255, 255, 0.04);

  /* Alert colors */
  --ifm-alert-background-color: #2d2d2d;
  --ifm-alert-border-color: #444444;

  /* Code block colors */
  --docusaurus-highlighted-code-line-bg: rgba(0, 0, 0, 0.3);
  --ifm-code-background: #2d2d2d;
  --ifm-code-color: #e0e0e0;
}

/* Dark mode specific overrides */
html[data-theme='dark'] {
  --ifm-background-color: #1a1a1a;
  --ifm-background-surface-color: #121212;
  --ifm-navbar-background-color: #1a1a1a;
  --ifm-footer-background-color: #1a1a1a;
  --ifm-hero-background-color: #121212;
  --ifm-card-background-color: #1e1e1e;
  --ifm-table-border-color: #333333;
  --ifm-table-header-background: #222222;
  --ifm-input-background-color: #2d2d2d;
  --ifm-input-border-color: #444444;
  --ifm-color-primary: #25c2a0;
}

/* Improved text contrast and readability */
html[data-theme='dark'] {
  /* Headings */
  h1, h2, h3, h4, h5, h6 {
    color: #ffffff !important;
  }

  /* Body text */
  p, div, span, li, td, th {
    color: #e0e0e0 !important;
  }

  /* Links */
  a {
    color: var(--ifm-color-primary-light) !important;
  }

  a:hover {
    color: var(--ifm-color-primary-lighter) !important;
  }

  /* Borders */
  .navbar,
  .table-of-contents,
  .pagination-nav,
  .alert,
  .card,
  .modal,
  hr {
    border-color: #333333 !important;
  }

  /* Tables */
  table {
    border-color: #333333 !important;
  }

  table tr:nth-child(2n) {
    background-color: #222222 !important;
  }

  /* Code blocks */
  .prism-code {
    background-color: #1e1e1e !important;
    border: 1px solid #333333 !important;
  }

  /* Blockquotes */
  blockquote {
    border-left: 4px solid var(--ifm-color-primary) !important;
    background-color: #222222 !important;
    color: #d0d0d0 !important;
  }

  /* Inputs */
  input, textarea, select {
    background-color: #2d2d2d !important;
    border: 1px solid #444444 !important;
    color: #e0e0e0 !important;
  }

  input:focus, textarea:focus, select:focus {
    border-color: var(--ifm-color-primary) !important;
    box-shadow: 0 0 0 2px rgba(37, 194, 160, 0.25) !important;
  }

  /* Buttons */
  .button {
    border-color: #444444 !important;
  }

  /* Search bar */
  .navbar__search-input {
    background-color: #2d2d2d !important;
    border: 1px solid #444444 !important;
    color: #e0e0e0 !important;
  }

  .navbar__search-input::placeholder {
    color: #a0a0a0 !important;
  }
}

/* Enhanced card styling */
html[data-theme='dark'] .card {
  background-color: var(--ifm-card-background-color) !important;
  border: 1px solid var(--ifm-card-border-color) !important;
  box-shadow: var(--ifm-global-shadow-md) !important;
}

/* Improved shadow effects */
html[data-theme='dark'] .shadow--lw {
  box-shadow: var(--ifm-global-shadow-lw) !important;
}

html[data-theme='dark'] .shadow--md {
  box-shadow: var(--ifm-global-shadow-md) !important;
}

html[data-theme='dark'] .shadow--xl {
  box-shadow: var(--ifm-global-shadow-xl) !important;
}

/* Better contrast for alert components */
html[data-theme='dark'] .alert {
  background-color: #2d2d2d !important;
  border-color: #444444 !important;
  color: #e0e0e0 !important;
}

html[data-theme='dark'] .alert--primary {
  --ifm-alert-background-color: #203530;
  --ifm-alert-border-color: #25c2a0;
}

html[data-theme='dark'] .alert--success {
  --ifm-alert-background-color: #253023;
  --ifm-alert-border-color: #4caf50;
}

html[data-theme='dark'] .alert--info {
  --ifm-alert-background-color: #203035;
  --ifm-alert-border-color: #2196f3;
}

html[data-theme='dark'] .alert--warning {
  --ifm-alert-background-color: #353020;
  --ifm-alert-border-color: #ff9800;
}

html[data-theme='dark'] .alert--danger {
  --ifm-alert-background-color: #352323;
  --ifm-alert-border-color: #f44336;
}

/* Improved sidebar navigation */
html[data-theme='dark'] .menu {
  background-color: #1a1a1a !important;
}

html[data-theme='dark'] .menu__list-item-collapsible:hover {
  background-color: #222222 !important;
}

html[data-theme='dark'] .menu__link--active {
  background-color: #25c2a0 !important;
  color: #ffffff !important;
}

/* Better code highlighting */
html[data-theme='dark'] .token.comment,
html[data-theme='dark'] .token.block-comment,
html[data-theme='dark'] .token.prolog,
html[data-theme='dark'] .token.doctype,
html[data-theme='dark'] .token.cdata {
  color: #a0a0a0 !important;
}

html[data-theme='dark'] .token.punctuation {
  color: #d0d0d0 !important;
}

html[data-theme='dark'] .token.property,
html[data-theme='dark'] .token.tag,
html[data-theme='dark'] .token.constant,
html[data-theme='dark'] .token.symbol,
html[data-theme='dark'] .token.deleted {
  color: #ff6b6b !important;
}

html[data-theme='dark'] .token.boolean,
html[data-theme='dark'] .token.number {
  color: #ffa500 !important;
}

html[data-theme='dark'] .token.selector,
html[data-theme='dark'] .token.attr-name,
html[data-theme='dark'] .token.string,
html[data-theme='dark'] .token.char,
html[data-theme='dark'] .token.builtin,
html[data-theme='dark'] .token.inserted {
  color: #51cf66 !important;
}

html[data-theme='dark'] .token.operator,
html[data-theme='dark'] .token.entity,
html[data-theme='dark'] .token.url,
html[data-theme='dark'] .language-css .token.string,
html[data-theme='dark'] .style .token.string {
  color: #74c0fc !important;
}

html[data-theme='dark'] .token.atrule,
html[data-theme='dark'] .token.attr-value,
html[data-theme='dark'] .token.keyword {
  color: #be4fdf !important;
}

/* Improved typography for readability */
html[data-theme='dark'] {
  font-feature-settings: 'kern' 1, 'liga' 1, 'clig' 1, 'calt' 1 !important;
}

/* Footer improvements */
html[data-theme='dark'] .footer {
  background-color: #121212 !important;
  border-top: 1px solid #333333 !important;
}

/* Breadcrumb improvements */
html[data-theme='dark'] .breadcrumb {
  background-color: #1e1e1e !important;
  border: 1px solid #333333 !important;
}

/* Pagination improvements */
html[data-theme='dark'] .pagination-nav__link {
  border: 1px solid #333333 !important;
  background-color: #1e1e1e !important;
}

html[data-theme='dark'] .pagination-nav__link:hover {
  border-color: var(--ifm-color-primary) !important;
  background-color: #222222 !important;
}

/* Search modal improvements */
html[data-theme='dark'] .DocSearch-Modal {
  background-color: #1a1a1a !important;
}

html[data-theme='dark'] .DocSearch-SearchBar {
  background-color: #222222 !important;
  border-bottom: 1px solid #333333 !important;
}

html[data-theme='dark'] .DocSearch-Hit {
  background-color: #1e1e1e !important;
  border: 1px solid #333333 !important;
}

/* Improved contrast for form elements */
html[data-theme='dark'] .form-control {
  background-color: #2d2d2d !important;
  border: 1px solid #444444 !important;
  color: #e0e0e0 !important;
}

/* Better contrast for buttons */
html[data-theme='dark'] .button--outline {
  color: var(--ifm-color-primary) !important;
  border-color: var(--ifm-color-primary) !important;
}

html[data-theme='dark'] .button--outline:hover {
  background-color: var(--ifm-color-primary) !important;
  color: white !important;
}

/* Enhanced mobile menu */
html[data-theme='dark'] .navbar-sidebar__brand {
  background-color: #1a1a1a !important;
  border-bottom: 1px solid #333333 !important;
}

html[data-theme='dark'] .navbar-sidebar__items {
  background-color: #1a1a1a !important;
}

/* Improved table of contents */
html[data-theme='dark'] .table-of-contents {
  background-color: #1e1e1e !important;
}

html[data-theme='dark'] .table-of-contents__left-border {
  border-left: 1px solid #333333 !important;
}

/* Better contrast for highlighted code lines */
html[data-theme='dark'] .docusaurus-highlight-code-line {
  background-color: rgba(0, 0, 0, 0.3) !important;
}
`;

  // Write the CSS file
  fs.writeFileSync(customCssPath, darkModeCss);

  console.log('Dark mode fixes applied successfully!');
  console.log(`Created/updated ${customCssPath}`);

  // Update docusaurus.config.js to include the custom CSS
  const configPath = path.join(frontendDir, 'docusaurus.config.js');
  if (fs.existsSync(configPath)) {
    let configContent = fs.readFileSync(configPath, 'utf8');

    // Add the custom CSS import to the stylesheets if not already present
    if (!configContent.includes('src/css/custom.css')) {
      if (configContent.includes('stylesheets: [')) {
        // Add to existing stylesheets array
        configContent = configContent.replace(
          /(stylesheets:\s*\[)/,
          `$1\n        {\n          href: '/src/css/custom.css',\n          type: 'text/css',\n        },`
        );
      } else if (configContent.includes('themeConfig: {')) {
        // Add stylesheets section after themeConfig opening
        configContent = configContent.replace(
          /themeConfig:\s*{/,
          `stylesheets: [\n        {\n          href: '/src/css/custom.css',\n          type: 'text/css',\n        },\n      ],\n      themeConfig: {`
        );
      }

      fs.writeFileSync(configPath, configContent);
      console.log('Updated docusaurus.config.js to include custom CSS');
    }
  }
}

// Export the function
module.exports = { fixDarkMode };

// If running directly
if (require.main === module) {
  const frontendDir = process.argv[2] || './frontend';
  fixDarkMode(frontendDir);
}