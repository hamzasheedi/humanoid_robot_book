const fs = require('fs');
const path = require('path');

/**
 * Creates a new responsive navbar with sign in, sign up, profile menu,
 * GitHub link, dark/light toggle, and animations
 */
function createNavbar(frontendDir = './frontend') {
  const navbarPath = path.join(frontendDir, 'src', 'components', 'Navbar', 'Navbar.jsx');
  const navbarCssPath = path.join(frontendDir, 'src', 'components', 'Navbar', 'Navbar.css');
  const navbarDir = path.dirname(navbarPath);

  // Create the navbar directory if it doesn't exist
  if (!fs.existsSync(navbarDir)) {
    fs.mkdirSync(navbarDir, { recursive: true });
  }

  // Create Navbar component
  const navbarComponent = `import React, { useState, useEffect } from 'react';
import './Navbar.css';

const Navbar = () => {
  const [isMenuOpen, setIsMenuOpen] = useState(false);
  const [isProfileOpen, setIsProfileOpen] = useState(false);
  const [darkMode, setDarkMode] = useState(false);

  // Initialize dark mode based on system preference
  useEffect(() => {
    const prefersDark = window.matchMedia('(prefers-color-scheme: dark)').matches;
    const savedTheme = localStorage.getItem('theme');
    setDarkMode(savedTheme ? savedTheme === 'dark' : prefersDark);
  }, []);

  // Apply theme to document
  useEffect(() => {
    if (darkMode) {
      document.documentElement.setAttribute('data-theme', 'dark');
      localStorage.setItem('theme', 'dark');
    } else {
      document.documentElement.setAttribute('data-theme', 'light');
      localStorage.setItem('theme', 'light');
    }
  }, [darkMode]);

  const toggleDarkMode = () => {
    setDarkMode(!darkMode);
  };

  const toggleMenu = () => {
    setIsMenuOpen(!isMenuOpen);
  };

  const toggleProfile = () => {
    setIsProfileOpen(!isProfileOpen);
  };

  return (
    <nav className="navbar">
      <div className="navbar-container">
        <div className="navbar-brand">
          <img
            src={darkMode ? "/img/logo-dark.svg" : "/img/logo-light.svg"}
            alt="Logo"
            className="navbar-logo"
          />
          <span className="navbar-title">Physical AI & Humanoid Robotics</span>
        </div>

        {/* Desktop Navigation */}
        <div className="navbar-menu desktop">
          <a href="/" className="nav-link">Home</a>
          <a href="/docs" className="nav-link">Textbook</a>
          <a href="/tutorials" className="nav-link">Tutorials</a>
          <a href="/resources" className="nav-link">Resources</a>
        </div>

        {/* Right Side Items */}
        <div className="navbar-right">
          <a
            href="https://github.com/your-repo/humanoid-robot-book"
            className="nav-icon github-link"
            target="_blank"
            rel="noopener noreferrer"
            aria-label="GitHub Repository"
          >
            <svg width="24" height="24" viewBox="0 0 24 24" fill="currentColor">
              <path d="M12 0c-6.626 0-12 5.373-12 12 0 5.302 3.438 9.8 8.207 11.387.599.111.793-.261.793-.577v-2.234c-3.338.726-4.033-1.416-4.033-1.416-.546-1.387-1.333-1.756-1.333-1.756-1.089-.745.083-.729.083-.729 1.205.084 1.839 1.237 1.839 1.237 1.07 1.834 2.807 1.304 3.492.997.107-.775.418-1.305.762-1.604-2.665-.305-5.467-1.334-5.467-5.931 0-1.311.469-2.381 1.236-3.221-.124-.303-.535-1.524.117-3.176 0 0 1.008-.322 3.301 1.23.957-.266 1.983-.399 3.003-.404 1.02.005 2.047.138 3.006.404 2.291-1.552 3.297-1.23 3.297-1.23.653 1.653.242 2.874.118 3.176.77.84 1.235 1.911 1.235 3.221 0 4.609-2.807 5.624-5.479 5.921.43.372.823 1.102.823 2.222v3.293c0 .319.192.694.801.576 4.765-1.589 8.199-6.086 8.199-11.386 0-6.627-5.373-12-12-12z"/>
            </svg>
          </a>

          <button
            className="theme-toggle"
            onClick={toggleDarkMode}
            aria-label={darkMode ? "Switch to light mode" : "Switch to dark mode"}
          >
            {darkMode ? (
              <svg width="24" height="24" viewBox="0 0 24 24" fill="currentColor">
                <path d="M12 2.25a.75.75 0 01.75.75v2.25a.75.75 0 01-1.5 0V3a.75.75 0 01.75-.75zM7.5 12a4.5 4.5 0 119 0 4.5 4.5 0 01-9 0zM18.894 6.166a.75.75 0 00-1.06-1.06l-1.591 1.59a.75.75 0 101.06 1.061l1.591-1.59zM21.75 12a.75.75 0 01-.75.75h-2.25a.75.75 0 010-1.5H21a.75.75 0 01.75.75zM17.834 18.894a.75.75 0 001.06-1.06l-1.59-1.591a.75.75 0 10-1.061 1.06l1.59 1.591zM12 18a.75.75 0 01.75.75V21a.75.75 0 01-1.5 0v-2.25A.75.75 0 0112 18zM7.758 17.303a.75.75 0 00-1.061-1.06l-1.591 1.59a.75.75 0 001.06 1.061l1.591-1.59zM6 12a.75.75 0 01-.75.75H3a.75.75 0 010-1.5h2.25A.75.75 0 016 12zM6.697 7.757a.75.75 0 001.06-1.06l-1.59-1.591a.75.75 0 00-1.061 1.06l1.59 1.591z"/>
              </svg>
            ) : (
              <svg width="24" height="24" viewBox="0 0 24 24" fill="currentColor">
                <path fillRule="evenodd" d="M9.528 1.718a.75.75 0 01.162.819A8.97 8.97 0 009 6a9 9 0 009 9 8.97 8.97 0 003.463-.69.75.75 0 01.981.98 10.503 10.503 0 01-9.694 6.46c-5.799 0-10.5-4.701-10.5-10.5 0-4.368 2.667-8.112 6.46-9.694a.75.75 0 01.818.162z" clipRule="evenodd"/>
              </svg>
            )}
          </button>

          <div className="profile-dropdown">
            <button
              className="profile-button"
              onClick={toggleProfile}
              aria-label="User profile menu"
            >
              <svg width="24" height="24" viewBox="0 0 24 24" fill="currentColor">
                <path d="M12 12c2.21 0 4-1.79 4-4s-1.79-4-4-4-4 1.79-4 4 1.79 4 4 4zm0 2c-2.67 0-8 1.34-8 4v2h16v-2c0-2.66-5.33-4-8-4z"/>
              </svg>
            </button>

            {isProfileOpen && (
              <div className="dropdown-menu">
                <a href="/profile" className="dropdown-item">Profile</a>
                <a href="/settings" className="dropdown-item">Settings</a>
                <hr className="dropdown-divider" />
                <a href="/logout" className="dropdown-item">Sign Out</a>
              </div>
            )}
          </div>

          <div className="auth-buttons">
            <button className="btn btn-outline">Sign In</button>
            <button className="btn btn-primary">Sign Up</button>
          </div>
        </div>

        {/* Mobile menu button */}
        <button
          className="menu-toggle"
          onClick={toggleMenu}
          aria-label="Toggle navigation menu"
        >
          <span></span>
          <span></span>
          <span></span>
        </button>
      </div>

      {/* Mobile Menu */}
      {isMenuOpen && (
        <div className="mobile-menu">
          <a href="/" className="mobile-nav-link">Home</a>
          <a href="/docs" className="mobile-nav-link">Textbook</a>
          <a href="/tutorials" className="mobile-nav-link">Tutorials</a>
          <a href="/resources" className="mobile-nav-link">Resources</a>
          <div className="mobile-auth-buttons">
            <button className="btn btn-outline">Sign In</button>
            <button className="btn btn-primary">Sign Up</button>
          </div>
        </div>
      )}
    </nav>
  );
};

export default Navbar;
`;

  // Create Navbar CSS
  const navbarCss = `/* Custom Navbar Styles */
.navbar {
  position: sticky;
  top: 0;
  z-index: 1000;
  background-color: var(--ifm-navbar-background-color, #ffffff);
  border-bottom: 1px solid var(--ifm-toc-border-color);
  transition: all 0.3s ease;
}

.navbar[data-theme="dark"] {
  background-color: var(--ifm-navbar-background-color, #1a1a1a);
}

.navbar-container {
  display: flex;
  align-items: center;
  justify-content: space-between;
  padding: 0.5rem 1rem;
  max-width: 1400px;
  margin: 0 auto;
}

.navbar-brand {
  display: flex;
  align-items: center;
  gap: 0.75rem;
  text-decoration: none;
}

.navbar-logo {
  height: 2.5rem;
  width: auto;
}

.navbar-title {
  font-size: 1.25rem;
  font-weight: 600;
  color: var(--ifm-navbar-title-color);
}

.navbar-menu.desktop {
  display: flex;
  gap: 2rem;
}

.nav-link {
  text-decoration: none;
  color: var(--ifm-navbar-link-color);
  font-weight: 500;
  padding: 0.5rem 0.75rem;
  border-radius: 4px;
  transition: all 0.2s ease;
  position: relative;
}

.nav-link:hover {
  color: var(--ifm-navbar-link-hover-color);
  background-color: var(--ifm-color-emphasis-100);
}

.nav-link::after {
  content: '';
  position: absolute;
  bottom: -2px;
  left: 0;
  width: 0;
  height: 2px;
  background-color: var(--ifm-color-primary);
  transition: width 0.3s ease;
}

.nav-link:hover::after {
  width: 100%;
}

.navbar-right {
  display: flex;
  align-items: center;
  gap: 1rem;
  position: relative;
}

.nav-icon {
  display: flex;
  align-items: center;
  justify-content: center;
  width: 36px;
  height: 36px;
  border-radius: 50%;
  background-color: var(--ifm-color-emphasis-100);
  color: var(--ifm-color-emphasis-700);
  text-decoration: none;
  transition: all 0.2s ease;
}

.nav-icon:hover {
  background-color: var(--ifm-color-emphasis-200);
  transform: translateY(-2px);
  color: var(--ifm-color-primary);
}

.theme-toggle {
  background: none;
  border: none;
  cursor: pointer;
  padding: 0.5rem;
  border-radius: 50%;
  display: flex;
  align-items: center;
  justify-content: center;
  transition: all 0.2s ease;
  color: var(--ifm-color-emphasis-700);
}

.theme-toggle:hover {
  background-color: var(--ifm-color-emphasis-100);
  transform: translateY(-2px);
}

.profile-dropdown {
  position: relative;
}

.profile-button {
  background: none;
  border: none;
  cursor: pointer;
  padding: 0.5rem;
  border-radius: 50%;
  display: flex;
  align-items: center;
  justify-content: center;
  transition: all 0.2s ease;
  color: var(--ifm-color-emphasis-700);
}

.profile-button:hover {
  background-color: var(--ifm-color-emphasis-100);
  transform: translateY(-2px);
}

.dropdown-menu {
  position: absolute;
  top: 100%;
  right: 0;
  background-color: var(--ifm-background-color);
  border: 1px solid var(--ifm-toc-border-color);
  border-radius: 8px;
  box-shadow: 0 4px 12px rgba(0, 0, 0, 0.15);
  min-width: 160px;
  z-index: 1001;
  margin-top: 0.5rem;
}

.dropdown-item {
  display: block;
  padding: 0.75rem 1rem;
  text-decoration: none;
  color: var(--ifm-color-emphasis-800);
  transition: background-color 0.2s ease;
}

.dropdown-item:hover {
  background-color: var(--ifm-color-emphasis-100);
}

.dropdown-divider {
  margin: 0.5rem 0;
  border: none;
  border-top: 1px solid var(--ifm-toc-border-color);
}

.auth-buttons {
  display: flex;
  gap: 0.75rem;
  align-items: center;
}

.btn {
  padding: 0.5rem 1rem;
  border-radius: 6px;
  border: 1px solid transparent;
  font-weight: 500;
  cursor: pointer;
  transition: all 0.2s ease;
  text-decoration: none;
  display: inline-block;
}

.btn-outline {
  background: transparent;
  border-color: var(--ifm-color-primary);
  color: var(--ifm-color-primary);
}

.btn-outline:hover {
  background-color: var(--ifm-color-primary);
  color: white;
}

.btn-primary {
  background-color: var(--ifm-color-primary);
  color: white;
  border-color: var(--ifm-color-primary);
}

.btn-primary:hover {
  background-color: var(--ifm-color-primary-dark);
  border-color: var(--ifm-color-primary-dark);
  transform: translateY(-1px);
  box-shadow: 0 4px 8px rgba(0, 0, 0, 0.2);
}

.menu-toggle {
  display: none;
  flex-direction: column;
  background: none;
  border: none;
  cursor: pointer;
  padding: 0.5rem;
}

.menu-toggle span {
  width: 25px;
  height: 3px;
  background-color: var(--ifm-color-emphasis-700);
  margin: 3px 0;
  transition: 0.3s;
}

.mobile-menu {
  display: none;
  flex-direction: column;
  background-color: var(--ifm-background-color);
  border-top: 1px solid var(--ifm-toc-border-color);
  padding: 1rem;
  position: absolute;
  top: 100%;
  left: 0;
  right: 0;
  box-shadow: 0 4px 12px rgba(0, 0, 0, 0.1);
}

.mobile-nav-link {
  padding: 0.75rem 1rem;
  text-decoration: none;
  color: var(--ifm-color-emphasis-800);
  border-radius: 4px;
  margin: 0.25rem 0;
}

.mobile-nav-link:hover {
  background-color: var(--ifm-color-emphasis-100);
}

.mobile-auth-buttons {
  display: flex;
  gap: 0.75rem;
  padding: 1rem 0;
  border-top: 1px solid var(--ifm-toc-border-color);
  margin-top: 1rem;
}

/* Responsive Design */
@media (max-width: 996px) {
  .navbar-menu.desktop {
    display: none;
  }

  .auth-buttons {
    display: none;
  }

  .menu-toggle {
    display: flex;
  }

  .mobile-menu.show {
    display: flex;
  }

  .navbar-container {
    padding: 0.5rem 1rem;
  }
}

/* Animations */
@keyframes fadeIn {
  from { opacity: 0; transform: translateY(-10px); }
  to { opacity: 1; transform: translateY(0); }
}

.dropdown-menu {
  animation: fadeIn 0.2s ease-out;
}

/* Theme-specific adjustments */
html[data-theme="dark"] .nav-link {
  color: var(--ifm-navbar-link-color);
}

html[data-theme="dark"] .nav-link:hover {
  color: var(--ifm-navbar-link-hover-color);
  background-color: var(--ifm-color-emphasis-200);
}

html[data-theme="dark"] .nav-icon {
  color: var(--ifm-color-emphasis-300);
}

html[data-theme="dark"] .nav-icon:hover {
  background-color: var(--ifm-color-emphasis-300);
  color: var(--ifm-color-primary-light);
}
`;

  // Write the files
  fs.writeFileSync(navbarPath, navbarComponent);
  fs.writeFileSync(navbarCssPath, navbarCss);

  console.log('Responsive navbar created successfully!');
  console.log(`Created ${navbarPath}`);
  console.log(`Created ${navbarCssPath}`);

  // Also update the Docusaurus config to use the custom navbar
  const configPath = path.join(frontendDir, 'docusaurus.config.js');
  if (fs.existsSync(configPath)) {
    let configContent = fs.readFileSync(configPath, 'utf8');

    // Add import for the custom navbar if not already present
    if (!configContent.includes('./src/components/Navbar/Navbar')) {
      configContent = `import Navbar from './src/components/Navbar/Navbar';\n` + configContent;

      // Replace the navbar config with custom component
      configContent = configContent.replace(
        /themeConfig:\s*{/i,
        `themeConfig: {
    navbar: {
      component: Navbar,
    },`
      );

      fs.writeFileSync(configPath, configContent);
      console.log('Updated docusaurus.config.js to use custom navbar');
    }
  }
}

// Export the function
module.exports = { createNavbar };

// If running directly
if (require.main === module) {
  const frontendDir = process.argv[2] || './frontend';
  createNavbar(frontendDir);
}