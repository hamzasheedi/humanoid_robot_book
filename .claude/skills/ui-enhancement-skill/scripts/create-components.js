const fs = require('fs');
const path = require('path');

/**
 * Generates JSX/React components for navbar, floating user menu,
 * chatbot window, and section layout wrappers
 */
function createComponents(frontendDir = './frontend') {
  const componentsDir = path.join(frontendDir, 'src', 'components');

  // Create the components directory if it doesn't exist
  if (!fs.existsSync(componentsDir)) {
    fs.mkdirSync(componentsDir, { recursive: true });
  }

  // Create UserMenu component
  const userMenuComponent = `import React, { useState, useRef, useEffect } from 'react';
import './UserMenu.css';

const UserMenu = () => {
  const [isOpen, setIsOpen] = useState(false);
  const menuRef = useRef(null);

  const toggleMenu = () => {
    setIsOpen(!isOpen);
  };

  // Close menu when clicking outside
  useEffect(() => {
    const handleClickOutside = (event) => {
      if (menuRef.current && !menuRef.current.contains(event.target)) {
        setIsOpen(false);
      }
    };

    document.addEventListener('mousedown', handleClickOutside);
    return () => {
      document.removeEventListener('mousedown', handleClickOutside);
    };
  }, []);

  return (
    <div className="user-menu" ref={menuRef}>
      <button
        className="user-menu-button"
        onClick={toggleMenu}
        aria-expanded={isOpen}
        aria-label="User menu"
      >
        <svg width="24" height="24" viewBox="0 0 24 24" fill="currentColor">
          <path d="M12 12c2.21 0 4-1.79 4-4s-1.79-4-4-4-4 1.79-4 4 1.79 4 4 4zm0 2c-2.67 0-8 1.34-8 4v2h16v-2c0-2.66-5.33-4-8-4z"/>
        </svg>
      </button>

      {isOpen && (
        <div className="user-menu-dropdown">
          <div className="user-menu-header">
            <div className="user-menu-avatar">
              <svg width="40" height="40" viewBox="0 0 24 24" fill="currentColor">
                <path d="M12 12c2.21 0 4-1.79 4-4s-1.79-4-4-4-4 1.79-4 4 1.79 4 4 4zm0 2c-2.67 0-8 1.34-8 4v2h16v-2c0-2.66-5.33-4-8-4z"/>
              </svg>
            </div>
            <div className="user-menu-info">
              <div className="user-menu-name">John Doe</div>
              <div className="user-menu-email">john@example.com</div>
            </div>
          </div>

          <div className="user-menu-divider"></div>

          <a href="/profile" className="user-menu-item">
            <svg width="18" height="18" viewBox="0 0 24 24" fill="currentColor">
              <path d="M12 12c2.21 0 4-1.79 4-4s-1.79-4-4-4-4 1.79-4 4 1.79 4 4 4zm0 2c-2.67 0-8 1.34-8 4v2h16v-2c0-2.66-5.33-4-8-4z"/>
            </svg>
            <span>Profile</span>
          </a>

          <a href="/settings" className="user-menu-item">
            <svg width="18" height="18" viewBox="0 0 24 24" fill="currentColor">
              <path d="M19.14,12.94c0.04-0.3,0.06-0.61,0.06-0.94c0-0.32-0.02-0.64-0.07-0.94l2.03-1.58c0.18-0.14,0.23-0.41,0.12-0.61 l-1.92-3.32c-0.12-0.22-0.37-0.29-0.59-0.22l-2.39,0.96c-0.5-0.38-1.03-0.7-1.62-0.94L14.4,2.81c-0.04-0.24-0.24-0.41-0.48-0.41 h-3.84c-0.24,0-0.43,0.17-0.47,0.41L9.25,5.35C8.66,5.59,8.12,5.92,7.63,6.29L5.24,5.33c-0.22-0.08-0.47,0-0.59,0.22L2.74,8.87 C2.62,9.08,2.66,9.34,2.86,9.48l2.03,1.58C4.84,11.36,4.82,11.69,4.82,12s0.02,0.64,0.07,0.94l-2.03,1.58 c-0.18,0.14-0.23,0.41-0.12,0.61l1.92,3.32c0.12,0.22,0.37,0.29,0.59,0.22l2.39-0.96c0.5,0.38,1.03,0.7,1.62,0.94l0.36,2.54 c0.05,0.24,0.24,0.41,0.48,0.41h3.84c0.24,0,0.44-0.17,0.47-0.41l0.36-2.54c0.59-0.24,1.13-0.56,1.62-0.94l2.39,0.96 c0.22,0.08,0.47,0,0.59-0.22l1.92-3.32c0.12-0.22,0.07-0.47-0.12-0.61L19.14,12.94z M12,15.6c-1.98,0-3.6-1.62-3.6-3.6 s1.62-3.6,3.6-3.6s3.6,1.62,3.6,3.6S13.98,15.6,12,15.6z"/>
            </svg>
            <span>Settings</span>
          </a>

          <a href="/bookmarks" className="user-menu-item">
            <svg width="18" height="18" viewBox="0 0 24 24" fill="currentColor">
              <path d="M17,3H7C5.9,3,5,3.9,5,5v16l7-3l7,3V5C19,3.9,18.1,3,17,3z"/>
            </svg>
            <span>Bookmarks</span>
          </a>

          <div className="user-menu-divider"></div>

          <a href="/logout" className="user-menu-item user-menu-item--danger">
            <svg width="18" height="18" viewBox="0 0 24 24" fill="currentColor">
              <path d="M14,12L6,6V5.5A1.5,1.5 0 0,1 7.5,4A1.5,1.5 0 0,1 9,5.5V6H14V5.5A1.5,1.5 0 0,1 15.5,4A1.5,1.5 0 0,1 17,5.5V18.5A1.5,1.5 0 0,1 15.5,20A1.5,1.5 0 0,1 14,18.5V18H9V18.5A1.5,1.5 0 0,1 7.5,20A1.5,1.5 0 0,1 6,18.5V18L14,12M20,9V12H23V14H20V17H18V14H15V12H18V9H20Z"/>
            </svg>
            <span>Sign Out</span>
          </a>
        </div>
      )}
    </div>
  );
};

export default UserMenu;
`;

  // Create UserMenu CSS
  const userMenuCss = `/* User Menu Styles */
.user-menu {
  position: relative;
  display: inline-block;
}

.user-menu-button {
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

.user-menu-button:hover {
  background-color: var(--ifm-color-emphasis-100);
  transform: translateY(-2px);
}

.user-menu-button:focus {
  outline: 2px solid var(--ifm-color-primary);
  outline-offset: 2px;
}

.user-menu-dropdown {
  position: absolute;
  top: 100%;
  right: 0;
  background-color: var(--ifm-background-color);
  border: 1px solid var(--ifm-toc-border-color);
  border-radius: 8px;
  box-shadow: var(--ifm-global-shadow-xl);
  min-width: 240px;
  z-index: 1001;
  margin-top: 0.5rem;
  animation: fadeIn 0.2s ease-out;
}

.user-menu-header {
  display: flex;
  align-items: center;
  padding: 1rem;
  border-bottom: 1px solid var(--ifm-toc-border-color);
}

.user-menu-avatar {
  margin-right: 0.75rem;
}

.user-menu-info {
  flex: 1;
}

.user-menu-name {
  font-weight: 600;
  margin-bottom: 0.25rem;
  color: var(--ifm-heading-color);
}

.user-menu-email {
  font-size: 0.875rem;
  color: var(--ifm-color-emphasis-600);
}

.user-menu-divider {
  height: 1px;
  background-color: var(--ifm-toc-border-color);
  margin: 0.5rem 0;
}

.user-menu-item {
  display: flex;
  align-items: center;
  gap: 0.75rem;
  padding: 0.75rem 1rem;
  text-decoration: none;
  color: var(--ifm-color-emphasis-800);
  transition: background-color 0.2s ease;
  border-radius: 0;
}

.user-menu-item:hover {
  background-color: var(--ifm-color-emphasis-100);
}

.user-menu-item--danger {
  color: var(--ifm-color-danger);
}

.user-menu-item--danger:hover {
  background-color: rgba(244, 67, 54, 0.1);
}

.user-menu-item svg {
  flex-shrink: 0;
}

.user-menu-item span {
  flex: 1;
}

/* Animations */
@keyframes fadeIn {
  from { opacity: 0; transform: translateY(-10px); }
  to { opacity: 1; transform: translateY(0); }
}

/* Dark mode specific adjustments */
html[data-theme="dark"] .user-menu-dropdown {
  background-color: #1e1e1e;
  border-color: #333333;
}

html[data-theme="dark"] .user-menu-header {
  border-color: #333333;
}

html[data-theme="dark"] .user-menu-item {
  color: #e0e0e0;
}

html[data-theme="dark"] .user-menu-item:hover {
  background-color: #2d2d2d;
}

/* Responsive */
@media (max-width: 768px) {
  .user-menu-dropdown {
    min-width: 200px;
    right: auto;
    left: 0;
  }
}
`;

  // Create SectionWrapper component
  const sectionWrapperComponent = `import React from 'react';
import './SectionWrapper.css';

const SectionWrapper = ({
  children,
  className = '',
  title,
  subtitle,
  centered = false,
  background = 'default',
  padding = 'default'
}) => {
  const sectionClasses = [
    'section-wrapper',
    centered && 'section-wrapper--centered',
    \`section-wrapper--bg-\${background}\`,
    \`section-wrapper--padding-\${padding}\`,
    className
  ].filter(Boolean).join(' ');

  return (
    <section className={sectionClasses}>
      {(title || subtitle) && (
        <div className="section-wrapper__header">
          {title && <h2 className="section-wrapper__title">{title}</h2>}
          {subtitle && <p className="section-wrapper__subtitle">{subtitle}</p>}
        </div>
      )}
      <div className="section-wrapper__content">
        {children}
      </div>
    </section>
  );
};

export default SectionWrapper;
`;

  // Create SectionWrapper CSS
  const sectionWrapperCss = `/* Section Wrapper Styles */
.section-wrapper {
  width: 100%;
  position: relative;
}

.section-wrapper--centered {
  text-align: center;
}

.section-wrapper--bg-default {
  background-color: var(--ifm-background-color);
}

.section-wrapper--bg-muted {
  background-color: var(--ifm-color-emphasis-100);
}

.section-wrapper--bg-primary {
  background-color: rgba(var(--ifm-color-primary-rgb), 0.05);
}

.section-wrapper--bg-gradient {
  background: linear-gradient(135deg, var(--ifm-color-primary) 0%, #667eea 100%);
  color: white;
}

.section-wrapper--padding-default {
  padding: 4rem 1rem;
}

.section-wrapper--padding-sm {
  padding: 2rem 1rem;
}

.section-wrapper--padding-lg {
  padding: 6rem 1rem;
}

.section-wrapper--padding-xl {
  padding: 8rem 1rem;
}

.section-wrapper__header {
  margin-bottom: 3rem;
  max-width: 800px;
  margin-left: auto;
  margin-right: auto;
}

.section-wrapper--centered .section-wrapper__header {
  text-align: center;
}

.section-wrapper__title {
  font-size: 2.5rem;
  font-weight: 700;
  margin-bottom: 1rem;
  color: var(--ifm-heading-color);
}

.section-wrapper--centered .section-wrapper__title {
  text-align: center;
}

.section-wrapper__subtitle {
  font-size: 1.25rem;
  color: var(--ifm-color-emphasis-700);
  line-height: 1.6;
  margin: 0;
}

.section-wrapper--centered .section-wrapper__subtitle {
  text-align: center;
}

.section-wrapper__content {
  max-width: 1200px;
  margin: 0 auto;
}

/* Dark mode specific adjustments */
html[data-theme="dark"] .section-wrapper--bg-muted {
  background-color: #1e1e1e;
}

html[data-theme="dark"] .section-wrapper--bg-primary {
  background-color: rgba(37, 194, 160, 0.1);
}

/* Responsive */
@media (max-width: 768px) {
  .section-wrapper--padding-default {
    padding: 3rem 1rem;
  }

  .section-wrapper--padding-lg,
  .section-wrapper--padding-xl {
    padding: 4rem 1rem;
  }

  .section-wrapper__title {
    font-size: 2rem;
  }

  .section-wrapper__subtitle {
    font-size: 1.125rem;
  }

  .section-wrapper__header {
    margin-bottom: 2rem;
  }
}

@media (max-width: 480px) {
  .section-wrapper--padding-default {
    padding: 2rem 1rem;
  }

  .section-wrapper__title {
    font-size: 1.75rem;
  }
}
`;

  // Write the files
  fs.writeFileSync(path.join(componentsDir, 'UserMenu.jsx'), userMenuComponent);
  fs.writeFileSync(path.join(componentsDir, 'UserMenu.css'), userMenuCss);
  fs.writeFileSync(path.join(componentsDir, 'SectionWrapper.jsx'), sectionWrapperComponent);
  fs.writeFileSync(path.join(componentsDir, 'SectionWrapper.css'), sectionWrapperCss);

  console.log('React components created successfully!');
  console.log(`Created ${path.join(componentsDir, 'UserMenu.jsx')}`);
  console.log(`Created ${path.join(componentsDir, 'UserMenu.css')}`);
  console.log(`Created ${path.join(componentsDir, 'SectionWrapper.jsx')}`);
  console.log(`Created ${path.join(componentsDir, 'SectionWrapper.css')}`);

  // Create index.js to export all components
  const indexJsContent = `// UI Enhancement Components
export { default as UserMenu } from './UserMenu';
export { default as SectionWrapper } from './SectionWrapper';
export { default as ChatbotWindow } from './ChatbotWindow';
// Note: Navbar is created separately as it has its own directory structure
`;

  fs.writeFileSync(path.join(componentsDir, 'index.js'), indexJsContent);
  console.log(`Created ${path.join(componentsDir, 'index.js')}`);
}

// Export the function
module.exports = { createComponents };

// If running directly
if (require.main === module) {
  const frontendDir = process.argv[2] || './frontend';
  createComponents(frontendDir);
}