const fs = require('fs');
const path = require('path');

/**
 * Generates clean, modular CSS for cards, module grids,
 * accented borders, typography, timelines, buttons, and responsive scaling
 */
function generateCSS(frontendDir = './frontend') {
  const cssDir = path.join(frontendDir, 'src', 'css');
  const modularCssPath = path.join(cssDir, 'modular-styles.css');

  // Create the css directory if it doesn't exist
  if (!fs.existsSync(cssDir)) {
    fs.mkdirSync(cssDir, { recursive: true });
  }

  // Create modular CSS file
  const modularCss = `/* Modular CSS for the Physical AI & Humanoid Robotics Textbook */

/* Card Styles */
.card {
  background-color: var(--ifm-card-background-color, #ffffff);
  border: 1px solid var(--ifm-card-border-color, #e0e0e0);
  border-radius: 8px;
  padding: 1.5rem;
  box-shadow: var(--ifm-global-shadow-lw);
  transition: all 0.3s ease;
  margin-bottom: 1rem;
}

.card:hover {
  transform: translateY(-4px);
  box-shadow: var(--ifm-global-shadow-md);
  border-color: var(--ifm-color-primary);
}

.card--primary {
  border-left: 4px solid var(--ifm-color-primary);
}

.card--secondary {
  border-left: 4px solid var(--ifm-color-secondary);
}

.card--success {
  border-left: 4px solid var(--ifm-color-success);
}

.card--info {
  border-left: 4px solid var(--ifm-color-info);
}

.card--warning {
  border-left: 4px solid var(--ifm-color-warning);
}

.card--danger {
  border-left: 4px solid var(--ifm-color-danger);
}

.card__header {
  margin-bottom: 1rem;
  padding-bottom: 0.75rem;
  border-bottom: 1px solid var(--ifm-card-border-color, #e0e0e0);
}

.card__body {
  flex: 1 1 auto;
}

.card__footer {
  margin-top: 1rem;
  padding-top: 0.75rem;
  border-top: 1px solid var(--ifm-card-border-color, #e0e0e0);
}

/* Module Grid */
.module-grid {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(300px, 1fr));
  gap: 2rem;
  margin: 2rem 0;
}

.module-grid--large {
  grid-template-columns: repeat(auto-fit, minmax(400px, 1fr));
}

.module-grid--small {
  grid-template-columns: repeat(auto-fit, minmax(250px, 1fr));
}

.module-card {
  display: flex;
  flex-direction: column;
  height: 100%;
  transition: all 0.3s ease;
}

.module-card:hover {
  transform: translateY(-5px);
  box-shadow: var(--ifm-global-shadow-md);
}

.module-icon {
  font-size: 2.5rem;
  margin-bottom: 1rem;
  color: var(--ifm-color-primary);
}

.module-title {
  font-size: 1.25rem;
  font-weight: 600;
  margin-bottom: 0.75rem;
  color: var(--ifm-heading-color);
}

.module-description {
  color: var(--ifm-color-emphasis-700);
  flex-grow: 1;
  margin-bottom: 1rem;
}

.module-link {
  display: inline-flex;
  align-items: center;
  gap: 0.5rem;
  color: var(--ifm-color-primary);
  text-decoration: none;
  font-weight: 500;
  transition: color 0.2s ease;
}

.module-link:hover {
  color: var(--ifm-color-primary-dark);
}

/* Accented Borders */
.border-accent--primary {
  border: 2px solid var(--ifm-color-primary) !important;
}

.border-accent--secondary {
  border: 2px solid var(--ifm-color-secondary) !important;
}

.border-accent--success {
  border: 2px solid var(--ifm-color-success) !important;
}

.border-accent--info {
  border: 2px solid var(--ifm-color-info) !important;
}

.border-accent--warning {
  border: 2px solid var(--ifm-color-warning) !important;
}

.border-accent--danger {
  border: 2px solid var(--ifm-color-danger) !important;
}

.border-left-accent--primary {
  border-left: 4px solid var(--ifm-color-primary) !important;
}

.border-left-accent--secondary {
  border-left: 4px solid var(--ifm-color-secondary) !important;
}

.border-left-accent--success {
  border-left: 4px solid var(--ifm-color-success) !important;
}

.border-left-accent--info {
  border-left: 4px solid var(--ifm-color-info) !important;
}

.border-left-accent--warning {
  border-left: 4px solid var(--ifm-color-warning) !important;
}

.border-left-accent--danger {
  border-left: 4px solid var(--ifm-color-danger) !important;
}

/* Typography */
.typography--display {
  font-size: 3rem;
  font-weight: 700;
  line-height: 1.2;
  margin-bottom: 1rem;
}

.typography--heading {
  font-size: 2rem;
  font-weight: 600;
  line-height: 1.3;
  margin-bottom: 1rem;
}

.typography--subheading {
  font-size: 1.5rem;
  font-weight: 600;
  line-height: 1.4;
  margin-bottom: 1rem;
}

.typography--body-large {
  font-size: 1.125rem;
  line-height: 1.6;
}

.typography--body-small {
  font-size: 0.875rem;
  line-height: 1.5;
}

.typography--caption {
  font-size: 0.75rem;
  color: var(--ifm-color-emphasis-600);
}

.text-primary {
  color: var(--ifm-color-primary) !important;
}

.text-secondary {
  color: var(--ifm-color-secondary) !important;
}

.text-success {
  color: var(--ifm-color-success) !important;
}

.text-info {
  color: var(--ifm-color-info) !important;
}

.text-warning {
  color: var(--ifm-color-warning) !important;
}

.text-danger {
  color: var(--ifm-color-danger) !important;
}

.text-muted {
  color: var(--ifm-color-emphasis-600) !important;
}

.font-weight-light {
  font-weight: 300 !important;
}

.font-weight-normal {
  font-weight: 400 !important;
}

.font-weight-medium {
  font-weight: 500 !important;
}

.font-weight-semibold {
  font-weight: 600 !important;
}

.font-weight-bold {
  font-weight: 700 !important;
}

/* Timeline */
.timeline {
  position: relative;
  padding: 2rem 0;
}

.timeline::before {
  content: '';
  position: absolute;
  top: 0;
  bottom: 0;
  width: 2px;
  background: var(--ifm-color-emphasis-300);
  left: 1rem;
}

.timeline-item {
  position: relative;
  margin-bottom: 2rem;
  padding-left: 3rem;
}

.timeline-item::before {
  content: '';
  position: absolute;
  width: 16px;
  height: 16px;
  border-radius: 50%;
  background: var(--ifm-color-primary);
  left: 0.5rem;
  top: 0.5rem;
  border: 3px solid var(--ifm-background-color);
}

.timeline-item--completed::before {
  background: var(--ifm-color-success);
}

.timeline-item--current::before {
  background: var(--ifm-color-warning);
  box-shadow: 0 0 0 4px rgba(255, 152, 0, 0.2);
}

.timeline-date {
  font-size: 0.875rem;
  color: var(--ifm-color-emphasis-600);
  margin-bottom: 0.25rem;
}

.timeline-title {
  font-weight: 600;
  margin-bottom: 0.5rem;
  color: var(--ifm-heading-color);
}

.timeline-content {
  color: var(--ifm-color-emphasis-700);
}

/* Buttons */
.btn {
  display: inline-flex;
  align-items: center;
  justify-content: center;
  padding: 0.75rem 1.5rem;
  border-radius: 6px;
  border: 1px solid transparent;
  font-weight: 500;
  font-size: 1rem;
  cursor: pointer;
  transition: all 0.2s ease;
  text-decoration: none;
  gap: 0.5rem;
  position: relative;
  overflow: hidden;
}

.btn:focus {
  outline: 2px solid var(--ifm-color-primary);
  outline-offset: 2px;
}

.btn:hover {
  transform: translateY(-2px);
  box-shadow: var(--ifm-global-shadow-md);
}

.btn:active {
  transform: translateY(0);
}

.btn--primary {
  background-color: var(--ifm-color-primary);
  color: white;
  border-color: var(--ifm-color-primary);
}

.btn--primary:hover {
  background-color: var(--ifm-color-primary-dark);
  border-color: var(--ifm-color-primary-dark);
}

.btn--secondary {
  background-color: var(--ifm-color-secondary);
  color: white;
  border-color: var(--ifm-color-secondary);
}

.btn--secondary:hover {
  background-color: var(--ifm-color-secondary-dark);
  border-color: var(--ifm-color-secondary-dark);
}

.btn--success {
  background-color: var(--ifm-color-success);
  color: white;
  border-color: var(--ifm-color-success);
}

.btn--success:hover {
  background-color: var(--ifm-color-success-dark);
  border-color: var(--ifm-color-success-dark);
}

.btn--outline-primary {
  background-color: transparent;
  color: var(--ifm-color-primary);
  border-color: var(--ifm-color-primary);
}

.btn--outline-primary:hover {
  background-color: var(--ifm-color-primary);
  color: white;
}

.btn--outline-secondary {
  background-color: transparent;
  color: var(--ifm-color-secondary);
  border-color: var(--ifm-color-secondary);
}

.btn--outline-secondary:hover {
  background-color: var(--ifm-color-secondary);
  color: white;
}

.btn--link {
  background: none;
  border: none;
  color: var(--ifm-color-primary);
  text-decoration: underline;
  padding: 0;
  font-weight: normal;
}

.btn--link:hover {
  color: var(--ifm-color-primary-dark);
  text-decoration: none;
}

.btn--sm {
  padding: 0.5rem 1rem;
  font-size: 0.875rem;
}

.btn--lg {
  padding: 1rem 2rem;
  font-size: 1.125rem;
}

.btn--block {
  display: block;
  width: 100%;
}

.btn:disabled,
.btn--disabled {
  opacity: 0.6;
  cursor: not-allowed;
  transform: none !important;
  box-shadow: none !important;
}

/* Responsive Scaling */
.scalable-container {
  max-width: 100%;
  width: 100%;
}

.scalable-text {
  font-size: clamp(1rem, 2.5vw, 1.5rem);
}

.scalable-heading {
  font-size: clamp(1.5rem, 4vw, 3rem);
}

.scalable-grid {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(min(300px, 100%), 1fr));
  gap: clamp(1rem, 2vw, 2rem);
}

/* Responsive Utilities */
@media (min-width: 576px) {
  .module-grid {
    grid-template-columns: repeat(auto-fit, minmax(320px, 1fr));
  }
}

@media (min-width: 768px) {
  .module-grid {
    grid-template-columns: repeat(auto-fit, minmax(350px, 1fr));
  }

  .module-grid--large {
    grid-template-columns: repeat(auto-fit, minmax(450px, 1fr));
  }

  .timeline::before {
    left: 2rem;
  }

  .timeline-item {
    padding-left: 4rem;
  }

  .timeline-item::before {
    left: 1.5rem;
  }
}

@media (min-width: 992px) {
  .module-grid {
    grid-template-columns: repeat(auto-fit, minmax(400px, 1fr));
  }

  .scalable-grid {
    grid-template-columns: repeat(auto-fit, minmax(min(350px, 100%), 1fr));
  }
}

@media (min-width: 1200px) {
  .module-grid {
    grid-template-columns: repeat(auto-fit, minmax(450px, 1fr));
  }

  .scalable-grid {
    grid-template-columns: repeat(auto-fit, minmax(min(400px, 100%), 1fr));
  }
}

/* Animations */
@keyframes fadeInUp {
  from {
    opacity: 0;
    transform: translateY(20px);
  }
  to {
    opacity: 1;
    transform: translateY(0);
  }
}

.fade-in-up {
  animation: fadeInUp 0.6s ease-out;
}

@keyframes slideInLeft {
  from {
    opacity: 0;
    transform: translateX(-20px);
  }
  to {
    opacity: 1;
    transform: translateX(0);
  }
}

.slide-in-left {
  animation: slideInLeft 0.5s ease-out;
}

@keyframes slideInRight {
  from {
    opacity: 0;
    transform: translateX(20px);
  }
  to {
    opacity: 1;
    transform: translateX(0);
  }
}

.slide-in-right {
  animation: slideInRight 0.5s ease-out;
}

/* Utility Classes */
.d-flex {
  display: flex !important;
}

.d-block {
  display: block !important;
}

.d-inline {
  display: inline !important;
}

.d-inline-block {
  display: inline-block !important;
}

.flex-column {
  flex-direction: column !important;
}

.flex-row {
  flex-direction: row !important;
}

.justify-content-center {
  justify-content: center !important;
}

.justify-content-between {
  justify-content: space-between !important;
}

.justify-content-around {
  justify-content: space-around !important;
}

.align-items-center {
  align-items: center !important;
}

.align-items-start {
  align-items: flex-start !important;
}

.align-items-end {
  align-items: flex-end !important;
}

.m-0 { margin: 0 !important; }
.m-1 { margin: 0.25rem !important; }
.m-2 { margin: 0.5rem !important; }
.m-3 { margin: 1rem !important; }
.m-4 { margin: 1.5rem !important; }
.m-5 { margin: 3rem !important; }

.mt-0 { margin-top: 0 !important; }
.mt-1 { margin-top: 0.25rem !important; }
.mt-2 { margin-top: 0.5rem !important; }
.mt-3 { margin-top: 1rem !important; }
.mt-4 { margin-top: 1.5rem !important; }
.mt-5 { margin-top: 3rem !important; }

.mb-0 { margin-bottom: 0 !important; }
.mb-1 { margin-bottom: 0.25rem !important; }
.mb-2 { margin-bottom: 0.5rem !important; }
.mb-3 { margin-bottom: 1rem !important; }
.mb-4 { margin-bottom: 1.5rem !important; }
.mb-5 { margin-bottom: 3rem !important; }

.p-0 { padding: 0 !important; }
.p-1 { padding: 0.25rem !important; }
.p-2 { padding: 0.5rem !important; }
.p-3 { padding: 1rem !important; }
.p-4 { padding: 1.5rem !important; }
.p-5 { padding: 3rem !important; }

.pt-0 { padding-top: 0 !important; }
.pt-1 { padding-top: 0.25rem !important; }
.pt-2 { padding-top: 0.5rem !important; }
.pt-3 { padding-top: 1rem !important; }
.pt-4 { padding-top: 1.5rem !important; }
.pt-5 { padding-top: 3rem !important; }

.pb-0 { padding-bottom: 0 !important; }
.pb-1 { padding-bottom: 0.25rem !important; }
.pb-2 { padding-bottom: 0.5rem !important; }
.pb-3 { padding-bottom: 1rem !important; }
.pb-4 { padding-bottom: 1.5rem !important; }
.pb-5 { padding-bottom: 3rem !important; }

.w-100 { width: 100% !important; }
.h-100 { height: 100% !important; }

.text-center { text-align: center !important; }
.text-left { text-align: left !important; }
.text-right { text-align: right !important; }

.text-uppercase { text-transform: uppercase !important; }
.text-lowercase { text-transform: lowercase !important; }
.text-capitalize { text-transform: capitalize !important; }

.text-nowrap { white-space: nowrap !important; }
.text-truncate { overflow: hidden; text-overflow: ellipsis; white-space: nowrap; }

/* Accessibility */
.sr-only {
  position: absolute;
  width: 1px;
  height: 1px;
  padding: 0;
  margin: -1px;
  overflow: hidden;
  clip: rect(0, 0, 0, 0);
  white-space: nowrap;
  border: 0;
}

.focusable:focus {
  outline: 2px solid var(--ifm-color-primary);
  outline-offset: 2px;
}

/* Print styles */
@media print {
  .no-print {
    display: none !important;
  }

  .card {
    box-shadow: none;
    border: 1px solid #ccc;
  }
}
`;

  // Write the CSS file
  fs.writeFileSync(modularCssPath, modularCss);

  console.log('Modular CSS generated successfully!');
  console.log(`Created ${modularCssPath}`);

  // Update docusaurus.config.js to include the modular CSS
  const configPath = path.join(frontendDir, 'docusaurus.config.js');
  if (fs.existsSync(configPath)) {
    let configContent = fs.readFileSync(configPath, 'utf8');

    // Add the modular CSS import to the stylesheets if not already present
    if (!configContent.includes('src/css/modular-styles.css')) {
      if (configContent.includes('stylesheets: [')) {
        // Add to existing stylesheets array
        configContent = configContent.replace(
          /(stylesheets:\s*\[)/,
          `$1\n        {\n          href: '/src/css/modular-styles.css',\n          type: 'text/css',\n        },`
        );
      } else if (configContent.includes('themeConfig: {')) {
        // Add stylesheets section after themeConfig opening
        configContent = configContent.replace(
          /themeConfig:\s*{/,
          `stylesheets: [\n        {\n          href: '/src/css/modular-styles.css',\n          type: 'text/css',\n        },\n      ],\n      themeConfig: {`
        );
      }

      fs.writeFileSync(configPath, configContent);
      console.log('Updated docusaurus.config.js to include modular CSS');
    }
  }
}

// Export the function
module.exports = { generateCSS };

// If running directly
if (require.main === module) {
  const frontendDir = process.argv[2] || './frontend';
  generateCSS(frontendDir);
}