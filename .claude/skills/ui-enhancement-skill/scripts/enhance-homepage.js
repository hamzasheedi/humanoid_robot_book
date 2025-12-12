const fs = require('fs');
const path = require('path');

/**
 * Enhances the Docusaurus homepage by updating the index.md file
 * with modern structure, layout blocks, iconography, and improved organization
 */
function enhanceHomepage(frontendDir = './frontend') {
  const indexPath = path.join(frontendDir, 'src', 'pages', 'index.module.css');
  const indexMdPath = path.join(frontendDir, 'src', 'pages', 'index.js'); // or index.md if using markdown

  // Create or update the homepage CSS module
  const homeCssContent = `
.heroBanner {
  padding: 4rem 0;
  text-align: center;
  position: relative;
  overflow: hidden;
  background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
  color: white;
}

@media screen and (max-width: 996px) {
  .heroBanner {
    padding: 2rem 0;
  }
}

.buttons {
  display: flex;
  align-items: center;
  justify-content: center;
  gap: 1rem;
  margin-top: 2rem;
}

.features {
  display: flex;
  align-items: center;
  padding: 2rem 0;
  width: 100%;
  background-color: var(--ifm-background-color);
}

.featureSvg {
  height: 200px;
  width: 200px;
  margin-bottom: 1rem;
}

.featureCard {
  padding: 2rem;
  border-radius: 8px;
  box-shadow: 0 4px 6px rgba(0, 0, 0, 0.1);
  transition: transform 0.3s ease, box-shadow 0.3s ease;
  margin-bottom: 1rem;
}

.featureCard:hover {
  transform: translateY(-5px);
  box-shadow: 0 8px 15px rgba(0, 0, 0, 0.2);
}

.timelineStep {
  display: flex;
  align-items: center;
  margin-bottom: 2rem;
  padding: 1rem;
  border-left: 3px solid var(--ifm-color-primary);
  background-color: var(--ifm-card-background-color);
  border-radius: 0 4px 4px 0;
}

.timelineIcon {
  font-size: 1.5rem;
  margin-right: 1rem;
  color: var(--ifm-color-primary);
}

.timelineContent {
  flex: 1;
}

.gridContainer {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(300px, 1fr));
  gap: 2rem;
  margin: 2rem 0;
}

.moduleCard {
  padding: 1.5rem;
  border-radius: 8px;
  border: 1px solid var(--ifm-color-emphasis-200);
  transition: all 0.3s ease;
}

.moduleCard:hover {
  border-color: var(--ifm-color-primary);
  box-shadow: 0 4px 12px rgba(0, 0, 0, 0.15);
}

.sectionTitle {
  text-align: center;
  margin-bottom: 3rem;
  font-size: 2.5rem;
  color: var(--ifm-heading-color);
}

.stepNumber {
  display: inline-flex;
  align-items: center;
  justify-content: center;
  width: 2.5rem;
  height: 2.5rem;
  border-radius: 50%;
  background-color: var(--ifm-color-primary);
  color: white;
  font-weight: bold;
  margin-right: 1rem;
}
`;

  // Create the index.js content for the enhanced homepage
  const homeJsContent = `import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageFeatures from '@site/src/components/HomepageFeatures';

import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <h1 className="hero__title">{siteConfig.title}</h1>
        <p className="hero__subtitle">{siteConfig.tagline}</p>
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="/docs/intro">
            Get Started
          </Link>
          <Link
            className="button button--primary button--lg"
            to="/docs/tutorial">
            Learn Robotics
          </Link>
        </div>
      </div>
    </header>
  );
}

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={\`Welcome to \${siteConfig.title}\`}
      description="Physical AI & Humanoid Robotics Textbook with integrated RAG chatbot">
      <HomepageHeader />
      <main>
        <section className={styles.features}>
          <div className="container">
            <h2 className={styles.sectionTitle}>Learning Modules</h2>
            <div className={styles.gridContainer}>
              <div className={styles.moduleCard}>
                <h3>Fundamentals of Robotics</h3>
                <p>Learn the foundational concepts of robotics, kinematics, and dynamics.</p>
              </div>
              <div className={styles.moduleCard}>
                <h3>Humanoid Design</h3>
                <p>Explore the principles behind humanoid robot design and biomechanics.</p>
              </div>
              <div className={styles.moduleCard}>
                <h3>AI Control Systems</h3>
                <p>Master advanced control algorithms for humanoid locomotion and manipulation.</p>
              </div>
            </div>
          </div>
        </section>

        <section className={styles.features}>
          <div className="container">
            <h2 className={styles.sectionTitle}>Learning Path</h2>
            <div className={styles.timelineStep}>
              <div className={styles.stepNumber}>1</div>
              <div className={styles.timelineContent}>
                <h3>Introduction to Physical AI</h3>
                <p>Understand the theoretical foundations of physical intelligence and embodied cognition.</p>
              </div>
            </div>
            <div className={styles.timelineStep}>
              <div className={styles.stepNumber}>2</div>
              <div className={styles.timelineContent}>
                <h3>Mechanical Design Principles</h3>
                <p>Learn about actuators, sensors, and mechanical structures for humanoid robots.</p>
              </div>
            </div>
            <div className={styles.timelineStep}>
              <div className={styles.stepNumber}>3</div>
              <div className={styles.timelineContent}>
                <h3>Control Algorithms</h3>
                <p>Implement advanced control systems for balance, locomotion, and interaction.</p>
              </div>
            </div>
            <div className={styles.timelineStep}>
              <div className={styles.stepNumber}>4</div>
              <div className={styles.timelineContent}>
                <h3>Integration & Testing</h3>
                <p>Combine all components and validate the performance of your humanoid system.</p>
              </div>
            </div>
          </div>
        </section>
      </main>
    </Layout>
  );
}
`;

  // Write the files
  fs.writeFileSync(indexPath, homeCssContent);
  fs.writeFileSync(indexMdPath, homeJsContent);

  console.log('Homepage enhanced successfully!');
  console.log(`Created ${indexPath}`);
  console.log(`Created ${indexMdPath}`);
}

// Export the function
module.exports = { enhanceHomepage };

// If running directly
if (require.main === module) {
  const frontendDir = process.argv[2] || './frontend';
  enhanceHomepage(frontendDir);
}