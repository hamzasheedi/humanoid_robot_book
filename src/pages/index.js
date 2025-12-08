import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import SetDarkModeDefault from '../components/SetDarkModeDefault';
import styles from './index.module.css';

// Define the ModuleCard component within this file
function ModuleCard({ title, description, link }) {
  return (
    <div className={clsx('col', 'col--4', 'margin-bottom--lg')}>
      <div className={clsx('card-demo', styles.moduleCard)}>
        <div className="card">
          <div className="card__body">
            <h3>{title}</h3>
            <p>{description}</p>
          </div>
          <div className="card__footer">
            <Link className="button button--secondary button--block" to={link}>
              Learn More
            </Link>
          </div>
        </div>
      </div>
    </div>
  );
}

export default function Home() {
  const { siteConfig } = useDocusaurusContext();

  return (
    <Layout
      title={`Physical AI & Humanoid Robotics`}
      description="AI Robotics Textbook covering ROS 2, Digital Twin simulation, NVIDIA Isaac, VLA and Capstone project">
      <SetDarkModeDefault /> {/* Set dark mode as default */}
      <header className={clsx('hero hero--primary', styles.heroBanner)}>
        <div className="container">
          <h1 className="hero__title">PHYSICAL AI & HUMANOID ROBOTICS</h1>
          <p className="hero__subtitle">AI Systems in the Physical World • Embodied Intelligence</p>
          <p className="hero__subtitle">Bridging the gap between <strong>digital intelligence</strong> and the <strong>physical body</strong> through humanoid robots.</p>
          <div className={styles.buttons}>
            <Link
              className="button button--secondary button--lg"
              to="/docs/modules/ros2/introduction">
              Get Started
            </Link>
            <Link
              className="button button--outline button--secondary button--lg"
              to="/docs/modules">
              Browse Modules
            </Link>
          </div>
        </div>
      </header>

      <main>
        {/* Course Overview Section */}
        <section className={clsx(styles.section, styles.courseTitleSection)}>
          <div className="container padding-horiz--md">
            <div className="row">
              <div className="col col--8 col--offset-2">
                <h2 className="text--center">COURSE OVERVIEW</h2>
                <p>
                  <strong>Focus and Theme:</strong> <em>AI Systems in the Physical World. Embodied Intelligence.</em>
                </p>
                <p>
                  <strong>Goal:</strong> <em>Bridging the gap between the digital brain and the physical body. Students apply their AI knowledge to control Humanoid Robots in simulated and real-world environments.</em>
                </p>
              </div>
            </div>
          </div>
        </section>

        {/* Quarter Overview Section */}
        <section className={clsx(styles.section, styles.quarterOverviewSection)}>
          <div className="container padding-horiz--md">
            <div className="row">
              <div className="col col--8 col--offset-2">
                <h2 className="text--center">QUARTER OVERVIEW</h2>
                <p>
                  The future of AI extends beyond digital spaces into real-world interaction. This capstone quarter introduces <strong>Physical AI</strong> — AI systems that understand physics, interact with the world, and control humanoid robots using ROS 2, Gazebo, Unity, and NVIDIA Isaac. Students will simulate, design, and deploy humanoid robots capable of natural interaction with their environment.
                </p>
                
                <div className="row padding-vert--md">
                  <div className="col col--4 text--center">
                    <h3>ROS 2</h3>
                    <p>Robotic Nervous System</p>
                  </div>
                  <div className="col col--4 text--center">
                    <h3>Gazebo / Unity</h3>
                    <p>Digital Twin Simulation</p>
                  </div>
                  <div className="col col--4 text--center">
                    <h3>NVIDIA Isaac</h3>
                    <p>AI Perception & Navigation</p>
                  </div>
                </div>
              </div>
            </div>
          </div>
        </section>

        {/* Modules Sequence Section - Showcase Style */}
        <section className={clsx(styles.section, styles.moduleSequenceSection)}>
          <div className="container padding-horiz--md">
            <div className="row">
              <div className="col col--10 col--offset-1">
                <h2 className="text--center">MODULES (SEQUENCE-WISE)</h2>
                
                <div className="module-sequence-container">
                  <div className="module-card">
                    <h2>1. ROS 2 (Robot Operating System 2)</h2>
                    <p>Foundational communication infrastructure for robotics</p>
                    <ul>
                      <li>ROS 2 fundamentals and implementation</li>
                      <li>Nodes, Topics, and Services</li>
                      <li>URDF for humanoid robots</li>
                      <li>Tutorials and exercises</li>
                    </ul>
                    <Link className="button button--secondary" to="/docs/modules/ros2/">Learn More</Link>
                  </div>
                  
                  <div className="module-card">
                    <h2>2. DIGITAL-TWIN</h2>
                    <p>Creating and validating robot behaviors in simulation</p>
                    <ul>
                      <li>Physics simulation in Gazebo</li>
                      <li>Visual rendering in Unity</li>
                      <li>Sensor simulation (LiDAR, IMU, cameras)</li>
                      <li>Digital Twin Hardware Compatibility Guidelines</li>
                    </ul>
                    <Link className="button button--secondary" to="/docs/modules/digital-twin/">Learn More</Link>
                  </div>
                  
                  <div className="module-card">
                    <h2>3. NVIDIA-ISAAC</h2>
                    <p>AI-powered perception and navigation</p>
                    <ul>
                      <li>NVIDIA Isaac Introduction</li>
                      <li>NVIDIA Isaac Perception Guide</li>
                      <li>NVIDIA Isaac Navigation Guide</li>
                      <li>Isaac Reinforcement Learning Modules</li>
                    </ul>
                    <Link className="button button--secondary" to="/docs/modules/nvidia-isaac/">Learn More</Link>
                  </div>
                  
                  <div className="module-card">
                    <h2>4. VLA (Vision-Language-Action)</h2>
                    <p>Integrating vision, language, and action for intelligent robot behavior</p>
                    <ul>
                      <li>Vision-Language-Action Introduction</li>
                      <li>Vision-Language-Action Voice Control Guide</li>
                      <li>Cognitive Planning for VLA Systems</li>
                      <li>VLA Model Compatibility Guidelines</li>
                    </ul>
                    <Link className="button button--secondary" to="/docs/modules/vla/">Learn More</Link>
                  </div>
                  
                  <div className="module-card">
                    <h2>5. CAPSTONE</h2>
                    <p>Integrating all modules into a complete humanoid robot project</p>
                    <ul>
                      <li>Capstone Project Introduction</li>
                      <li>Capstone Project Outline - AI-Powered Humanoid Robot</li>
                      <li>Comprehensive Integration Guide for Capstone Project</li>
                      <li>Capstone Project Exercises - Integrating All Modules</li>
                    </ul>
                    <Link className="button button--secondary" to="/docs/modules/capstone/">Learn More</Link>
                  </div>
                </div>
              </div>
            </div>
          </div>
        </section>

        {/* Learning Outcomes */}
        <section className={clsx(styles.section, styles.learningOutcomesSection)}>
          <div className="container padding-horiz--md">
            <div className="row">
              <div className="col col--8 col--offset-2">
                <h2 className="text--center">LEARNING OUTCOMES</h2>
                <p>After completing this course, students will be able to:</p>
                <ul>
                  <li>Master ROS 2 concepts and implementation for humanoid robotics</li>
                  <li>Simulate robots with both Gazebo and Unity for comprehensive testing</li>
                  <li>Develop AI-powered perception and navigation with NVIDIA Isaac</li>
                  <li>Integrate Vision-Language-Action capabilities for natural robot interaction</li>
                  <li>Integrate GPT models for cognitive planning and decision-making</li>
                  <li>Build conversational robotic systems using voice commands</li>
                  <li>Validate robot behaviors in both simulation and real-world environments</li>
                </ul>
              </div>
            </div>
          </div>
        </section>

        {/* Weekly Breakdown */}
        <section className={clsx(styles.section, styles.weeklyBreakdownSection)}>
          <div className="container padding-horiz--md">
            <div className="row">
              <div className="col col--8 col--offset-2">
                <h2 className="text--center">WEEKLY BREAKDOWN</h2>
                
                <details className="padding-vert--md">
                  <summary><strong>Weeks 1–2: Foundations</strong></summary>
                  <p>Introduction to robotics concepts, setup, and foundational tools</p>
                </details>
                
                <details className="padding-vert--md">
                  <summary><strong>Weeks 3–5: ROS 2 Fundamentals</strong></summary>
                  <p>Core ROS 2 concepts, nodes, topics, and services</p>
                </details>
                
                <details className="padding-vert--md">
                  <summary><strong>Weeks 6–7: Digital Twin Simulation</strong></summary>
                  <p>Gazebo and Unity simulation environment setup</p>
                </details>
                
                <details className="padding-vert--md">
                  <summary><strong>Weeks 8–10: NVIDIA Isaac</strong></summary>
                  <p>AI perception, navigation, and Isaac tools</p>
                </details>
                
                <details className="padding-vert--md">
                  <summary><strong>Weeks 11–12: VLA Integration</strong></summary>
                  <p>Vision-Language-Action systems and integration</p>
                </details>
                
                <details className="padding-vert--md">
                  <summary><strong>Week 13: Capstone Integration</strong></summary>
                  <p>Integrating all modules and final project completion</p>
                </details>
              </div>
            </div>
          </div>
        </section>

        {/* Assessments */}
        <section className={clsx(styles.section, styles.assessmentSection)}>
          <div className="container padding-horiz--md">
            <div className="row">
              <div className="col col--8 col--offset-2">
                <h2 className="text--center">ASSESSMENTS</h2>
                
                <div className="row padding-vert--md">
                  <div className="col col--6">
                    <h3>Module-Specific Assessments</h3>
                    <ul>
                      <li><Link to="/docs/modules/ros2/exercises/">ROS 2 Exercises</Link></li>
                      <li><Link to="/docs/modules/digital-twin/exercises/">Digital Twin Simulation Exercises</Link></li>
                      <li><Link to="/docs/modules/nvidia-isaac/exercises/">NVIDIA Isaac Exercises</Link></li>
                      <li><Link to="/docs/modules/vla/exercises/">VLA Integration Exercises</Link></li>
                    </ul>
                  </div>
                  <div className="col col--6">
                    <h3>Capstone Project Assessment</h3>
                    <ul>
                      <li><Link to="/docs/modules/capstone/project-outline">Comprehensive Capstone Project</Link></li>
                      <li>Performance evaluation and validation</li>
                      <li>Presentation and demonstration</li>
                    </ul>
                  </div>
                </div>
              </div>
            </div>
          </div>
        </section>

        {/* Hardware Requirements */}
        <section className={clsx(styles.section, styles.hardwareRequirementsSection)}>
          <div className="container padding-horiz--md">
            <div className="row">
              <div className="col col--8 col--offset-2">
                <h2 className="text--center">HARDWARE REQUIREMENTS & LAB SETUP</h2>
                
                <div className="row padding-vert--lg">
                  <div className="col col--4">
                    <h3>1. Digital Twin Workstation (Required)</h3>
                    <ul>
                      <li>RTX GPU (4070 Ti minimum)</li>
                      <li>Ubuntu 22.04</li>
                      <li>64GB RAM</li>
                    </ul>
                  </div>
                  
                  <div className="col col--4">
                    <h3>2. Physical AI Edge Kit</h3>
                    <ul>
                      <li>Jetson Orin Nano / NX</li>
                      <li>RealSense D435i</li>
                      <li>IMU, Microphone array</li>
                    </ul>
                  </div>
                  
                  <div className="col col--4">
                    <h3>3. Alternative Cloud Setup (High OpEx)</h3>
                    <ul>
                      <li>AWS g5 / g6 instances</li>
                      <li>Cloud-based Isaac Sim</li>
                      <li>Local Jetson for deployment</li>
                    </ul>
                  </div>
                </div>
              </div>
            </div>
          </div>
        </section>

        {/* Call to Action */}
        <section className={clsx('hero hero--dark', styles.ctaSection)}>
          <div className="container text--center">
            <h2>Ready to Start Building Physical AI?</h2>
            <p>Begin your journey into humanoid robotics and embodied intelligence</p>
            <div className={styles.buttons}>
              <Link
                className="button button--primary button--lg margin-horiz--md"
                to="/docs/modules/ros2/introduction">
                Begin Your Journey
              </Link>
              <Link
                className="button button--secondary button--lg margin-horiz--md"
                to="/docs/modules">
                Explore All Modules
              </Link>
              <Link
                className="button button--outline button--lg margin-horiz--md"
                to="/docs/exercises">
                View Exercises
              </Link>
            </div>
          </div>
        </section>
      </main>
    </Layout>
  );
}