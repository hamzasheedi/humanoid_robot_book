import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
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
      <header className={clsx('hero hero--primary', styles.heroBanner)}>
        <div className="container">
          <h1 className="hero__title">Physical AI & Humanoid Robotics</h1>
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
        {/* Book Overview Section */}
        <section className={styles.section}>
          <div className="container padding-horiz--md">
            <div className="row">
              <div className="col col--8 col--offset-2">
                <h2 className="text--center padding-bottom--md">Quarter Overview</h2>
                <p>
                  The future of AI extends beyond digital spaces into real-world interaction.
                  This capstone quarter introduces <strong>Physical AI</strong> — AI systems that understand physics,
                  interact with the world, and control humanoid robots using:
                </p>

                <div className="row padding-vert--md">
                  <div className="col col--4">
                    <ul className={styles.featureList}>
                      <li>ROS 2</li>
                    </ul>
                  </div>
                  <div className="col col--4">
                    <ul className={styles.featureList}>
                      <li>Gazebo</li>
                    </ul>
                  </div>
                  <div className="col col--4">
                    <ul className={styles.featureList}>
                      <li>Unity</li>
                    </ul>
                  </div>
                </div>
                <div className="row">
                  <div className="col col--4 col--offset-4">
                    <ul className={styles.featureList}>
                      <li>NVIDIA Isaac</li>
                    </ul>
                  </div>
                </div>

                <p>
                  Students will simulate, design, and deploy humanoid robots capable of natural interaction.
                </p>
              </div>
            </div>
          </div>
        </section>

        {/* Modules Section */}
        <section className={clsx(styles.section, styles.modulesSection)}>
          <div className="container padding-horiz--md">
            <div className="row">
              <div className="col col--10 col--offset-1">
                <h2 className="text--center padding-bottom--md">Core Modules</h2>

                <div className="row">
                  <ModuleCard
                    title="Module 1: Robotic Nervous System (ROS 2)"
                    description="Nodes, Topics, Services; rclpy Python bridges; URDF for humanoids"
                    link="/docs/modules/ros2/introduction"
                  />

                  <ModuleCard
                    title="Module 2: Digital Twin Simulation (Gazebo & Unity)"
                    description="Physics simulation; High-fidelity rendering; Sensor simulation (LiDAR, Depth, IMUs)"
                    link="/docs/modules/digital-twin/introduction"
                  />

                  <ModuleCard
                    title="Module 3: AI-Robot Brain (NVIDIA Isaac)"
                    description="Isaac Sim & synthetic data; Isaac ROS & VSLAM; Nav2 path planning"
                    link="/docs/modules/nvidia-isaac/introduction"
                  />
                </div>

                <div className="row">
                  <ModuleCard
                    title="Module 4: Vision-Language-Action (VLA)"
                    description="Whisper for voice commands; LLM-powered action planning; Final Capstone: Autonomous Humanoid Robot"
                    link="/docs/modules/vla/introduction"
                  />

                  <ModuleCard
                    title="Module 5: Capstone Project"
                    description="Integration of all modules in a complete humanoid robot project"
                    link="/docs/modules/capstone/introduction"
                  />

                  <ModuleCard
                    title="Learning Resources"
                    description="Exercises, assessments, and external resources"
                    link="/docs/exercises"
                  />
                </div>
              </div>
            </div>
          </div>
        </section>

        {/* Learning Outcomes */}
        <section className={styles.section}>
          <div className="container padding-horiz--md">
            <div className="row">
              <div className="col col--8 col--offset-2">
                <h2 className="text--center">🎯 Learning Outcomes</h2>

                <div className="row padding-vert--md">
                  <div className="col col--6">
                    <ul>
                      <li>Understand Physical AI principles</li>
                      <li>Master ROS 2 concepts and implementation</li>
                      <li>Build and validate digital twin simulations</li>
                      <li>Design humanoid robot behaviors</li>
                    </ul>
                  </div>
                  <div className="col col--6">
                    <ul>
                      <li>Integrate GPT models for cognitive planning</li>
                      <li>Build conversational robotic systems</li>
                      <li>Validate systems in simulation and real-world</li>
                      <li>Develop end-to-end robot capabilities</li>
                    </ul>
                  </div>
                </div>
              </div>
            </div>
          </div>
        </section>

        {/* Hardware Requirements */}
        <section className={styles.section}>
          <div className="container padding-horiz--md">
            <div className="row">
              <div className="col col--8 col--offset-2">
                <h2 className="text--center">💻 Hardware Requirements</h2>

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

        {/* Weekly Breakdown (expandable) */}
        <section className={styles.section}>
          <div className="container padding-horiz--md">
            <div className="row">
              <div className="col col--8 col--offset-2">
                <h2 className="text--center">🗓 Weekly Breakdown</h2>

                <details className="padding-vert--md">
                  <summary>Weeks 1–2: Foundations</summary>
                  <p>Introduction to robotics concepts, setup, and foundational tools</p>
                </details>

                <details className="padding-vert--md">
                  <summary>Weeks 3–5: ROS 2 Fundamentals</summary>
                  <p>Core ROS 2 concepts, nodes, topics, and services</p>
                </details>

                <details className="padding-vert--md">
                  <summary>Weeks 6–7: Gazebo Simulation</summary>
                  <p>Digital twin creation and simulation techniques</p>
                </details>

                <details className="padding-vert--md">
                  <summary>Weeks 8–10: NVIDIA Isaac</summary>
                  <p>AI perception, navigation, and Isaac tools</p>
                </details>

                <details className="padding-vert--md">
                  <summary>Weeks 11–12: Humanoid Robot Development</summary>
                  <p>Integration of modules and robot development</p>
                </details>

                <details className="padding-vert--md">
                  <summary>Week 13: Conversational Robotics</summary>
                  <p>Vision-Language-Action integration and capstone completion</p>
                </details>
              </div>
            </div>
          </div>
        </section>

        {/* Assessments */}
        <section className={styles.section}>
          <div className="container padding-horiz--md">
            <div className="row">
              <div className="col col--8 col--offset-2">
                <h2 className="text--center">🧪 Assessments</h2>

                <div className="row padding-vert--md">
                  <div className="col col--6">
                    <h3>Module-Specific</h3>
                    <ul>
                      <li>ROS package development project</li>
                      <li>Gazebo simulation environment creation</li>
                      <li>Isaac perception pipeline implementation</li>
                    </ul>
                  </div>
                  <div className="col col--6">
                    <h3>Capstone Project</h3>
                    <ul>
                      <li>Complete humanoid robot with VLA integration</li>
                      <li>Performance evaluation and validation</li>
                      <li>Presentation and demonstration</li>
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