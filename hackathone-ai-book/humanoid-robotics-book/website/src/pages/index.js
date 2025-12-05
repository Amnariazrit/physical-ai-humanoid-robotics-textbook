import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageFeatures from '@site/src/components/HomepageFeatures'; // Assuming you might add features later

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
            to="/docs/module1-ros2/chapter1">
            Start Reading 📖
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
      title={`Hello from ${siteConfig.title}`}
      description="Description will go into a meta tag in <head />">
      <HomepageHeader />
      <main>
        {/* <HomepageFeatures /> */} {/* Uncomment if you create HomepageFeatures component */}
        <section className={styles.heroContent}>
          <div className="container text--center">
            <h2>Welcome to the Physical AI & Humanoid Robotics Textbook!</h2>
            <p>
              This comprehensive guide is designed for students, educators, and enthusiasts eager to delve into the exciting world of humanoid robotics. Explore modules covering ROS 2, Gazebo, NVIDIA Isaac Sim, and Vision-Language-Action (VLA) models to build intelligent, autonomous robots.
            </p>
            <p>
              Get ready for hands-on learning with practical examples and simulations that bridge the gap between theory and real-world application.
            </p>
            {/* <img src="/img/humanoid-robot-hero.png" alt="Humanoid Robot" className={styles.heroImage} /> */}
          </div>
        </section>
      </main>
    </Layout>
  );
}