import type {ReactNode} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';

import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <Heading as="h1" className="hero__title">
          {siteConfig.title}
        </Heading>
        <p className="hero__subtitle">{siteConfig.tagline}</p>
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="/docs/introduction">
            Start Reading ⏱️
          </Link>
        </div>
      </div>
    </header>
  );
}

type ModuleItem = {
  title: string;
  description: JSX.Element;
  to: string;
};

const ModuleList: ModuleItem[] = [
  {
    title: 'Module 1: The Robotic Nervous System (ROS 2)',
    to: '/docs/module1/chapter1',
    description: (
      <>
        Master the foundational framework for robot development, from basic concepts to advanced system integration.
      </>
    ),
  },
  {
    title: 'Module 2: The Digital Twin (Gazebo & Unity)',
    to: '/docs/module2/chapter1',
    description: (
      <>
        Learn to simulate complex robotic systems in virtual environments, crucial for design and testing.
      </>
    ),
  },
  {
    title: 'Module 3: The AI-Robot Brain (NVIDIA Isaac)',
    to: '/docs/module3/chapter1',
    description: (
      <>
        Dive into AI-driven perception and decision-making with NVIDIA's powerful platform for robotics.
      </>
    ),
  },
  {
    title: 'Module 4: Vision-Language-Action (VLA)',
    to: '/docs/module4/chapter1',
    description: (
      <>
        Understand how to integrate computer vision and natural language processing for advanced robotic control.
      </>
    ),
  },
];

function Module({title, description, to}: ModuleItem) {
  return (
    <div className={clsx('col col--6 margin-bottom--lg')}>
      <Link className={styles.moduleCard} to={to}>
        <div className="text--center padding-horiz--md">
          <Heading as="h3">{title}</Heading>
          <p>{description}</p>
        </div>
      </Link>
    </div>
  );
}

function HomepageMainContent() {
  return (
    <section className={styles.modules}>
      <div className="container">
        <Heading as="h2" className="text--center margin-bottom--lg">Explore the World of Robotics</Heading>
        <div className="row">
          {ModuleList.map((props, idx) => (
            <Module key={idx} {...props} />
          ))}
        </div>
        <div className="row">
          <div className={clsx('col col--12 text--center')}>
            <p>Whether you're a student, hobbyist, or professional, this resource will equip you with the knowledge and tools to design, build, and program the next generation of intelligent robots.</p>
          </div>
        </div>
      </div>
    </section>
  );
}

export default function Home(): ReactNode {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Home`}
      description="A comprehensive guide to modern robotics development focusing on Physical AI and Humanoid Robotics.">
      <HomepageHeader />
      <main>
        <HomepageMainContent />
      </main>
    </Layout>
  );
}