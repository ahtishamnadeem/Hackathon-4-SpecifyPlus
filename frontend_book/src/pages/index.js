import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageFeatures from '@site/src/components/HomepageFeatures';

import Heading from '@theme/Heading';
import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <img
          src="/img/hero-banner-img.png"
          alt="Hero Background"
          className={styles.heroBackgroundImg}
          style={{
            position: 'absolute',
            top: 0,
            left: 0,
            width: '100%',
            height: '100%',
            objectFit: 'cover',
            zIndex: -2
          }}
        />
        {/* Dark mode overlay */}
        <div className={styles.heroBackgroundOverlay}></div>
        <div className={styles.heroContent}>
          <Heading as="h1" className={`hero__title ${styles.fadeInUp}`}>
            {siteConfig.title}
          </Heading>
          <p className={`hero__subtitle ${styles.fadeInUpDelay1}`}>Physical AI and Humanoid Robotics</p>
          <p className={`hero__tagline ${styles.fadeInUpDelay2}`}>{siteConfig.tagline}</p>
          <div className={styles.buttons}>
            <Link
              className="button button--primary button--lg"
              to="/docs/intro">
              Start Learning
            </Link>
            <Link
              className="button button--secondary button--lg"
              to="/chat">
              Try RAG Chatbot
            </Link>
          </div>
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
        <HomepageFeatures />
      </main>
    </Layout>
  );
}
