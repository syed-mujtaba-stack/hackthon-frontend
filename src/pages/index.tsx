import type {ReactNode, CSSProperties} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageFeatures from '@site/src/components/HomepageFeatures';
import LearningPath from '@site/src/components/LearningPath';
import Heading from '@theme/Heading';

import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  const heroStyle: CSSProperties = {
    backgroundImage: 'url(https://copilot.microsoft.com/th/id/BCO.77a0bd4e-bad6-46a0-9ef8-887370dbde8d.png)',
    backgroundSize: '100% 100%',
    backgroundPosition: 'center',
    backgroundRepeat: 'no-repeat',
    position: 'relative',
    minHeight: '600px',
    width: '100%',
    display: 'flex',
    alignItems: 'center'
  };
  
  const overlayStyle: CSSProperties = {
    position: 'absolute',
    top: 0,
    left: 0,
    right: 0,
    bottom: 0,
    background: 'rgba(0, 0, 0, 0.5)',
    zIndex: 1
  };
  
  const containerStyle: CSSProperties = {
    position: 'relative',
    zIndex: 2
  };
  
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)} style={heroStyle}>
      <div style={overlayStyle}></div>
      <div className="container" style={containerStyle}>
        <Heading as="h1" className="hero__title" style={{color: 'white', textShadow: '2px 2px 4px rgba(0, 0, 0, 0.8)'}}>
          {siteConfig.title}
        </Heading>
        <p className="hero__subtitle" style={{color: 'white', textShadow: '2px 2px 4px rgba(0, 0, 0, 0.8)'}}>{siteConfig.tagline}</p>
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="/docs/intro">
            Start Learning Physical AI 🤖
          </Link>
        </div>
      </div>
    </header>
  );
}

export default function Home(): ReactNode {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Hello from ${siteConfig.title}`}
      description="Description will go into a meta tag in <head />">
      <HomepageHeader />
      <main>
        <HomepageFeatures />
        <LearningPath />
      </main>
    </Layout>
  );
}
