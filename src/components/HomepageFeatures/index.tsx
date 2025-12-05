import type {ReactNode} from 'react';
import clsx from 'clsx';
import Heading from '@theme/Heading';
import { FaRobot, FaBrain, FaLanguage, FaGamepad, FaMapMarkedAlt, FaTools, FaEye, FaBalanceScale, FaComments } from 'react-icons/fa';
import styles from './styles.module.css';

type FeatureItem = {
  title: string;
  Icon: React.ComponentType<React.ComponentProps<'svg'>>;
  description: ReactNode;
};

const FeatureList: FeatureItem[] = [
  {
    title: 'AI-Powered Learning',
    Icon: FaBrain,
    description: (
      <>
        Interactive RAG chatbot helps you understand complex concepts with 
        contextual answers from the textbook content.
      </>
    ),
  },
  {
    title: 'Hands-On Robotics',
    Icon: FaRobot,
    description: (
      <>
        Master ROS 2, URDF, Gazebo simulation, and real-world humanoid robotics 
        through practical examples and projects.
      </>
    ),
  },
  {
    title: 'Personalized Experience',
    Icon: FaLanguage,
    description: (
      <>
        Content adapts to your skill level (beginner/intermediate/expert) with 
        Urdu translation support for accessibility.
      </>
    ),
  },
  {
    title: 'Advanced Simulations',
    Icon: FaGamepad,
    description: (
      <>
        Learn with NVIDIA Isaac Sim, Unity visualization, and Gazebo for 
        photorealistic robotics testing and development.
      </>
    ),
  },
  {
    title: 'VSLAM & Navigation',
    Icon: FaMapMarkedAlt,
    description: (
      <>
        Master autonomous movement, visual SLAM algorithms, and 
        navigation systems for humanoid robots.
      </>
    ),
  },
  {
    title: 'Capstone Project',
    Icon: FaTools,
    description: (
      <>
        Build a complete autonomous humanoid robot from scratch 
        applying all learned concepts and skills.
      </>
    ),
  },
  {
    title: 'Vision-Language Models',
    Icon: FaEye,
    description: (
      <>
        Explore cutting-edge VLA models that combine computer vision, 
        natural language, and robotic action.
      </>
    ),
  },
  {
    title: 'Control & Balance',
    Icon: FaBalanceScale,
    description: (
      <>
        Learn bipedal locomotion, balance control algorithms, and 
        dynamic stability for humanoid robots.
      </>
    ),
  },
  {
    title: 'Conversational Robotics',
    Icon: FaComments,
    description: (
      <>
        Enable natural language interaction with robots using 
        advanced dialogue systems and understanding.
      </>
    ),
  },
];

function Feature({title, Icon, description}: FeatureItem) {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center">
        <Icon className={styles.featureSvg} role="img" style={{ fontSize: '32px' }} />
      </div>
      <div className="text--center padding-horiz--md">
        <Heading as="h3">{title}</Heading>
        <p>{description}</p>
      </div>
    </div>
  );
}

export default function HomepageFeatures(): ReactNode {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          {FeatureList.slice(0, 3).map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
        <div className="row">
          {FeatureList.slice(3, 6).map((props, idx) => (
            <Feature key={idx + 3} {...props} />
          ))}
        </div>
        <div className="row">
          {FeatureList.slice(6, 9).map((props, idx) => (
            <Feature key={idx + 6} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}
