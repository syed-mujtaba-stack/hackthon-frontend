import type {ReactNode} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import Heading from '@theme/Heading';
import { FaBook, FaRobot, FaGamepad, FaEye, FaComments, FaTools } from 'react-icons/fa';
import styles from './styles.module.css';

type ChapterItem = {
  number: string;
  title: string;
  description: string;
  link: string;
  Icon: React.ComponentType<React.ComponentProps<'svg'>>;
  difficulty: 'Beginner' | 'Intermediate' | 'Advanced';
};

const ChapterList: ChapterItem[] = [
  {
    number: '01',
    title: 'Introduction to Physical AI',
    description: 'Foundations of embodied intelligence and AI-robotics integration',
    link: '/docs/intro',
    Icon: FaBook,
    difficulty: 'Beginner'
  },
  {
    number: '02',
    title: 'ROS 2 Fundamentals',
    description: 'Master the robotic nervous system with ROS 2 Humble',
    link: '/docs/chapter-02-ros2-fundamentals',
    Icon: FaRobot,
    difficulty: 'Beginner'
  },
  {
    number: '03',
    title: 'URDF and Xacro',
    description: 'Robot description formats and modeling techniques',
    link: '/docs/chapter-03-urdf',
    Icon: FaTools,
    difficulty: 'Intermediate'
  },
  {
    number: '04',
    title: 'Gazebo Simulation',
    description: 'Testing robots in digital twin environments',
    link: '/docs/chapter-04-gazebo',
    Icon: FaGamepad,
    difficulty: 'Intermediate'
  },
  {
    number: '05',
    title: 'Unity Visualization',
    description: 'High-fidelity rendering and robot visualization',
    link: '/docs/chapter-05-unity',
    Icon: FaEye,
    difficulty: 'Intermediate'
  },
  {
    number: '06',
    title: 'NVIDIA Isaac Sim',
    description: 'Photorealistic simulation for advanced robotics',
    link: '/docs/chapter-06-isaac-sim',
    Icon: FaGamepad,
    difficulty: 'Advanced'
  }
];

function Chapter({ number, title, description, link, Icon, difficulty }: ChapterItem) {
  const difficultyColor = {
    Beginner: 'green',
    Intermediate: 'orange',
    Advanced: 'red'
  };

  return (
    <div className={clsx('col col--4', styles.chapterCard)}>
      <div className={styles.chapterHeader}>
        <span className={styles.chapterNumber}>{number}</span>
        <span className={clsx('badge', `badge--${difficultyColor[difficulty]}`)}>
          {difficulty}
        </span>
      </div>
      <div className={styles.chapterIcon}>
        <Icon style={{ fontSize: '24px' }} />
      </div>
      <Heading as="h3" className={styles.chapterTitle}>
        {title}
      </Heading>
      <p className={styles.chapterDescription}>{description}</p>
      <Link to={link} className="button button--primary button--sm">
        Start Chapter
      </Link>
    </div>
  );
}

export default function LearningPath(): ReactNode {
  return (
    <section className={styles.learningPath}>
      <div className="container">
        <div className="row">
          <div className="col col--12">
            <Heading as="h2" className="text--center">
              Your Learning Journey
            </Heading>
            <p className="text--center">
              Progress through carefully structured chapters from basics to advanced topics
            </p>
          </div>
        </div>
        <div className="row">
          {ChapterList.map((props, idx) => (
            <Chapter key={idx} {...props} />
          ))}
        </div>
        <div className="row">
          <div className="col col--12 text--center">
            <Link to="/docs/chapter-07-vslam" className="button button--secondary button--lg">
              View All Chapters →
            </Link>
          </div>
        </div>
      </div>
    </section>
  );
}
