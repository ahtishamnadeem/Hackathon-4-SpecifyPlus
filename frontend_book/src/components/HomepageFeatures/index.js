import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css';
import useOnScreen from '@site/src/utils/useOnScreen';

const FeatureList = [
  {
    title: 'Physical AI',
    imageUrl: '/img/card1.png',
    description: (
      <>
        Foundation of embodied intelligence. Master AI systems that
        perceive, interact, and learn from the physical world.
      </>
    ),
  },
  {
    title: 'Humanoid Robotics',
    imageUrl: '/img/card2.png',
    description: (
      <>
        Advanced locomotion & interaction. Build robots that move,
        think, and collaborate like humans.
      </>
    ),
  },
  {
    title: 'ROS 2 Ecosystem',
    imageUrl: '/img/card3.png',
    description: (
      <>
        Distributed robotics framework. Navigate, communicate, and
        orchestrate complex robotic systems.
      </>
    ),
  },
];

function Feature({imageUrl, title, description}) {
  const [ref, isOnScreen] = useOnScreen({ threshold: 0.1, rootMargin: '0px 0px -50px 0px' });

  return (
    <div
      ref={ref}
      className={clsx(
        'col col--4',
        styles.featureCard,
        isOnScreen ? styles.featureCardVisible : ''
      )}
    >
      <div className="text--center">
        <img src={imageUrl} alt={title} className={styles.featureImage} />
      </div>
      <div className="text--center padding-horiz--md">
        <Heading as="h3" className={styles.featureTitle}>{title}</Heading>
        <p className={styles.featureDescription}>{description}</p>
      </div>
    </div>
  );
}

export default function HomepageFeatures() {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}
