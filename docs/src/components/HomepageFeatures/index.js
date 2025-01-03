import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css';
import useBaseUrl from '@docusaurus/useBaseUrl';

const FeatureList = [
  {
    title: 'REXASIPRO project',
    image: '/img/rexasipro.png',
    description: (
      <>
        The REXASI-PRO project aims to release a novel engineering framework to develop greener and Trustworthy Artificial Intelligence solutions.
        The project will develop in parallel the design of novel trustworthy-by-construction solutions for social navigations
        and a methodology to certify the robustness of AI-based autonomous vehicles for people with reduced mobility.
        <a href="https://rexasi-pro.spindoxlabs.com/"> More</a>.
      </>
    ),
  },
];

function Feature({ image, title, description }) {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center">
        <img src={useBaseUrl(image)} />
      </div>
      <div className="text--center padding-horiz--md">
        <Heading as="h3">{title}</Heading>
        <p>{description}</p>
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
