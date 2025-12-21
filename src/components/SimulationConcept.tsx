import React from 'react';
import clsx from 'clsx';
import styles from './SimulationConcept.module.css';

interface SimulationConceptProps {
  title: string;
  type: 'gazebo' | 'unity' | 'sensor' | 'physics';
  description: string;
  usage: string;
}

const SimulationConcept: React.FC<SimulationConceptProps> = ({
  title,
  type,
  description,
  usage,
}) => {
  return (
    <div className={styles.simulationConcept}>
      <div className={styles.header}>
        <span className={clsx(styles.badge, styles[type])}>
          {type.toUpperCase()}
        </span>
        <h3 className={styles.title}>{title}</h3>
      </div>
      <div className={styles.content}>
        <div className={styles.description}>
          <h4>Description</h4>
          <p>{description}</p>
        </div>
        <div className={styles.usage}>
          <h4>Usage</h4>
          <p>{usage}</p>
        </div>
      </div>
    </div>
  );
};

export default SimulationConcept;