import React from 'react';
import clsx from 'clsx';
import styles from './LearningObjectives.module.css';

interface LearningObjectivesProps {
  objectives: string[];
}

const LearningObjectives: React.FC<LearningObjectivesProps> = ({ objectives }) => {
  return (
    <div className={styles.learningObjectives}>
      <h3>Learning Objectives</h3>
      <ul className={styles.objectivesList}>
        {objectives.map((objective, index) => (
          <li key={index} className={styles.objectiveItem}>
            {objective}
          </li>
        ))}
      </ul>
    </div>
  );
};

export default LearningObjectives;