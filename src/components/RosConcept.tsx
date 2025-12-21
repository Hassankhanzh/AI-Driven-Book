import React from 'react';
import clsx from 'clsx';
import styles from './RosConcept.module.css';

type RosConceptProps = {
  title: string;
  type: 'node' | 'topic' | 'service' | 'action' | 'parameter';
  description: string;
  usage: string;
};

export default function RosConcept({
  title,
  type,
  description,
  usage,
}: RosConceptProps): JSX.Element {
  const typeLabels = {
    node: { label: 'Node', color: 'var(--ifm-color-primary)' },
    topic: { label: 'Topic', color: '#28a745' },
    service: { label: 'Service', color: '#ffc107' },
    action: { label: 'Action', color: '#17a2b8' },
    parameter: { label: 'Parameter', color: '#6f42c1' },
  };

  const typeInfo = typeLabels[type];

  return (
    <div className={styles.rosConcept}>
      <div className={styles.header} style={{ borderBottom: `2px solid ${typeInfo.color}` }}>
        <span className={clsx(styles.badge, styles[type])}>{typeInfo.label}</span>
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
}