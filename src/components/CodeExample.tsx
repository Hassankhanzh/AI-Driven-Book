import React from 'react';
import clsx from 'clsx';
import styles from './CodeExample.module.css';

type CodeExampleProps = {
  title: string;
  description: string;
  code: string;
  language?: string;
};

export default function CodeExample({
  title,
  description,
  code,
  language = 'python',
}: CodeExampleProps): JSX.Element {
  return (
    <div className={styles.codeExample}>
      <div className={styles.header}>
        <h3>{title}</h3>
      </div>
      <div className={styles.content}>
        <p>{description}</p>
        <div className={styles.codeBlock}>
          <pre>
            <code className={`language-${language}`}>{code}</code>
          </pre>
        </div>
      </div>
    </div>
  );
}