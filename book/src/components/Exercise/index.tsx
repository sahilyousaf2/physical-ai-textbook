import React, { ReactNode } from 'react';
import clsx from 'clsx';
import styles from './styles.module.css';

interface Props {
  children: ReactNode;
  title: string;
  difficulty?: 'beginner' | 'intermediate' | 'advanced';
  estimatedTime?: number; // in minutes
  type?: 'practical' | 'theoretical' | 'coding' | 'analysis';
}

const Exercise = ({ children, title, difficulty = 'beginner', estimatedTime, type = 'practical' }: Props): JSX.Element => {
  const getDifficultyColor = () => {
    switch (difficulty) {
      case 'beginner':
        return 'beginner';
      case 'intermediate':
        return 'intermediate';
      case 'advanced':
        return 'advanced';
      default:
        return 'beginner';
    }
  };

  const getDifficultyLabel = () => {
    switch (difficulty) {
      case 'beginner':
        return 'Beginner';
      case 'intermediate':
        return 'Intermediate';
      case 'advanced':
        return 'Advanced';
      default:
        return 'Beginner';
    }
  };

  const getTypeLabel = () => {
    switch (type) {
      case 'practical':
        return 'Practical Exercise';
      case 'theoretical':
        return 'Theoretical Exercise';
      case 'coding':
        return 'Coding Exercise';
      case 'analysis':
        return 'Analysis Exercise';
      default:
        return 'Exercise';
    }
  };

  return (
    <div className={clsx('practice-exercise', styles.exerciseContainer)}>
      <div className="practice-exercise-title">
        <span className={clsx(styles.exerciseIcon, 'practice-exercise-icon')}>📝</span>
        <span className={styles.exerciseTitle}>
          {getTypeLabel()}: {title}
        </span>
        <span className={clsx(styles.difficultyBadge, styles[getDifficultyColor()])}>
          {getDifficultyLabel()}
        </span>
        {estimatedTime && (
          <span className={styles.timeBadge}>
            ~{estimatedTime} min
          </span>
        )}
      </div>
      <div className={styles.exerciseContent}>
        {children}
      </div>
    </div>
  );
};

export default Exercise;