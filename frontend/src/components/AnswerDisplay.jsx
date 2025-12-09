import React from 'react';
import './AnswerDisplay.css';

const AnswerDisplay = ({ answer, sources, confidence, onCopy }) => {
  return (
    <div className="answer-display">
      <div className="answer-content">
        <p>{answer}</p>
        {sources && sources.length > 0 && (
          <div className="sources-section">
            <h4>References:</h4>
            <ul>
              {sources.map((source, index) => (
                <li key={index}>
                  {source.title || `Source ${index + 1}`}
                  {source.module && <span className="module-tag">({source.module})</span>}
                  {source.section && <span className="section-tag"> - {source.section}</span>}
                  {source.relevance_score && (
                    <span className="relevance-score"> (Relevance: {(source.relevance_score * 100).toFixed(1)}%)</span>
                  )}
                </li>
              ))}
            </ul>
          </div>
        )}
        {confidence !== undefined && (
          <div className="confidence-section">
            <span className={`confidence-score ${confidence > 0.7 ? 'high' : confidence > 0.4 ? 'medium' : 'low'}`}>
              Confidence: {(confidence * 100).toFixed(1)}%
            </span>
          </div>
        )}
      </div>
      {onCopy && (
        <button className="copy-button" onClick={() => onCopy(answer)}>
          Copy Answer
        </button>
      )}
    </div>
  );
};

export default AnswerDisplay;