/**
 * Error Display Component
 *
 * Displays error messages to the user
 */

import React from 'react';

const ErrorDisplay = ({ message, onRetry = null }) => {
  return (
    <div className="error-display">
      <span className="error-icon">⚠️</span>
      <span>{message}</span>
      {onRetry && (
        <button onClick={onRetry} className="retry-button">
          Retry
        </button>
      )}

      <style jsx>{`
        .error-display {
          background-color: #ffebee;
          color: #c62828;
          padding: 12px 16px;
          border-radius: 4px;
          margin: 8px 16px;
          border: 1px solid #ffcdd2;
          display: flex;
          align-items: center;
          gap: 8px;
          font-size: 0.9em;
        }

        .error-icon {
          font-weight: bold;
          font-size: 1.2em;
        }

        .retry-button {
          background-color: #c62828;
          color: white;
          border: none;
          padding: 4px 8px;
          border-radius: 4px;
          cursor: pointer;
          font-size: 0.8em;
          margin-left: auto;
        }

        .retry-button:hover {
          background-color: #b71c1c;
        }
      `}</style>
    </div>
  );
};

export default ErrorDisplay;