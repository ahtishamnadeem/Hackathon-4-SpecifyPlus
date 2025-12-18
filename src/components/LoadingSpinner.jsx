/**
 * Loading Spinner Component
 *
 * Displays a loading indicator during API requests
 */

import React from 'react';

const LoadingSpinner = ({ message = 'Processing your request...' }) => {
  return (
    <div className="loading-spinner">
      <div className="spinner"></div>
      <span>{message}</span>

      <style jsx>{`
        .loading-spinner {
          display: flex;
          justify-content: center;
          align-items: center;
          padding: 16px;
          color: #666;
          gap: 8px;
        }

        .spinner {
          border: 3px solid #f3f3f3;
          border-top: 3px solid #007bff;
          border-radius: 50%;
          width: 20px;
          height: 20px;
          animation: spin 1s linear infinite;
        }

        @keyframes spin {
          0% { transform: rotate(0deg); }
          100% { transform: rotate(360deg); }
        }
      `}</style>
    </div>
  );
};

export default LoadingSpinner;