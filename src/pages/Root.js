/**
 * Root Component
 *
 * Global root component for the Docusaurus application.
 * This component wraps the entire application and is perfect
 * for adding global components like the floating chatbot.
 */

import React from 'react';
import FloatingChatbot from '../components/FloatingChatbot';

function Root({ children }) {
  return (
    <>
      {children}
      <FloatingChatbot />
    </>
  );
}

export default Root;