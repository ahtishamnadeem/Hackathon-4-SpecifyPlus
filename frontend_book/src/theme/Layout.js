import React from 'react';
import OriginalLayout from '@theme-original/Layout';
import {useLocation} from '@docusaurus/router';
import FloatingChatbot from '../components/FloatingChatbot';

// No wrapper needed here — FloatingChatbot can use hooks safely
export default function Layout(props) {
  const {pathname} = useLocation();

  // Unique ID for main content (skip link)
  const mainContentId = `main-content-${pathname.replace(/\//g, '-') || 'home'}`;

  return (
    <>
      {/* Skip to main content link for keyboard users */}
      <a href={`#${mainContentId}`} className="skip-to-content">
        Skip to main content
      </a>

      <OriginalLayout {...props}>
        <main id={mainContentId}>
          {props.children}
        </main>

        {/* Floating Chatbot inside OriginalLayout to ensure ColorModeProvider */}
        <FloatingChatbot />
      </OriginalLayout>
    </>
  );
}
