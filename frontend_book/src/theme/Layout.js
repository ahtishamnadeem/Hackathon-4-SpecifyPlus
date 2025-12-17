import React from 'react';
import OriginalLayout from '@theme-original/Layout';
import {useLocation} from '@docusaurus/router';

export default function Layout(props) {
  const {pathname} = useLocation();

  // Create a unique ID for the main content based on the pathname
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
      </OriginalLayout>
    </>
  );
}