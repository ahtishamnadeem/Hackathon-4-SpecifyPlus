import React from 'react';
import TranslationToggle from '@site/src/components/TranslationToggle/TranslationToggle';

const MDXContentWrapper = ({ children }) => {
  // Generate a unique ID based on the current route for caching
  const location = typeof window !== 'undefined' ? window.location : null;
  const contentId = location ? `mdx-${location.pathname.replace(/[\/:]/g, '-')}` : undefined;

  return (
    <TranslationToggle contentId={contentId}>
      {children}
    </TranslationToggle>
  );
};

export default MDXContentWrapper;