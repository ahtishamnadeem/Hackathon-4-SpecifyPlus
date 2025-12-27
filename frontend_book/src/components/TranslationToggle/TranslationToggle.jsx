import React, { useState, useEffect, useMemo } from 'react';
import { useColorMode } from '@docusaurus/theme-common';
import translationApi from '@site/src/utils/translationApi';
import { prepareContentForTranslation, postProcessTranslation } from '@site/src/utils/contentProcessor';
import styles from './TranslationToggle.module.css';

const TranslationToggle = ({ children, contentId }) => {
  const [isUrdu, setIsUrdu] = useState(false);
  const [translatedContent, setTranslatedContent] = useState(null);
  const [isTranslating, setIsTranslating] = useState(false);
  const { colorMode } = useColorMode();

  // Generate contentId if not provided
  const generatedContentId = useMemo(() => {
    if (contentId) return contentId;
    return `translation-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`;
  }, [contentId]);

  // Check for cached translation
  useEffect(() => {
    const cached = localStorage.getItem(`urdu-translation-${generatedContentId}`);
    if (cached) {
      try {
        const parsed = JSON.parse(cached);
        setTranslatedContent(parsed);
      } catch (e) {
        console.error('Failed to parse cached translation:', e);
      }
    }
  }, [generatedContentId]);

  const translateToUrdu = async () => {
    if (isUrdu) {
      setIsUrdu(false);
      return;
    }

    // Check cache first
    if (translatedContent) {
      setIsUrdu(true);
      return;
    }

    setIsTranslating(true);

    try {
      // Prepare content for translation
      const englishText = prepareContentForTranslation(children);

      // Call the translation API
      const rawTranslatedText = await translationApi.translateToUrdu(englishText, 'educational');

      // Post-process the translation
      const translatedText = postProcessTranslation(rawTranslatedText);

      const translationData = {
        original: children,
        translated: translatedText,
        timestamp: Date.now()
      };

      setTranslatedContent(translationData);

      // Cache in localStorage
      localStorage.setItem(`urdu-translation-${generatedContentId}`, JSON.stringify(translationData));

      setIsUrdu(true);
    } catch (error) {
      console.error('Translation failed:', error);
      alert(`Translation failed: ${error.message || 'Please try again later'}`);
    } finally {
      setIsTranslating(false);
    }
  };

  // Get the current content to display
  const currentContent = isUrdu && translatedContent ? translatedContent.translated : children;

  return (
    <div className={styles.translationContainer}>
      <div className={styles.translationHeader}>
        <button
          className={`${styles.translationToggle} ${
            isUrdu ? styles.urduActive : styles.englishActive
          } ${isTranslating ? styles.translating : ''} ${
            colorMode === 'dark' ? styles.darkMode : ''
          }`}
          onClick={translateToUrdu}
          disabled={isTranslating}
          title={isUrdu ? 'Switch to English' : 'Translate to Urdu'}
        >
          <span className={styles.toggleIcon}>
            {isTranslating ? (
              <span className={styles.spinner}>🔄</span>
            ) : isUrdu ? (
              <span className={styles.flag}>🇺🇸</span>
            ) : (
              <span className={styles.flag}>🇵🇰</span>
            )}
          </span>
          <span className={styles.toggleText}>
            {isTranslating ? 'Translating...' : isUrdu ? 'English' : 'اردو میں ترجمہ کریں'}
          </span>
        </button>
      </div>

      <div
        className={`${styles.translatedContent} ${isUrdu ? styles.urduContent : styles.englishContent}`}
        dir={isUrdu ? 'rtl' : 'ltr'}
      >
        {typeof currentContent === 'string' ? (
          <div dangerouslySetInnerHTML={{ __html: currentContent }} />
        ) : (
          currentContent
        )}
      </div>
    </div>
  );
};

export default TranslationToggle;