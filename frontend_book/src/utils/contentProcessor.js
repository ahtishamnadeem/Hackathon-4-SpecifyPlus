/**
 * Content Processor for Translation
 * Handles extraction and reconstruction of content with markdown structure preservation
 */

// Function to extract text content while preserving structure information
export const extractTextWithStructure = (node, path = []) => {
  if (typeof node === 'string' || typeof node === 'number') {
    return {
      type: 'text',
      content: String(node),
      path: [...path]
    };
  }

  if (Array.isArray(node)) {
    return node.map((child, index) =>
      extractTextWithStructure(child, [...path, index])
    ).flat();
  }

  if (node && typeof node === 'object' && node.$$typeof) {
    // React element
    const { type, props } = node;
    const elementPath = [...path, type];

    if (props && props.children) {
      const childrenData = extractTextWithStructure(props.children, elementPath);
      return [{
        type: 'element',
        tag: type,
        props: { ...props, children: undefined }, // Don't include children in structure data
        path: elementPath,
        children: Array.isArray(childrenData) ? childrenData : [childrenData]
      }, ...childrenData.flat ? childrenData.flat() : childrenData];
    } else {
      return [{
        type: 'element',
        tag: type,
        props: { ...props },
        path: elementPath
      }];
    }
  }

  return [];
};

// Function to reconstruct content from translation while preserving structure
export const reconstructContentWithTranslation = (originalStructure, translatedText) => {
  // For now, we'll return the translated text directly
  // In a more advanced implementation, we would map the translated text back to the structure
  return translatedText;
};

// Function to convert markdown content to a format suitable for translation
export const prepareContentForTranslation = (content) => {
  // Convert React elements to plain text while keeping structure information
  if (typeof content === 'string') {
    return content;
  }

  // For React elements, extract the text content
  const structure = extractTextWithStructure(content);
  const textContent = structure
    .filter(item => item.type === 'text')
    .map(item => item.content)
    .join(' ');

  return textContent;
};

// Function to post-process translated content to ensure proper formatting
export const postProcessTranslation = (translatedText) => {
  // Clean up any obvious translation artifacts
  let cleaned = translatedText.replace(/\n\s*\n\s*\n/g, '\n\n'); // Remove excessive newlines
  cleaned = cleaned.replace(/\s+$/, ''); // Remove trailing whitespace
  cleaned = cleaned.replace(/^\s+/, ''); // Remove leading whitespace

  return cleaned;
};