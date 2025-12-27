---
title: Urdu Translation Feature
sidebar_position: 1
---

# Urdu Translation Feature

This page demonstrates how to use the Urdu translation feature in your documentation.

## How to Use

To add the translation toggle to your documentation page, simply import and use the `TranslationToggle` component:

```jsx
import TranslationToggle from '@site/src/components/TranslationToggle/TranslationToggle';

<TranslationToggle contentId="unique-content-id">
  {/* Your content here */}
</TranslationToggle>
```

## Example

Here's an example of how to wrap your content with the translation toggle:

```jsx
import TranslationToggle from '@site/src/components/TranslationToggle/TranslationToggle';

export default function ExamplePage() {
  const content = (
    <div>
      <h1>Chapter Title</h1>
      <p>This is sample content that can be translated to Urdu.</p>
      <h2>Technical Concepts</h2>
      <p>ROS 2 (Robot Operating System 2) is a flexible framework for writing robot software.</p>
    </div>
  );

  return (
    <TranslationToggle contentId="example-chapter-1">
      {content}
    </TranslationToggle>
  );
}
```

## Features

- **Toggle Button**: Switch between English and Urdu content
- **Caching**: Translated content is cached locally to avoid repeated API calls
- **RTL Support**: Proper right-to-left layout for Urdu text
- **Responsive**: Works on all device sizes
- **Preserved Formatting**: Maintains markdown structure and formatting