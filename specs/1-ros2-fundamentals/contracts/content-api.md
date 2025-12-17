# Content API Contract: ROS 2 Fundamentals Book

## Overview
This document defines the content structure and metadata contracts for the ROS 2 Fundamentals educational book. Since this is a static Docusaurus site, these contracts ensure consistency in content formatting and metadata.

## Content Entity Structure

### Module Metadata Contract
```json
{
  "id": "string (unique identifier)",
  "title": "string (display title)",
  "description": "string (brief description)",
  "targetAudience": "string (specific audience)",
  "prerequisites": "array of strings",
  "learningObjectives": "array of strings",
  "duration": "string (estimated total duration)",
  "chapters": "array of chapter references"
}
```

### Chapter Metadata Contract
```json
{
  "id": "string (unique within module)",
  "title": "string (chapter title)",
  "description": "string (brief chapter overview)",
  "learningOutcomes": "array of strings",
  "duration": "string (estimated completion time)",
  "order": "integer (sequence position)",
  "prerequisites": "array of strings (chapter-specific requirements)"
}
```

### Content Page Contract
```json
{
  "slug": "string (URL-friendly identifier)",
  "title": "string (page title)",
  "description": "string (meta description)",
  "sidebar_label": "string (navigation label)",
  "sidebar_position": "integer (position in sidebar)",
  "keywords": "array of strings (SEO keywords)",
  "authors": "array of author objects (optional)",
  "tags": "array of strings (content tags)"
}
```

## Navigation Contract

### Sidebar Structure
```json
{
  "module-1": {
    "type": "category",
    "label": "Module 1: The Robotic Nervous System",
    "items": [
      {
        "type": "doc",
        "id": "module-1/intro-to-ros2"
      },
      {
        "type": "doc",
        "id": "module-1/python-agents-ros2"
      },
      {
        "type": "doc",
        "id": "module-1/humanoid-urdf-modeling"
      }
    ]
  }
}
```

## Content Validation Rules

### Frontmatter Requirements
1. Every content page must include required frontmatter fields
2. All string fields must be non-empty
3. Sidebar positions must be positive integers
4. Slugs must follow kebab-case format: `^[a-z0-9]+(?:-[a-z0-9]+)*$`

### Content Requirements
1. Each chapter must have an introduction section
2. Code examples must be properly formatted with language specification
3. All external links must be verified for accuracy
4. Images must include alt text for accessibility

## URL Structure Contract

### Module Pages
- Base: `/docs/module-1/`
- Chapter 1: `/docs/module-1/intro-to-ros2`
- Chapter 2: `/docs/module-1/python-agents-ros2`
- Chapter 3: `/docs/module-1/humanoid-urdf-modeling`

### Content Standards
1. URLs must be lowercase with hyphens as separators
2. URLs should reflect the content hierarchy
3. URLs must remain stable after publication
4. All internal links must use relative paths

## Content Update Contract

### Versioning
- Content changes follow semantic versioning for major structural changes
- Minor content updates don't require version changes
- Breaking changes (URL changes, major reorganization) require advance notice

### Change Notification
- Updates to learning objectives must be communicated to stakeholders
- Prerequisites changes must be validated against student requirements
- New content additions must follow the established content patterns