# Data Model: Docusaurus Project Audit and Production Readiness

**Feature**: 6-docusaurus-audit
**Date**: 2025-12-17
**Status**: Complete

## Configuration Entities

### Docusaurus Configuration
- **Name**: docusaurus.config.js
- **Purpose**: Main site configuration including metadata, themes, and navigation
- **Fields**:
  - title: Site title (string)
  - tagline: Site tagline (string)
  - url: Production URL (string)
  - baseUrl: Base path (string)
  - favicon: Favicon path (string)
  - organizationName: GitHub org/user name (string)
  - projectName: Repository name (string)
  - presets: Docusaurus presets (array)
  - themeConfig: Theme configuration (object)
- **Relationships**: Links to sidebar configuration and content pages

### Sidebar Configuration
- **Name**: sidebars.js
- **Purpose**: Navigation structure and content organization
- **Fields**:
  - type: Item type (string - 'category' or 'doc')
  - label: Display label (string)
  - items: Child items (array)
  - link: Navigation link (object)
- **Relationships**: References to documentation files in docs/ directory

## Content Entities

### Documentation Page
- **Name**: MD/MDX files in docs/ directory
- **Purpose**: Educational content for the ROS 2 course
- **Fields**:
  - id: Document identifier (string)
  - title: Page title (string)
  - sidebar_label: Navigation label (string)
  - description: Page description (string)
  - tags: Content tags (array)
- **Relationships**: Referenced by sidebar configuration for navigation

## Theme and Styling Entities

### CSS Custom Properties
- **Name**: src/css/custom.css
- **Purpose**: Custom styling and theme configuration
- **Fields**:
  - color variables: --ifm-color-* (CSS custom properties)
  - spacing variables: --ifm-spacing-* (CSS custom properties)
  - typography variables: --ifm-font-* (CSS custom properties)
  - component variables: --ifm-* (CSS custom properties)
- **Relationships**: Used by Docusaurus components for consistent theming

### Theme Components
- **Name**: Docusaurus theme components
- **Purpose**: UI elements and layouts
- **Fields**:
  - Navbar: Navigation header component
  - Sidebar: Navigation sidebar component
  - Footer: Page footer component
  - Layout: Page layout components
- **Relationships**: Configured via themeConfig in docusaurus.config.js

## SEO and Metadata Entities

### Meta Tags Configuration
- **Name**: Docusaurus meta tag system
- **Purpose**: SEO and social sharing metadata
- **Fields**:
  - title: Page title (string)
  - description: Page description (string)
  - keywords: SEO keywords (array)
  - author: Content author (string)
  - image: Social sharing image (string)
- **Relationships**: Generated from frontmatter in MD/MDX files

### Sitemap Configuration
- **Name**: Docusaurus sitemap plugin
- **Purpose**: Search engine crawling and indexing
- **Fields**:
  - changefreq: Change frequency (string)
  - priority: Page priority (number)
  - lastmod: Last modification date (date)
- **Relationships**: Generated from site content and configuration

## Performance Optimization Entities

### Build Configuration
- **Name**: Docusaurus build settings
- **Purpose**: Production build optimization
- **Fields**:
  - minification: Code minification settings (boolean/object)
  - compression: Asset compression settings (object)
  - code splitting: Code splitting configuration (object)
- **Relationships**: Applied during npm run build process

### Asset Optimization
- **Name**: Static asset handling
- **Purpose**: Image and file optimization
- **Fields**:
  - images: Image optimization settings (object)
  - fonts: Font loading optimization (object)
  - scripts: JavaScript optimization (object)
- **Relationships**: Handled by Docusaurus build process