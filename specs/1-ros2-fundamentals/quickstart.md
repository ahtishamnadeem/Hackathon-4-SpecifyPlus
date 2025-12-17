# Quickstart: Setting up the ROS 2 Fundamentals Book

## Prerequisites

- Node.js (version 18 or higher)
- npm or yarn package manager
- Git for version control
- A code editor (VS Code recommended)

## Installation Steps

### 1. Clone the Repository
```bash
git clone <repository-url>
cd <repository-name>
```

### 2. Install Dependencies
```bash
npm install
# or
yarn install
```

### 3. Start the Development Server
```bash
npm run start
# or
yarn start
```

This will start the Docusaurus development server at `http://localhost:3000`.

### 4. Access Module 1 Content
Once the server is running, navigate to:
- Main book: `http://localhost:3000`
- Module 1: `http://localhost:3000/docs/module-1`
- Chapter 1: `http://localhost:3000/docs/module-1/intro-to-ros2`
- Chapter 2: `http://localhost:3000/docs/module-1/python-agents-ros2`
- Chapter 3: `http://localhost:3000/docs/module-1/humanoid-urdf-modeling`

## Adding New Content

### Creating a New Chapter
1. Create a new Markdown file in `docs/module-1/`
2. Add frontmatter with title, description, and sidebar properties
3. Update `sidebars.js` to include the new chapter in the navigation

### Example Chapter Frontmatter
```markdown
---
title: "Chapter Title"
description: "Brief description of the chapter content"
sidebar_label: "Chapter Title"
sidebar_position: 2
---

# Chapter Title

Content goes here...
```

## Building for Production

To build the static site for deployment:

```bash
npm run build
# or
yarn build
```

The built site will be available in the `build/` directory.

## Deployment

The site is configured for GitHub Pages deployment. After building, the content in the `build/` directory can be deployed to GitHub Pages.

For GitHub Actions deployment, ensure the workflow file is properly configured in `.github/workflows/deploy.yml`.

## Troubleshooting

### Common Issues

1. **Port already in use**
   - Error: `EADDRINUSE: address already in use`
   - Solution: Use a different port with `npm run start -- --port 3001`

2. **Dependency conflicts**
   - Error during `npm install`
   - Solution: Clear cache with `npm cache clean --force` and reinstall

3. **Build failures**
   - Check for syntax errors in Markdown files
   - Ensure all internal links are valid
   - Verify image paths are correct