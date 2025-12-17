# Data Model: ROS 2 Fundamentals Educational Content

## Content Structure

### Module Entity
- **name**: string (e.g., "Module 1: The Robotic Nervous System")
- **description**: string (brief overview of the module)
- **target_audience**: string (AI and robotics students with basic Python knowledge)
- **chapters**: array of Chapter entities
- **learning_objectives**: array of strings (what students should learn)
- **prerequisites**: array of strings (what students should know beforehand)

### Chapter Entity
- **id**: string (unique identifier for the chapter)
- **title**: string (chapter title)
- **description**: string (brief overview of the chapter)
- **content**: string (path to markdown file)
- **order**: number (sequence in the module)
- **learning_outcomes**: array of strings (specific skills/knowledge)
- **duration_estimate**: string (estimated time to complete)

### Content Page Entity
- **slug**: string (URL-friendly identifier)
- **title**: string (page title)
- **frontmatter**: object (metadata for Docusaurus)
  - **title**: string
  - **description**: string
  - **keywords**: array of strings
  - **sidebar_label**: string
  - **sidebar_position**: number

## Validation Rules

### Module Validation
- Must have at least one chapter
- Module name must be unique within the course
- Learning objectives must align with functional requirements in spec
- Target audience must match specification

### Chapter Validation
- Chapter title must be unique within the module
- Content path must exist and be accessible
- Order must be a positive integer
- Learning outcomes must be specific and measurable
- Duration estimate must be reasonable (not negative)

### Content Page Validation
- Slug must follow kebab-case format
- Frontmatter must include required fields
- Content file must be in Markdown format
- All internal links must be valid

## State Transitions

### Content Creation Workflow
1. **Draft**: Content is being written, not yet ready for review
2. **Review**: Content is complete and ready for peer review
3. **Approved**: Content has passed review and is ready for publication
4. **Published**: Content is live in the documentation site

### Content Update Workflow
- Published content that needs updates moves back to Draft state
- Updated content follows the same workflow: Draft → Review → Approved → Published
- Major updates may require additional stakeholder approval