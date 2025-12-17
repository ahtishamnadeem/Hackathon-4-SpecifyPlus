# Data Model: Digital Twin Educational Content

## Content Structure

### Module Entity
- **name**: string (e.g., "Module 2: The Digital Twin")
- **description**: string (brief overview of the module)
- **target_audience**: string (AI and robotics students progressing into simulation and embodied AI)
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
  - **learning_outcomes**: array of strings

## Key Entities for Digital Twin Simulation

### Digital Twin Entity
- **name**: string (name of the digital twin system)
- **physical_counterpart**: string (description of the real-world robot/system)
- **simulation_environment**: string (Gazebo, Unity, or other)
- **sensors**: array of Sensor entities
- **physics_properties**: object (mass, friction, damping, etc.)
- **synchronization_frequency**: number (how often state is synchronized)

### Simulation Environment Entity
- **type**: string (gazebo, unity, webots, etc.)
- **version**: string (version of the simulation software)
- **physics_engine**: string (ODE, Bullet, etc.)
- **supported_sensors**: array of strings (sensor types supported)
- **rendering_capabilities**: object (visual quality, frame rate, etc.)

### Sensor Entity
- **type**: string (lidar, camera, imu, etc.)
- **model**: string (specific sensor model if applicable)
- **parameters**: object (range, resolution, accuracy, etc.)
- **noise_model**: object (how to simulate real sensor noise)
- **data_format**: string (format of the simulated data)

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

### Digital Twin Validation
- Each digital twin must have a corresponding physical system
- Simulation parameters must be realistic
- Sensor configurations must match real-world capabilities
- Synchronization mechanisms must be clearly defined

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