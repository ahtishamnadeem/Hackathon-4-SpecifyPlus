# Data Model: AI-Robot Brain Educational Content

## Content Structure

### Module Entity
- **name**: string (e.g., "Module 3: The AI-Robot Brain")
- **description**: string (brief overview of the module)
- **target_audience**: string (AI and robotics students advancing into perception, navigation, and AI-driven humanoid control)
- **chapters**: array of Chapter entities
- **learning_objectives**: array of strings (what students should learn)
- **prerequisites**: array of strings (what students should know beforehand, including Module 1 and basic CUDA/GPU computing knowledge)

### Chapter Entity
- **id**: string (unique identifier for the chapter)
- **title**: string (chapter title)
- **description**: string (brief overview of the chapter)
- **content**: string (path to markdown file)
- **order**: number (sequence in the module)
- **learning_outcomes**: array of strings (specific skills/knowledge)
- **duration_estimate**: string (estimated time to complete)
- **hardware_requirements**: string (specific hardware needs like GPU specifications)

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
  - **hardware_requirements**: array of strings (GPU, CUDA, etc.)

## Key Entities for AI-Robot Brain Concepts

### Isaac Sim Environment Entity
- **name**: string (name of the simulation environment)
- **description**: string (what the environment is designed to simulate)
- **complexity_level**: string (basic, intermediate, advanced)
- **rendering_features**: array of strings (photorealistic rendering, lighting models, etc.)
- **physics_properties**: object (accuracy settings, simulation parameters)
- **synthetic_data_capabilities**: array of strings (types of data that can be generated)

### Isaac ROS Pipeline Entity
- **name**: string (name of the pipeline)
- **components**: array of strings (specific Isaac ROS packages used)
- **acceleration_type**: string (GPU, tensor core, etc.)
- **performance_metrics**: object (speedup ratios, FPS, etc.)
- **supported_sensors**: array of strings (sensor types supported by the pipeline)
- **real_time_performance**: boolean (whether it meets real-time requirements)

### VSLAM System Entity
- **algorithm_type**: string (ORB-SLAM, RTAB-MAP, etc.)
- **input_modalities**: array of strings (stereo cameras, RGB-D, etc.)
- **output_types**: array of strings (point clouds, pose estimates, etc.)
- **accuracy_metrics**: object (position, orientation accuracy)
- **computational_requirements**: object (GPU memory, compute capability)
- **environment_constraints**: array of strings (lighting, texture, etc.)

### Nav2 Humanoid Planner Entity
- **locomotion_type**: string (bipedal, with specific gait patterns)
- **stability_requirements**: array of strings (balance constraints)
- **navigation_constraints**: array of strings (footstep planning, COM control)
- **costmap_modifications**: object (custom layers for humanoid navigation)
- **controller_adaptations**: array of strings (specialized controllers for bipedal robots)

### Synthetic Data Generator Entity
- **data_type**: string (images, point clouds, sensor data)
- **annotation_quality**: string (ground truth accuracy)
- **diversity_metrics**: object (variance in lighting, angles, etc.)
- **labeling_schema**: object (format and types of labels provided)
- **simulation_fidelity**: string (level of realism in generated data)

## Validation Rules

### Module Validation
- Must have at least one chapter covering Isaac Sim
- Module name must be unique within the course
- Learning objectives must align with functional requirements in spec
- Target audience must match specification

### Chapter Validation
- Chapter title must be unique within the module
- Content path must exist and be accessible
- Order must be a positive integer
- Learning outcomes must be specific and measurable
- Hardware requirements must be clearly specified

### Content Page Validation
- Slug must follow kebab-case format
- Frontmatter must include required fields
- Content file must be in Markdown format
- All internal links must be valid
- Code examples must be properly formatted with Isaac-specific syntax

### Isaac Integration Validation
- Each Isaac Sim environment must be runnable with documented setup
- Isaac ROS pipelines must demonstrate hardware acceleration benefits
- Nav2 configurations must be adapted for humanoid-specific constraints
- Performance benchmarks must be reproducible and verifiable

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