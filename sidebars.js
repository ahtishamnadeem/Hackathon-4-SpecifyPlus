// @ts-check

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.

 @type {import('@docusaurus/plugin-content-docs').SidebarsConfig}
 */
const sidebars = {
  // By default, Docusaurus generates a sidebar from the docs folder structure
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System',
      items: [
        {
          type: 'doc',
          id: 'module-1/index'
        },
        {
          type: 'doc',
          id: 'module-1/intro-to-ros2'
        },
        {
          type: 'doc',
          id: 'module-1/python-agents-ros2'
        },
        {
          type: 'doc',
          id: 'module-1/humanoid-urdf-modeling'
        }
      ]
    },
    {
      type: 'category',
      label: 'Module 2: Advanced ROS 2 Concepts',
      items: [
        {
          type: 'doc',
          id: 'module-2/index'
        },
        {
          type: 'doc',
          id: 'module-2/advanced-pubsub-patterns'
        },
        {
          type: 'doc',
          id: 'module-2/lifecycle-nodes'
        },
        {
          type: 'doc',
          id: 'module-2/performance-optimization'
        }
      ]
    },
    {
      type: 'category',
      label: 'Module 3: Perception and Navigation Systems',
      items: [
        {
          type: 'doc',
          id: 'module-3/index'
        },
        {
          type: 'doc',
          id: 'module-3/sensor-integration'
        },
        {
          type: 'doc',
          id: 'module-3/computer-vision'
        },
        {
          type: 'doc',
          id: 'module-3/navigation-systems'
        }
      ]
    },
    {
      type: 'category',
      label: 'Module 4: AI Integration and Capstone Project',
      items: [
        {
          type: 'doc',
          id: 'module-4/index'
        },
        {
          type: 'doc',
          id: 'module-4/cognitive-planning-llms'
        },
        {
          type: 'doc',
          id: 'module-4/voice-to-action-whisper'
        },
        {
          type: 'doc',
          id: 'module-4/capstone-autonomous-humanoid'
        }
      ]
    }
  ],

  // But you can create a sidebar manually
  /*
  tutorialSidebar: [
    'intro',
    'hello',
    {
      type: 'category',
      label: 'Tutorial',
      items: ['tutorial-basics/create-a-document'],
    },
  ],
   */
};

export default sidebars;
