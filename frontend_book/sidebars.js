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
      label: 'Module 2: The Digital Twin',
      items: [
        {
          type: 'doc',
          id: 'module-2/index'
        },
        {
          type: 'doc',
          id: 'module-2/physics-simulation-gazebo'
        },
        {
          type: 'doc',
          id: 'module-2/high-fidelity-unity'
        },
        {
          type: 'doc',
          id: 'module-2/sensor-simulation'
        }
      ]
    },
    {
      type: 'category',
      label: 'Module 3: The AI-Robot Brain',
      items: [
        {
          type: 'doc',
          id: 'module-3/index'
        },
        {
          type: 'doc',
          id: 'module-3/photorealistic-simulation-isaac-sim'
        },
        {
          type: 'doc',
          id: 'module-3/hardware-accelerated-ai-isaac-ros'
        },
        {
          type: 'doc',
          id: 'module-3/path-planning-nav2'
        }
      ]
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action',
      items: [
        {
          type: 'doc',
          id: 'module-4/index'
        },
        {
          type: 'doc',
          id: 'module-4/voice-to-action-whisper'
        },
        {
          type: 'doc',
          id: 'module-4/cognitive-planning-llms'
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
