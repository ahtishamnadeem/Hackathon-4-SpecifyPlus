// @ts-check
// `@type` JSDoc annotations allow editor autocompletion and type checking
// (when paired with `@ts-check`).
// There are various equivalent ways to declare your Docusaurus config.
// See: https://docusaurus.io/docs/api/docusaurus-config

import {themes as prismThemes} from 'prism-react-renderer';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI and Humanoid Robotics',
  tagline: 'Learn ROS 2 as the robotic nervous system for humanoid control, communication, and embodiment',
  favicon: 'img/favicon.ico',

  // Future flags, see https://docusaurus.io/docs/api/docusaurus-config#future
  future: {
    v4: true, // Improve compatibility with the upcoming Docusaurus v4
  },

  // Set the production url of your site here
  url: 'https://sigma-hackathon-4-specify-plus.vercel.app/',
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: '/',

  // GitHub pages deployment config.
  // If you aren't using GitHub pages, you don't need these.
  organizationName: 'your-username', // Usually your GitHub org/user name.
  projectName: 'ros2-fundamentals-book', // Usually your repo name.

  onBrokenLinks: 'throw',

  // Even if you don't use internationalization, you can use this field to set
  // useful metadata like html lang. For example, if your site is Chinese, you
  // may want to replace "en" with "zh-Hans".
  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          sidebarPath: './sidebars.js',
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/facebook/docusaurus/tree/main/packages/create-docusaurus/templates/shared/',
          showLastUpdateTime: true,
          showLastUpdateAuthor: true,
        },
        blog: {
          showReadingTime: true,
          feedOptions: {
            type: ['rss', 'atom'],
            xslt: true,
          },
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/facebook/docusaurus/tree/main/packages/create-docusaurus/templates/shared/',
          // Useful options to enforce blogging best practices
          onInlineTags: 'warn',
          onInlineAuthors: 'warn',
          onUntruncatedBlogPosts: 'warn',
        },
        theme: {
          customCss: './src/css/custom.css',
        },
        sitemap: {
          changefreq: 'weekly',
          priority: 0.5,
          filename: 'sitemap.xml',
          ignorePatterns: ['/tags/**'],
        },
      }),
    ],
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      // Replace with your project's social card
      image: 'img/docusaurus-social-card.jpg',
      colorMode: {
        defaultMode: 'light',
        disableSwitch: false,
        respectPrefersColorScheme: true,
      },
      metadata: [
        {name: 'keywords', content: 'ROS 2, robotics, AI, physical AI, humanoid robotics, navigation, Isaac Sim, Isaac ROS, robot operating system, autonomous robots, machine learning, computer vision'},
        {name: 'author', content: 'Physical AI and Humanoid Robotics Book'},
        {name: 'description', content: 'Learn ROS 2 as the robotic nervous system for humanoid control, communication, and embodiment. Comprehensive guide to robotics, AI, and physical AI applications.'},
        {name: 'og:type', content: 'website'},
        {name: 'og:site_name', content: 'Physical AI and Humanoid Robotics'},
        {name: 'og:description', content: 'Learn ROS 2 as the robotic nervous system for humanoid control, communication, and embodiment. Comprehensive guide to robotics, AI, and physical AI applications.'},
        {name: 'og:image', content: 'img/docusaurus-social-card.jpg'},
        {name: 'og:image:alt', content: 'ROS 2 Fundamentals Book Cover'},
        {name: 'og:url', content: 'https://your-ros2-book-site.example.com'},
        {name: 'twitter:card', content: 'summary_large_image'},
        {name: 'twitter:site', content: '@robotics'},
        {name: 'twitter:description', content: 'Learn ROS 2 as the robotic nervous system for humanoid control, communication, and embodiment.'},
        {name: 'twitter:image', content: 'img/docusaurus-social-card.jpg'},
        {name: 'twitter:image:alt', content: 'ROS 2 Fundamentals Book Cover'},
        {name: 'theme-color', content: '#2563eb'},
        {name: 'msapplication-TileColor', content: '#2563eb'},
        {name: 'application-name', content: 'Physical AI and Humanoid Robotics'},
      ],
      navbar: {
        title: 'Physical AI and Humanoid Robotics',
        logo: {
          alt: 'Robot Logo',
          src: 'img/robot.png',
          width: 32,
          height: 32,
        },
        style: 'primary',
        items: [
          {
            type: 'docSidebar',
            sidebarId: 'tutorialSidebar',
            position: 'left',
            label: 'Modules',
          },
          {
            to: '/docs/module-1/intro-to-ros2',
            label: 'Chapters',
            position: 'left'
          },
          {
            href: 'https://github.com/ahtishamnadeem',
            label: 'GitHub',
            position: 'right',
          },
        ],
      },
      footer: {
        style: 'dark',
        links: [
          {
            title: 'Learning Modules',
            items: [
              {
                label: 'Module 1: The Robotic Nervous System',
                to: '/docs/module-1/intro-to-ros2',
              },
              {
                label: 'Module 2: The Digital Twin',
                to: '/docs/module-2/',
              },
              {
                label: 'Module 3: The AI-Robot Brain',
                to: '/docs/module-3/',
              },
              {
                label: 'Module 4: Vision-Language-Action',
                to: '/docs/module-4/',
              },
            ],
          },
          {
            title: 'Resources',
            items: [
              {
                label: 'ROS 2 Official Documentation',
                href: 'https://docs.ros.org/en/rolling/',
              },
              {
                label: 'Physical AI Research',
                href: 'https://ai.google/research/physical-ai',
              },
              {
                label: 'Isaac ROS',
                href: 'https://nvidia-isaac-ros.github.io/',
              },
              {
                label: 'Isaac Sim',
                href: 'https://docs.omniverse.nvidia.com/isaacsim/latest/overview.html',
              },
            ],
          },
          {
            title: 'Community',
            items: [
              {
                label: 'GitHub',
                href: 'https://github.com/ahtishamnadeem',
              },
              {
                label: 'LinkedIn',
                href: 'https://www.linkedin.com/in/muhammadahtishamwebdev/',
              },
              {
                label: 'ROS Answers',
                href: 'https://answers.ros.org/questions/',
              },
              {
                label: 'Discord',
                href: 'https://discord.gg/robotics',
              },
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} ROS 2 Fundamentals Book. Built with Docusaurus.\nDeveloped by | CodeWithAhtii`,
      },
      prism: {
        theme: prismThemes.github,
        darkTheme: prismThemes.dracula,
        additionalLanguages: ['bash', 'json', 'python', 'docker'],
      },
      tableOfContents: {
        minHeadingLevel: 2,
        maxHeadingLevel: 4,
      },
    }),
};

export default config;
