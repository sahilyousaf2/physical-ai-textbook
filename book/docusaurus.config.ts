import { themes as prismThemes } from 'prism-react-renderer';
import type { Config } from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

// This runs in Node.js - Don't use client-side code here

const config: Config = {
  title: 'Physical AI & Humanoid Robotics Textbook',
  tagline: 'A comprehensive textbook for teaching Physical AI & Humanoid Robotics',
  favicon: 'img/favicon.ico',

  future: {
    v4: true,
  },

  // ✅ VERCEL CONFIG
  url: 'https://physical-ai-textbook.vercel.app',
  baseUrl: '/',

  onBrokenLinks: 'throw',
  onBrokenMarkdownLinks: 'warn',

  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: './sidebars.ts',
          routeBasePath: '/docs',
          showLastUpdateTime: true,
          showLastUpdateAuthor: true,
        },
        blog: {
          showReadingTime: true,
          feedOptions: {
            type: ['rss', 'atom'],
            xslt: true,
          },
          onInlineTags: 'warn',
          onInlineAuthors: 'warn',
          onUntruncatedBlogPosts: 'warn',
        },
        theme: {
          customCss: './src/css/custom.css',
        },
      } satisfies Preset.Options,
    ],
  ],

  themes: [
    [
      require.resolve('@docusaurus/theme-mermaid'),
      {
        options: {
          maxZoom: 1.5,
          theme: { light: 'default', dark: 'dark' },
        },
      },
    ],
  ],

  themeConfig: {
    image: 'img/docusaurus-social-card.jpg',

    colorMode: {
      respectPrefersColorScheme: true,
    },

    navbar: {
      title: 'Physical AI Textbook',
      logo: {
        alt: 'Physical AI & Humanoid Robotics Textbook Logo',
        src: 'img/logo.svg',
      },
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'tutorialSidebar',
          position: 'left',
          label: 'Textbook',
        },
        {
          type: 'search',
          position: 'right',
        },
        {
          href: 'https://github.com/sahilyousaf2/physical-ai-textbook',
          label: 'GitHub',
          position: 'right',
        },
      ],
    },

    footer: {
      style: 'dark',
      links: [
        {
          title: 'Textbook',
          items: [
            { label: 'Introduction', to: '/docs/' },
            { label: 'Module 1: The Robotic Nervous System', to: '/docs/module-1' },
            { label: 'Module 2: The Digital Twin', to: '/docs/module-2' },
            { label: 'Module 3: The AI-Robot Brain', to: '/docs/module-3' },
            { label: 'Module 4: Vision-Language-Action', to: '/docs/module-4' },
          ],
        },
        {
          title: 'Resources',
          items: [
            { label: 'Setup Guide', to: '/docs/setup' },
            { label: 'Glossary', to: '/docs/glossary' },
            { label: 'Additional Resources', to: '/docs/resources' },
          ],
        },
        {
          title: 'More',
          items: [
            {
              label: 'GitHub',
              href: 'https://github.com/sahilyousaf2/physical-ai-textbook',
            },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} Physical AI Textbook Project. Built with Docusaurus.`,
    },

    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
    },
  } satisfies Preset.ThemeConfig,
};

export default config;
