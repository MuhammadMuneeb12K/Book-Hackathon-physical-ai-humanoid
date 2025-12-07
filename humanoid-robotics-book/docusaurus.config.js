// docusaurus.config.js
module.exports = {
  title: 'Humanoid Robotics Book',
  tagline: 'Project Documentation',
  url: 'https://example.com',
  baseUrl: '/',
  onBrokenLinks: 'throw',
  markdown: {
    hooks: {
      onBrokenMarkdownLinks: 'warn',
    },
  },
  favicon: 'img/favicon.ico',
  organizationName: 'your-org', 
  projectName: 'humanoid-robotics-book', 
  presets: [
    [
      '@docusaurus/preset-classic',
      {
        docs: {
          path: 'docs',         // your docs folder
          routeBasePath: '/',   // root path
        },
        theme: {
          customCss: require.resolve('./src/css/custom.css'),
        },
      },
    ],
  ],
};
