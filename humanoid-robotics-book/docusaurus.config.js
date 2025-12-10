// docusaurus.config.js




module.exports = {
  title: 'Humanoid Robotics Book',
  tagline: 'Project Documentation',
  url: 'https://muhammadmuneeb12k.github.io',
  baseUrl: '/Book-Hackathon-physical-ai-humanoid/',
  trailingSlash: true,
  onBrokenLinks: 'throw',
  markdown: {
    hooks: {
      onBrokenMarkdownLinks: 'warn',
    },
  },
  favicon: 'img/favicon.ico',
  organizationName: 'muhammadmuneeb12k', 
  projectName: 'Book-Hackathon-physical-ai-humanoid', 
  presets: [
    [
      '@docusaurus/preset-classic',
      {
        docs: {
          sidebarPath: require.resolve('./sidebars.js'),
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
