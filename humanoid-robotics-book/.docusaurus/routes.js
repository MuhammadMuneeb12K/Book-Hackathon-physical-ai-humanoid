import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/__docusaurus/debug',
    component: ComponentCreator('/__docusaurus/debug', '5ff'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/config',
    component: ComponentCreator('/__docusaurus/debug/config', '5ba'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/content',
    component: ComponentCreator('/__docusaurus/debug/content', 'a2b'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/globalData',
    component: ComponentCreator('/__docusaurus/debug/globalData', 'c3c'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/metadata',
    component: ComponentCreator('/__docusaurus/debug/metadata', '156'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/registry',
    component: ComponentCreator('/__docusaurus/debug/registry', '88c'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/routes',
    component: ComponentCreator('/__docusaurus/debug/routes', '000'),
    exact: true
  },
  {
    path: '/',
    component: ComponentCreator('/', '927'),
    routes: [
      {
        path: '/',
        component: ComponentCreator('/', '574'),
        routes: [
          {
            path: '/',
            component: ComponentCreator('/', 'b95'),
            routes: [
              {
                path: '/appendices/',
                component: ComponentCreator('/appendices/', '19d'),
                exact: true,
                sidebar: "defaultSidebar"
              },
              {
                path: '/capstone/capstone-integration-guide',
                component: ComponentCreator('/capstone/capstone-integration-guide', '725'),
                exact: true,
                sidebar: "defaultSidebar"
              },
              {
                path: '/category/miscellaneous',
                component: ComponentCreator('/category/miscellaneous', '3d9'),
                exact: true,
                sidebar: "defaultSidebar"
              },
              {
                path: '/category/module-1-the-robotic-nervous-system-ros-2',
                component: ComponentCreator('/category/module-1-the-robotic-nervous-system-ros-2', '020'),
                exact: true,
                sidebar: "defaultSidebar"
              },
              {
                path: '/category/module-2-humanoid-robot-kinematics',
                component: ComponentCreator('/category/module-2-humanoid-robot-kinematics', '357'),
                exact: true,
                sidebar: "defaultSidebar"
              },
              {
                path: '/category/module-3-humanoid-robot-dynamics',
                component: ComponentCreator('/category/module-3-humanoid-robot-dynamics', '539'),
                exact: true,
                sidebar: "defaultSidebar"
              },
              {
                path: '/category/module-4-humanoid-robot-control',
                component: ComponentCreator('/category/module-4-humanoid-robot-control', '04b'),
                exact: true,
                sidebar: "defaultSidebar"
              },
              {
                path: '/getting-started',
                component: ComponentCreator('/getting-started', 'cb7'),
                exact: true,
                sidebar: "defaultSidebar"
              },
              {
                path: '/hardware-lab-setup/',
                component: ComponentCreator('/hardware-lab-setup/', 'ef9'),
                exact: true,
                sidebar: "defaultSidebar"
              },
              {
                path: '/home/overview',
                component: ComponentCreator('/home/overview', 'd79'),
                exact: true,
                sidebar: "defaultSidebar"
              },
              {
                path: '/misc/technical-plan',
                component: ComponentCreator('/misc/technical-plan', '6d1'),
                exact: true,
                sidebar: "defaultSidebar"
              },
              {
                path: '/module-1/building-python-nodes',
                component: ComponentCreator('/module-1/building-python-nodes', '8aa'),
                exact: true,
                sidebar: "defaultSidebar"
              },
              {
                path: '/module-1/launch-files-multi-node',
                component: ComponentCreator('/module-1/launch-files-multi-node', '59b'),
                exact: true,
                sidebar: "defaultSidebar"
              },
              {
                path: '/module-1/ros2-architecture',
                component: ComponentCreator('/module-1/ros2-architecture', '7ba'),
                exact: true,
                sidebar: "defaultSidebar"
              },
              {
                path: '/module-1/tf2-frames-transformations',
                component: ComponentCreator('/module-1/tf2-frames-transformations', '4d4'),
                exact: true,
                sidebar: "defaultSidebar"
              },
              {
                path: '/module-1/urdf-humanoid-robots',
                component: ComponentCreator('/module-1/urdf-humanoid-robots', 'b79'),
                exact: true,
                sidebar: "defaultSidebar"
              },
              {
                path: '/module-2/building-gazebo-worlds',
                component: ComponentCreator('/module-2/building-gazebo-worlds', '30d'),
                exact: true,
                sidebar: "defaultSidebar"
              },
              {
                path: '/module-2/physics-simulation',
                component: ComponentCreator('/module-2/physics-simulation', '8f1'),
                exact: true,
                sidebar: "defaultSidebar"
              },
              {
                path: '/module-2/sensor-simulation',
                component: ComponentCreator('/module-2/sensor-simulation', '708'),
                exact: true,
                sidebar: "defaultSidebar"
              },
              {
                path: '/module-2/unity-for-robotics',
                component: ComponentCreator('/module-2/unity-for-robotics', 'f36'),
                exact: true,
                sidebar: "defaultSidebar"
              },
              {
                path: '/module-2/what-is-digital-twin',
                component: ComponentCreator('/module-2/what-is-digital-twin', '308'),
                exact: true,
                sidebar: "defaultSidebar"
              },
              {
                path: '/module-3/introduction-isaac-sim',
                component: ComponentCreator('/module-3/introduction-isaac-sim', '394'),
                exact: true,
                sidebar: "defaultSidebar"
              },
              {
                path: '/module-3/navigation-nav2-humanoids',
                component: ComponentCreator('/module-3/navigation-nav2-humanoids', '4ff'),
                exact: true,
                sidebar: "defaultSidebar"
              },
              {
                path: '/module-3/perception-isaac-ros-vslam',
                component: ComponentCreator('/module-3/perception-isaac-ros-vslam', '90d'),
                exact: true,
                sidebar: "defaultSidebar"
              },
              {
                path: '/module-3/synthetic-data-domain-randomization',
                component: ComponentCreator('/module-3/synthetic-data-domain-randomization', '36d'),
                exact: true,
                sidebar: "defaultSidebar"
              },
              {
                path: '/module-3/training-robot-policies',
                component: ComponentCreator('/module-3/training-robot-policies', '775'),
                exact: true,
                sidebar: "defaultSidebar"
              },
              {
                path: '/module-4/full-vla-pipeline',
                component: ComponentCreator('/module-4/full-vla-pipeline', '3ef'),
                exact: true,
                sidebar: "defaultSidebar"
              },
              {
                path: '/module-4/llm-cognitive-planning',
                component: ComponentCreator('/module-4/llm-cognitive-planning', 'f99'),
                exact: true,
                sidebar: "defaultSidebar"
              },
              {
                path: '/module-4/visual-object-grounding',
                component: ComponentCreator('/module-4/visual-object-grounding', 'fe7'),
                exact: true,
                sidebar: "defaultSidebar"
              },
              {
                path: '/module-4/voice-command-whisper',
                component: ComponentCreator('/module-4/voice-command-whisper', 'b19'),
                exact: true,
                sidebar: "defaultSidebar"
              },
              {
                path: '/module-4/what-is-vla',
                component: ComponentCreator('/module-4/what-is-vla', '954'),
                exact: true,
                sidebar: "defaultSidebar"
              },
              {
                path: '/',
                component: ComponentCreator('/', 'a61'),
                exact: true,
                sidebar: "defaultSidebar"
              }
            ]
          }
        ]
      }
    ]
  },
  {
    path: '*',
    component: ComponentCreator('*'),
  },
];
