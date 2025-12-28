import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/Book-Hackathon-physical-ai-humanoid/__docusaurus/debug/',
    component: ComponentCreator('/Book-Hackathon-physical-ai-humanoid/__docusaurus/debug/', '205'),
    exact: true
  },
  {
    path: '/Book-Hackathon-physical-ai-humanoid/__docusaurus/debug/config/',
    component: ComponentCreator('/Book-Hackathon-physical-ai-humanoid/__docusaurus/debug/config/', '1c2'),
    exact: true
  },
  {
    path: '/Book-Hackathon-physical-ai-humanoid/__docusaurus/debug/content/',
    component: ComponentCreator('/Book-Hackathon-physical-ai-humanoid/__docusaurus/debug/content/', '1d0'),
    exact: true
  },
  {
    path: '/Book-Hackathon-physical-ai-humanoid/__docusaurus/debug/globalData/',
    component: ComponentCreator('/Book-Hackathon-physical-ai-humanoid/__docusaurus/debug/globalData/', 'abf'),
    exact: true
  },
  {
    path: '/Book-Hackathon-physical-ai-humanoid/__docusaurus/debug/metadata/',
    component: ComponentCreator('/Book-Hackathon-physical-ai-humanoid/__docusaurus/debug/metadata/', 'c10'),
    exact: true
  },
  {
    path: '/Book-Hackathon-physical-ai-humanoid/__docusaurus/debug/registry/',
    component: ComponentCreator('/Book-Hackathon-physical-ai-humanoid/__docusaurus/debug/registry/', '2c4'),
    exact: true
  },
  {
    path: '/Book-Hackathon-physical-ai-humanoid/__docusaurus/debug/routes/',
    component: ComponentCreator('/Book-Hackathon-physical-ai-humanoid/__docusaurus/debug/routes/', 'fbe'),
    exact: true
  },
  {
    path: '/Book-Hackathon-physical-ai-humanoid/',
    component: ComponentCreator('/Book-Hackathon-physical-ai-humanoid/', '99f'),
    routes: [
      {
        path: '/Book-Hackathon-physical-ai-humanoid/',
        component: ComponentCreator('/Book-Hackathon-physical-ai-humanoid/', 'a5d'),
        routes: [
          {
            path: '/Book-Hackathon-physical-ai-humanoid/',
            component: ComponentCreator('/Book-Hackathon-physical-ai-humanoid/', 'dd9'),
            routes: [
              {
                path: '/Book-Hackathon-physical-ai-humanoid/appendices/',
                component: ComponentCreator('/Book-Hackathon-physical-ai-humanoid/appendices/', 'd72'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Book-Hackathon-physical-ai-humanoid/capstone/capstone-integration-guide/',
                component: ComponentCreator('/Book-Hackathon-physical-ai-humanoid/capstone/capstone-integration-guide/', '55b'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Book-Hackathon-physical-ai-humanoid/getting-started/',
                component: ComponentCreator('/Book-Hackathon-physical-ai-humanoid/getting-started/', 'e22'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Book-Hackathon-physical-ai-humanoid/hardware-lab-setup/',
                component: ComponentCreator('/Book-Hackathon-physical-ai-humanoid/hardware-lab-setup/', '024'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Book-Hackathon-physical-ai-humanoid/home/overview/',
                component: ComponentCreator('/Book-Hackathon-physical-ai-humanoid/home/overview/', 'de2'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Book-Hackathon-physical-ai-humanoid/misc/technical-plan/',
                component: ComponentCreator('/Book-Hackathon-physical-ai-humanoid/misc/technical-plan/', 'b81'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Book-Hackathon-physical-ai-humanoid/module-1/building-python-nodes/',
                component: ComponentCreator('/Book-Hackathon-physical-ai-humanoid/module-1/building-python-nodes/', '88c'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Book-Hackathon-physical-ai-humanoid/module-1/launch-files-multi-node/',
                component: ComponentCreator('/Book-Hackathon-physical-ai-humanoid/module-1/launch-files-multi-node/', '9e1'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Book-Hackathon-physical-ai-humanoid/module-1/ros2-architecture/',
                component: ComponentCreator('/Book-Hackathon-physical-ai-humanoid/module-1/ros2-architecture/', '503'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Book-Hackathon-physical-ai-humanoid/module-1/tf2-frames-transformations/',
                component: ComponentCreator('/Book-Hackathon-physical-ai-humanoid/module-1/tf2-frames-transformations/', '9a7'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Book-Hackathon-physical-ai-humanoid/module-1/urdf-humanoid-robots/',
                component: ComponentCreator('/Book-Hackathon-physical-ai-humanoid/module-1/urdf-humanoid-robots/', '019'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Book-Hackathon-physical-ai-humanoid/module-2/building-gazebo-worlds/',
                component: ComponentCreator('/Book-Hackathon-physical-ai-humanoid/module-2/building-gazebo-worlds/', '365'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Book-Hackathon-physical-ai-humanoid/module-2/physics-simulation/',
                component: ComponentCreator('/Book-Hackathon-physical-ai-humanoid/module-2/physics-simulation/', 'a57'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Book-Hackathon-physical-ai-humanoid/module-2/sensor-simulation/',
                component: ComponentCreator('/Book-Hackathon-physical-ai-humanoid/module-2/sensor-simulation/', '289'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Book-Hackathon-physical-ai-humanoid/module-2/unity-for-robotics/',
                component: ComponentCreator('/Book-Hackathon-physical-ai-humanoid/module-2/unity-for-robotics/', '1c4'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Book-Hackathon-physical-ai-humanoid/module-2/what-is-digital-twin/',
                component: ComponentCreator('/Book-Hackathon-physical-ai-humanoid/module-2/what-is-digital-twin/', 'a1d'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Book-Hackathon-physical-ai-humanoid/module-3/introduction-isaac-sim/',
                component: ComponentCreator('/Book-Hackathon-physical-ai-humanoid/module-3/introduction-isaac-sim/', 'b0d'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Book-Hackathon-physical-ai-humanoid/module-3/navigation-nav2-humanoids/',
                component: ComponentCreator('/Book-Hackathon-physical-ai-humanoid/module-3/navigation-nav2-humanoids/', '4f8'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Book-Hackathon-physical-ai-humanoid/module-3/perception-isaac-ros-vslam/',
                component: ComponentCreator('/Book-Hackathon-physical-ai-humanoid/module-3/perception-isaac-ros-vslam/', 'aa9'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Book-Hackathon-physical-ai-humanoid/module-3/synthetic-data-domain-randomization/',
                component: ComponentCreator('/Book-Hackathon-physical-ai-humanoid/module-3/synthetic-data-domain-randomization/', 'c6b'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Book-Hackathon-physical-ai-humanoid/module-3/training-robot-policies/',
                component: ComponentCreator('/Book-Hackathon-physical-ai-humanoid/module-3/training-robot-policies/', '0f0'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Book-Hackathon-physical-ai-humanoid/module-4/full-vla-pipeline/',
                component: ComponentCreator('/Book-Hackathon-physical-ai-humanoid/module-4/full-vla-pipeline/', 'dc0'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Book-Hackathon-physical-ai-humanoid/module-4/llm-cognitive-planning/',
                component: ComponentCreator('/Book-Hackathon-physical-ai-humanoid/module-4/llm-cognitive-planning/', 'adc'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Book-Hackathon-physical-ai-humanoid/module-4/visual-object-grounding/',
                component: ComponentCreator('/Book-Hackathon-physical-ai-humanoid/module-4/visual-object-grounding/', '0eb'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Book-Hackathon-physical-ai-humanoid/module-4/voice-command-whisper/',
                component: ComponentCreator('/Book-Hackathon-physical-ai-humanoid/module-4/voice-command-whisper/', '826'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Book-Hackathon-physical-ai-humanoid/module-4/what-is-vla/',
                component: ComponentCreator('/Book-Hackathon-physical-ai-humanoid/module-4/what-is-vla/', '83b'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Book-Hackathon-physical-ai-humanoid/',
                component: ComponentCreator('/Book-Hackathon-physical-ai-humanoid/', 'be7'),
                exact: true,
                sidebar: "tutorialSidebar"
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
