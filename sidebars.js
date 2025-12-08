// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    {
      type: 'doc',
      id: 'index',
      label: 'Home'
    },
    {
      type: 'category',
      label: 'Modules',
      collapsed: false,
      items: [
        {
          type: 'category',
          label: 'ROS 2 (Robot Operating System 2)',
          collapsed: false,
          items: [
            'modules/ros2/index',
            'modules/ros2/introduction',
            'modules/ros2/setup',
            'modules/ros2/troubleshooting',
            {
              type: 'category',
              label: 'Tutorials',
              items: [
                'modules/ros2/tutorials/publisher-subscriber',
                'modules/ros2/tutorials/action-service',
              ]
            },
            {
              type: 'category',
              label: 'Exercises',
              items: [
                'modules/ros2/exercises/index',
              ]
            }
          ],
        },
        {
          type: 'category',
          label: 'Digital Twin Simulation (Gazebo & Unity)',
          collapsed: false,
          items: [
            'modules/digital-twin/index',
            'modules/digital-twin/introduction',
            'modules/digital-twin/hardware-compatibility',
            {
              type: 'category',
              label: 'Gazebo Simulation',
              items: [
                'modules/digital-twin/gazebo/index',
              ]
            },
            {
              type: 'category',
              label: 'Unity Simulation',
              items: [
                'modules/digital-twin/unity/index',
              ]
            },
            {
              type: 'category',
              label: 'Exercises',
              items: [
                'modules/digital-twin/exercises/index',
              ]
            }
          ],
        },
        {
          type: 'category',
          label: 'NVIDIA Isaac (AI Perception & Navigation)',
          collapsed: false,
          items: [
            'modules/nvidia-isaac/index',
            'modules/nvidia-isaac/introduction',
            'modules/nvidia-isaac/perception',
            'modules/nvidia-isaac/navigation',
            'modules/nvidia-isaac/reinforcement-learning',
            {
              type: 'category',
              label: 'Exercises',
              items: [
                'modules/nvidia-isaac/exercises/index',
              ]
            }
          ],
        },
        {
          type: 'category',
          label: 'Vision-Language-Action (VLA)',
          collapsed: false,
          items: [
            'modules/vla/index',
            'modules/vla/introduction',
            'modules/vla/voice-control',
            'modules/vla/cognitive-planning',
            'modules/vla/model-compatibility',
            {
              type: 'category',
              label: 'Exercises',
              items: [
                'modules/vla/exercises/index',
              ]
            }
          ],
        },
        {
          type: 'category',
          label: 'Capstone Project',
          collapsed: false,
          items: [
            'modules/capstone/index',
            'modules/capstone/introduction',
            'modules/capstone/project-outline',
            'modules/capstone/integration-guide',
            'modules/capstone/validation-and-troubleshooting',
            {
              type: 'category',
              label: 'Exercises',
              items: [
                'modules/capstone/exercises/index',
              ]
            }
          ],
        }
      ],
    },
    {
      type: 'category',
      label: 'Learning Resources',
      collapsed: false,
      items: [
        'learning-path-options',
        {
          type: 'category',
          label: 'Exercises & Assessments',
          items: [
            'exercises/index',
            'exercises/assessments/ros2/fundamentals-quiz',
            'exercises/assessments/digital-twin/fundamentals-quiz',
            'exercises/assessments/nvidia-isaac/fundamentals-quiz',
            'exercises/assessments/vla/fundamentals-quiz',
          ]
        },
      ],
    },
    {
      type: 'category',
      label: 'References',
      collapsed: true,
      items: [
        'references/glossary',
        'references/external-links',
        'references/citation-standards',
        {
          type: 'category',
          label: 'Hardware Specifications',
          items: [
            'references/hardware-specifications/index',
          ]
        },
        {
          type: 'category',
          label: 'Robot Platforms',
          items: [
            'references/robot-platforms/index',
          ]
        }
      ],
    },
    'deployment-validation',
    'length-validation-framework',
  ],
};

export default sidebars;