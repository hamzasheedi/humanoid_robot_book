// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    'index',
    {
      type: 'category',
      label: 'Textbook Modules',
      items: [
        'modules/index',
        {
          type: 'category',
          label: 'Digital Twin',
          items: [
            'modules/digital-twin/index',
            'modules/digital-twin/introduction',
            {
              type: 'category',
              label: 'Unity',
              items: [
                'modules/digital-twin/unity/index',
              ],
            },
            {
              type: 'category',
              label: 'Gazebo',
              items: [
                'modules/digital-twin/gazebo/index',
              ],
            },
            'modules/digital-twin/hardware-compatibility',
          ],
        },
        {
          type: 'category',
          label: 'ROS2',
          items: [
            'modules/ros2/index',
            'modules/ros2/introduction',
            'modules/ros2/setup',
            {
              type: 'category',
              label: 'Tutorials',
              items: [
                'modules/ros2/tutorials/index',
                'modules/ros2/tutorials/publisher-subscriber',
                'modules/ros2/tutorials/action-service',
              ],
            },
            'modules/ros2/troubleshooting',
          ],
        },
        {
          type: 'category',
          label: 'NVIDIA Isaac',
          items: [
            'modules/nvidia-isaac/index',
            'modules/nvidia-isaac/introduction',
            'modules/nvidia-isaac/perception',
            'modules/nvidia-isaac/navigation',
            'modules/nvidia-isaac/reinforcement-learning',
          ],
        },
        {
          type: 'category',
          label: 'VLA',
          items: [
            'modules/vla/index',
            'modules/vla/introduction',
            'modules/vla/model-compatibility',
            'modules/vla/cognitive-planning',
            'modules/vla/voice-control',
          ],
        },
        {
          type: 'category',
          label: 'Capstone',
          items: [
            'modules/capstone/index',
            'modules/capstone/introduction',
            'modules/capstone/project-outline',
            'modules/capstone/integration-guide',
            'modules/capstone/validation-and-troubleshooting',
          ],
        },
        'chatbot-integration',
      ],
    },
    {
      type: 'category',
      label: 'Exercises & Assessments',
      items: [
        'exercises/index',
        'assessments/index',
        {
          type: 'category',
          label: 'Fundamentals Quizzes',
          items: [
            'exercises/assessments/ros2/fundamentals-quiz',
            'exercises/assessments/nvidia-isaac/fundamentals-quiz',
            'exercises/assessments/vla/fundamentals-quiz',
            'exercises/assessments/digital-twin/fundamentals-quiz',
          ],
        },
      ],
    },
    {
      type: 'category',
      label: 'Deployment',
      items: [
        'deployment-options',
        'vercel-deployment',
        'vercel-deployment-powershell-guide',
        'vercel-deployment-verification',
        'vercel-deployment-complete',
        'deployment-validation',
        'length-validation-framework',
      ],
    },
    {
      type: 'category',
      label: 'Learning Resources',
      items: [
        'learning-path-options',
      ],
    },
    {
      type: 'category',
      label: 'References',
      items: [
        'references/index',
        'references/glossary',
        'references/external-links',
        'references/citations/standards',
        'references/citation-standards',
        {
          type: 'category',
          label: 'Hardware Specifications',
          items: [
            'references/hardware-specifications/index',
          ],
        },
        {
          type: 'category',
          label: 'Robot Platforms',
          items: [
            'references/robot-platforms/index',
          ],
        },
      ],
    },
  ],
};

module.exports = sidebars;