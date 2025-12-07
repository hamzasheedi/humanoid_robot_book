# Data Model: AI Robotics Textbook

## Core Entities

### Module
- **name**: String (required) - Name of the module (e.g., "ROS 2 Fundamentals")
- **description**: String (required) - Brief description of the module
- **learningObjectives**: Array[String] (required) - List of learning objectives for the module
- **prerequisites**: Array[String] - List of prerequisite knowledge/skills
- **duration**: Number - Estimated time to complete in hours
- **difficulty**: Enum("beginner", "intermediate", "advanced") - Difficulty level
- **contentPath**: String (required) - Path to the module's content files
- **assets**: Array[Asset] - Associated diagrams, images, code examples

### Lesson
- **title**: String (required) - Title of the lesson
- **module**: Module (foreign key) - The module this lesson belongs to
- **content**: String (required) - The lesson content in Markdown format
- **order**: Number (required) - Order of the lesson within the module
- **duration**: Number - Estimated time to complete in minutes
- **objectives**: Array[String] - Learning objectives for this lesson
- **exercises**: Array[Exercise] - Exercises associated with this lesson

### Exercise
- **title**: String (required) - Title of the exercise
- **description**: String (required) - Detailed description of the exercise
- **type**: Enum("practical", "theoretical", "assessment") - Type of exercise
- **difficulty**: Enum("beginner", "intermediate", "advanced") - Difficulty level
- **instructions**: String (required) - Step-by-step instructions
- **expectedOutcome**: String - What the student should achieve
- **solution**: String - Solution or reference implementation
- **assets**: Array[Asset] - Associated files, diagrams, or code for the exercise

### Asset
- **name**: String (required) - Name of the asset
- **type**: Enum("image", "diagram", "code", "simulation", "video", "model") - Type of asset
- **path**: String (required) - Path to the asset file
- **description**: String - Description of the asset's purpose
- **contentType**: String - MIME type or file extension
- **associatedLessons**: Array[Lesson] - Lessons that use this asset

### Assessment
- **title**: String (required) - Title of the assessment
- **module**: Module (foreign key) - The module this assessment belongs to
- **questions**: Array[Question] - Questions in the assessment
- **passingScore**: Number - Minimum percentage needed to pass
- **timeLimit**: Number - Time limit in minutes (0 if no limit)

### Question
- **type**: Enum("multiple-choice", "short-answer", "practical") - Type of question
- **text**: String (required) - Question text
- **options**: Array[String] - Options for multiple choice questions
- **correctAnswer**: String - Correct answer
- **explanation**: String - Explanation of the correct answer
- **difficulty**: Enum("beginner", "intermediate", "advanced")

### HardwareConfiguration
- **name**: String (required) - Name of the configuration (e.g., "Jetson Student Kit")
- **description**: String (required) - Description of the configuration
- **specifications**: JSON - Hardware specifications (CPU, GPU, RAM, etc.)
- **supportedModules**: Array[Module] - Modules that support this configuration
- **setupInstructions**: String - Instructions for setting up this configuration
- **limitations**: Array[String] - Known limitations of this configuration

### CodeExample
- **title**: String (required) - Title of the code example
- **description**: String (required) - Description of what the code does
- **language**: String (required) - Programming language (e.g., "Python", "C++")
- **code**: String (required) - The actual code
- **associatedLesson**: Lesson (foreign key) - The lesson this code example is for
- **expectedOutput**: String - What the code should produce when run
- **errorHandling**: String - How errors in the code are handled
- **modifications**: Array[String] - Suggested modifications for learning

### SimulationEnvironment
- **name**: String (required) - Name of the simulation environment (e.g., "Gazebo", "Unity")
- **version**: String - Version of the simulation software
- **description**: String (required) - Description of the simulation environment
- **supportedRobots**: Array[String] - List of robot models supported
- **physicsEngine**: String - Name of the physics engine used
- **setupInstructions**: String - Instructions for setting up the environment
- **tutorials**: Array[Lesson] - Associated tutorials for this environment
- **compatibility**: Array[HardwareConfiguration] - Compatible hardware configurations

### RobotPlatform
- **name**: String (required) - Name of the robot platform (e.g., "NAO", "Pepper", "Custom Humanoid")
- **description**: String (required) - Description of the robot platform
- **urdfPath**: String - Path to URDF file if available
- **supportedEnvironments**: Array[SimulationEnvironment] - Simulation environments supported
- **capabilities**: Array[String] - List of robot capabilities (e.g., "walking", "grasping", "speech")
- **sensors**: Array[String] - List of sensors available on the robot
- **actuators**: Array[String] - List of actuators available on the robot
- **tutorials**: Array[Lesson] - Associated lessons for this robot platform
- **compatibility**: Array[HardwareConfiguration] - Compatible hardware configurations

## Relationships

- Module `1-to-many` Lesson (one module contains multiple lessons)
- Module `1-to-many` Exercise (one module has multiple exercises)
- Lesson `1-to-many` Exercise (one lesson can have multiple exercises)
- Lesson `many-to-many` Asset (lessons use multiple assets, assets can be used by multiple lessons)
- Module `1-to-many` Assessment (one module has one assessment)
- Assessment `1-to-many` Question (one assessment has multiple questions)
- Module `many-to-many` HardwareConfiguration (modules can support multiple hardware configurations)
- Lesson `1-to-many` CodeExample (one lesson can have multiple code examples)
- Module `1-to-many` SimulationEnvironment (one module can use multiple simulation environments)
- SimulationEnvironment `many-to-many` RobotPlatform (simulation environments can support multiple robot platforms)
- HardwareConfiguration `many-to-many` RobotPlatform (hardware configurations can support multiple robot platforms)