---
title: Unity Simulation Guide
---

# Unity Simulation Guide

Unity is a powerful cross-platform game engine that can be used for creating digital twins and robotic simulations with high-quality graphics and physics. This guide will help you set up and use Unity for robotics simulation.

## Learning Objectives

After completing this module, you will be able to:
- Install and configure Unity for robotics simulation
- Create and modify robot models in Unity
- Set up realistic physics environments
- Integrate Unity with ROS 2 using ROS# or Unity Robotics Package

## Unity Installation

### Unity Hub Setup
1. Download and install Unity Hub from https://unity.com/download
2. Create a Unity account if you don't have one
3. Through Unity Hub, install Unity Editor version 2022.3 LTS (recommended for stability)

### Required Packages
Install these packages through Unity Package Manager:
- Physics package for realistic physics simulation
- XR packages if you plan to use VR/AR
- Unity Robotics Package (if available)

## Unity Concepts for Robotics

### GameObjects and Components
- **GameObjects**: The basic objects in a Unity scene
- **Components**: Scripts, meshes, physics properties, and other features attached to GameObjects
- **Transforms**: Position, rotation, and scale of objects

### Physics Engine
Unity's physics engine includes:
- Collision detection
- Rigid body dynamics
- Joint systems
- Material properties for friction and bounciness

### Scripting
Unity uses C# for scripting, which allows you to:
- Control robot movements
- Handle sensor data
- Implement AI behaviors

## Creating Your First Robot Simulation

### Basic Robot Setup

1. Create a new 3D project in Unity Hub
2. Create a basic robot structure:
   - Create an empty GameObject as the robot root
   - Add child GameObjects for each robot part (body, wheels, arms, etc.)
   - Add 3D models or primitive shapes (cubes, cylinders) for each part

3. Add physics properties:
   - Add Rigidbody components to parts that need physics simulation
   - Add Collider components to define collision shapes
   - Create Joints to connect robot parts

### Example Robot with Differential Drive

Here's a simple C# script to control a differential drive robot:

```csharp
using UnityEngine;

public class DifferentialDriveController : MonoBehaviour
{
    public float maxSpeed = 5.0f;
    public float maxAngularVelocity = 2.0f;
    
    public Transform leftWheel;
    public Transform rightWheel;
    
    private float linearVelocity = 0f;
    private float angularVelocity = 0f;
    
    void Start()
    {
        // Initialize any required components
    }
    
    void Update()
    {
        // In a real implementation, these would come from ROS 2
        // For now, we'll use keyboard input for demonstration
        ProcessInput();
        MoveRobot();
    }
    
    void ProcessInput()
    {
        // This is temporary - in a real implementation, these values would come from ROS 2
        linearVelocity = Input.GetAxis("Vertical") * maxSpeed;
        angularVelocity = -Input.GetAxis("Horizontal") * maxAngularVelocity;
    }
    
    void MoveRobot()
    {
        // Calculate wheel velocities based on differential drive kinematics
        float wheelRadius = 0.1f;  // Adjust to your robot's wheel radius
        float wheelSeparation = 0.4f;  // Adjust to your robot's wheel separation
        
        float leftWheelVel = (linearVelocity - angularVelocity * wheelSeparation / 2) / wheelRadius;
        float rightWheelVel = (linearVelocity + angularVelocity * wheelSeparation / 2) / wheelRadius;
        
        // Rotate wheels
        if (leftWheel != null)
        {
            leftWheel.Rotate(Vector3.right, leftWheelVel * Mathf.Rad2Deg * Time.deltaTime);
        }
        
        if (rightWheel != null)
        {
            rightWheel.Rotate(Vector3.right, rightWheelVel * Mathf.Rad2Deg * Time.deltaTime);
        }
        
        // Update robot position (simplified - for full physics simulation, use Rigidbody)
        transform.Translate(Vector3.forward * linearVelocity * Time.deltaTime);
        transform.Rotate(Vector3.up, angularVelocity * Mathf.Rad2Deg * Time.deltaTime);
    }
    
    // In a real implementation, you would use ROS 2 message callbacks instead of Update()
    public void SetVelocity(float linear, float angular)
    {
        linearVelocity = linear;
        angularVelocity = angular;
    }
}
```

### Adding Sensors

Unity can simulate various sensors:

#### Camera Sensor
1. Add a Camera component to your robot
2. Configure the camera properties (field of view, resolution)
3. Capture images using Unity's image utilities

#### LiDAR Simulation
1. Use raycasting to simulate LiDAR data
2. Create a script that sends multiple rays in a fan pattern
3. Calculate distances to obstacles

### Physics Configuration

For realistic robot simulation:
1. Set appropriate masses for each robot part
2. Configure drag and angular drag
3. Set up joints with appropriate constraints
4. Calibrate physics materials for realistic friction

## ROS 2 Integration

### Using Unity Robotics Package
Unity provides the Unity Robotics Package for ROS 2 integration:

1. Install the Unity Robotics Package
2. Use the ROS-TCP-Connector for communication
3. Implement ROS message publishers and subscribers in C#

### Example Communication Setup

```csharp
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;

public class UnityRobotController : MonoBehaviour
{
    ROSConnection ros;
    string robotName = "unity_robot";
    
    // Subscribe to ROS topics
    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterSubscriber<TwistMsg>("/cmd_vel");
    }
    
    void OnMessageReceived(TwistMsg cmd_vel)
    {
        // Process the ROS message and update robot movement
        float linear = (float)cmd_vel.linear.x;
        float angular = (float)cmd_vel.angular.z;
        
        // Update your robot controller with these values
        SetVelocity(linear, angular);
    }
    
    void Update()
    {
        // Publish sensor data back to ROS
        if (Time.time % 0.1f < Time.deltaTime) // Publish every 0.1 seconds
        {
            // Create and publish a sensor message
            var odom_msg = new OdometryMsg();
            odom_msg.header = new HeaderMsg();
            odom_msg.header.stamp = new TimeStamp(ros.Clock.Now);
            odom_msg.header.frame_id = "odom";
            
            // Fill in position and orientation
            odom_msg.pose.pose.position = new Vector3Msg(transform.position.x, 
                                                         transform.position.y, 
                                                         transform.position.z);
            odom_msg.pose.pose.orientation = new QuaternionMsg(
                transform.rotation.x, 
                transform.rotation.y, 
                transform.rotation.z, 
                transform.rotation.w);
            
            // Publish the message
            ros.Send<OdometryMsg>("/odom", odom_msg);
        }
    }
}
```

## Comparison: Unity vs Gazebo

### Unity Advantages:
- Superior graphics and visualization
- Cross-platform deployment
- Large community and assets
- Game development features (VR/AR support)

### Gazebo Advantages:
- Better physics accuracy for robotics
- Native ROS integration
- Robot-specific tools and models
- Lightweight compared to Unity

## Performance Considerations

### Unity Performance
- Unity is more resource-intensive than Gazebo
- Consider using lower-resolution models for better performance
- Optimize rendering settings for real-time simulation
- Use occlusion culling and other optimization techniques

### Hardware Requirements
- Unity requires more powerful GPUs than Gazebo
- Gazebo can run on Jetson platforms; Unity typically requires more powerful hardware

## Exercise

Create a Unity scene with a simple robot model and implement differential drive control. Add a camera sensor to the robot and configure it to publish images to a ROS 2 topic using the Unity-ROS integration.

## Troubleshooting

### Common Issues
- Physics artifacts: Check mass properties and collision shapes
- Slow performance: Reduce polygon counts, optimize rendering
- ROS connection failures: Ensure TCP connections are not blocked by firewalls
- Coordinate system mismatch: Unity uses left-handed coordinates; ROS uses right-handed

### Debugging Tips
- Use Unity's built-in Profiler to find performance bottlenecks
- Visualize physics colliders to ensure proper collision detection
- Monitor ROS topics to ensure proper message flow