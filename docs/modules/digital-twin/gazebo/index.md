---
title: Gazebo Simulation Guide
---

# Gazebo Simulation Guide

Gazebo is a powerful 3D simulation environment that provides realistic physics simulation, high-quality graphics, and convenient programmatic interfaces. This guide will help you set up and use Gazebo for digital twin applications.

## Learning Objectives

After completing this module, you will be able to:
- Install and configure Gazebo for robotics simulation
- Create and modify robot models for simulation
- Simulate robot behavior with realistic physics
- Integrate Gazebo with ROS 2 using gazebo_ros_pkgs

## Gazebo Installation

### For Ubuntu 22.04 with ROS 2 Humble
```bash
sudo apt update
# Install Gazebo (Fortress version is compatible with ROS 2 Humble)
sudo apt install gazebo libgazebo-dev
# Install ROS 2 Gazebo interfaces
sudo apt install ros-humble-gazebo-*
```

### For Windows (using WSL2)
- Install Ubuntu 22.04 on WSL2
- Follow the Ubuntu installation steps above
- Set up X11 forwarding for GUI applications

### For Jetson Platform
Gazebo can be resource-intensive on Jetson platforms. Consider using headless mode for heavy simulations:
```bash
gz sim -s
```

## Basic Gazebo Concepts

### Worlds
World files define the environment in which your robot will operate. They specify:
- Physical properties (gravity, atmosphere)
- Lighting conditions
- Static objects (ground planes, walls, etc.)
- Dynamic objects

### Models
Model files define the robots and objects that will interact in the world. They include:
- Visual mesh (how the object looks)
- Collision mesh (how the object physically interacts)
- Physical properties (mass, friction, etc.)
- Joints and actuators (for robots)

### Plugins
Gazebo plugins extend functionality and enable integration with external tools like ROS 2. Common plugins include:
- Joint controllers
- Sensor interfaces
- ROS 2 bridge plugins

## Creating Your First Simulation

### Simple Robot Model

1. Create a directory for your robot model:
```bash
mkdir -p ~/.gazebo/models/my_robot
```

2. Create the model configuration file `model.config`:
```xml
<?xml version="1.0"?>
<model>
  <name>My Robot</name>
  <version>1.0</version>
  <sdf version='1.6'>model.sdf</sdf>
  <author>
    <name>Your Name</name>
    <email>your.email@example.com</email>
  </author>
  <description>A simple robot for simulation.</description>
</model>
```

3. Create the model description file `model.sdf`:
```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="my_robot">
    <!-- Base link -->
    <link name="chassis">
      <pose>0 0 0.1 0 0 0</pose>
      <inertial>
        <mass>5.0</mass>
        <inertia>
          <ixx>0.1</ixx> <ixy>0.0</ixy> <ixz>0.0</ixz>
          <iyy>0.1</iyy> <iyz>0.0</iyz> <izz>0.1</izz>
        </inertia>
      </inertial>
      <visual name="chassis_visual">
        <geometry>
          <box><size>0.5 0.3 0.2</size></box>
        </geometry>
      </visual>
      <collision name="chassis_collision">
        <geometry>
          <box><size>0.5 0.3 0.2</size></box>
        </geometry>
      </collision>
    </link>

    <!-- Left wheel -->
    <link name="left_wheel">
      <inertial>
        <mass>0.5</mass>
        <inertia>
          <ixx>0.01</ixx> <ixy>0.0</ixy> <ixz>0.0</ixz>
          <iyy>0.01</iyy> <iyz>0.0</iyz> <izz>0.01</izz>
        </inertia>
      </inertial>
      <visual name="left_wheel_visual">
        <geometry>
          <cylinder><radius>0.1</radius><length>0.05</length></cylinder>
        </geometry>
      </visual>
      <collision name="left_wheel_collision">
        <geometry>
          <cylinder><radius>0.1</radius><length>0.05</length></cylinder>
        </geometry>
      </collision>
    </link>

    <!-- Right wheel -->
    <link name="right_wheel">
      <inertial>
        <mass>0.5</mass>
        <inertia>
          <ixx>0.01</ixx> <ixy>0.0</ixy> <ixz>0.0</ixz>
          <iyy>0.01</iyy> <iyz>0.0</iyz> <izz>0.01</izz>
        </inertia>
      </inertial>
      <visual name="right_wheel_visual">
        <geometry>
          <cylinder><radius>0.1</radius><length>0.05</length></cylinder>
        </geometry>
      </visual>
      <collision name="right_wheel_collision">
        <geometry>
          <cylinder><radius>0.1</radius><length>0.05</length></cylinder>
        </geometry>
      </collision>
    </link>

    <!-- Joints -->
    <joint name="left_wheel_hinge" type="continuous">
      <parent>chassis</parent>
      <child>left_wheel</child>
      <axis>
        <xyz>0 1 0</xyz>
      </axis>
      <pose>0.25 0.2 0 0 0 0</pose>
    </joint>

    <joint name="right_wheel_hinge" type="continuous">
      <parent>chassis</parent>
      <child>right_wheel</child>
      <axis>
        <xyz>0 1 0</xyz>
      </axis>
      <pose>0.25 -0.2 0 0 0 0</pose>
    </joint>
  </model>
</sdf>
```

### Running Your Simulation

Launch Gazebo with your world:
```bash
gz sim -r my_world.sdf
```

Or launch with ROS 2 integration:
```bash
# Terminal 1
ros2 launch gazebo_ros gazebo.launch.py

# Terminal 2 (after spawning your robot)
ros2 run gazebo_ros spawn_entity.py -entity my_robot -file ~/.gazebo/models/my_robot/model.sdf -x 0 -y 0 -z 0.1
```

## ROS 2 Integration

### Using gazebo_ros_pkgs

The `gazebo_ros_pkgs` package provides the bridge between Gazebo and ROS 2:

- `libgazebo_ros_factory.so`: Spawns models in Gazebo via ROS 2 topics/services
- `libgazebo_ros_joint_state_publisher.so`: Publishes joint states to ROS 2
- `libgazebo_ros_diff_drive.so`: Provides differential drive control

### Example Integration in URDF

To use this robot with ROS 2, you can include Gazebo plugins in your URDF:

```xml
<robot name="my_robot">
  <!-- Your URDF model definition here -->
  
  <!-- Gazebo plugins -->
  <gazebo>
    <plugin filename="libgazebo_ros_diff_drive.so" name="diff_drive">
      <left_joint>left_wheel_hinge</left_joint>
      <right_joint>right_wheel_hinge</right_joint>
      <wheel_separation>0.4</wheel_separation>
      <wheel_diameter>0.2</wheel_diameter>
      <command_topic>cmd_vel</command_topic>
      <odometry_topic>odom</odometry_topic>
      <odometry_frame>odom</odometry_frame>
      <robot_base_frame>chassis</robot_base_frame>
    </plugin>
  </gazebo>
</robot>
```

## Advanced Topics

### Using URDF with Gazebo

For more complex robots, it's often easier to use URDF and automatically convert to SDF:

1. Create your robot in URDF format
2. Use xacro to generate the SDF
3. Include Gazebo-specific tags in your URDF

### Sensors in Gazebo

Add various sensors to your robot model:
- Camera sensors
- LiDAR sensors
- IMU sensors
- Force/torque sensors

### Physics Tuning

Adjust physics parameters to match real robot behavior:
- Material properties (friction, restitution)
- Joint damping
- Control parameters

## Exercise

Create a model of a simple humanoid robot with at least 4 joints and spawn it in Gazebo. Add a camera sensor and publish the camera images to a ROS 2 topic.