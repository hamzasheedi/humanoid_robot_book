---
title: Digital Twin Simulation Exercises
---

# Digital Twin Simulation Exercises

These exercises reinforce the concepts of digital twin simulation using both Gazebo and Unity. Complete them to gain hands-on experience with creating and simulating robots.

## Exercise 1: Simple Robot in Gazebo

**Difficulty**: Beginner  
**Estimated Time**: 60 minutes

### Objective
Create a simple differential drive robot in Gazebo with ROS 2 integration.

### Instructions
1. Create a URDF file for a simple robot with:
   - A chassis (box shape)
   - Two wheels (cylinder shapes)
   - Proper joints connecting wheels to chassis
2. Create a Gazebo world file with:
   - A ground plane
   - Lighting setup
   - Basic obstacles
3. Integrate the robot with ROS 2 using appropriate Gazebo plugins
4. Test the robot by publishing velocity commands to `/cmd_vel`

### Expected Outcome
A robot that moves in response to velocity commands published via ROS 2.

### Solution
A reference implementation is available in the solutions section.

## Exercise 2: Robot in Unity

**Difficulty**: Beginner  
**Estimated Time**: 90 minutes

### Objective
Create a simple robot model in Unity and implement basic controls.

### Instructions
1. Create a Unity scene with a simple robot model:
   - Create base, wheels, and basic structure using primitive shapes
   - Add appropriate colliders and rigidbodies
2. Implement a controller script for differential drive movement
3. Add a camera component to simulate a camera sensor
4. Test the robot movement using keyboard controls

### Expected Outcome
A Unity scene with a controllable robot and camera view.

### Hints
- Use appropriate masses for realistic physics
- Consider the relationship between wheel rotation and linear movement
- Test your physics configuration with simple movements

## Exercise 3: Compare Gazebo vs Unity Simulation

**Difficulty**: Intermediate  
**Estimated Time**: 120 minutes

### Objective
Create the same robot in both Gazebo and Unity and compare the simulation characteristics.

### Instructions
1. Create an identical robot model in both Gazebo and Unity
2. Implement the same control algorithm for both simulations
3. Document differences in:
   - Physics behavior
   - Rendering quality
   - Performance characteristics
   - Ease of use for different applications
4. Create a simple navigation task that both robots must complete
5. Compare the results of the task in both simulators

### Expected Outcome
A detailed comparison of the simulation characteristics and recommendations for when to use each platform.

### Deliverables
- Report comparing both simulators
- Code for both implementations
- Video or screenshots showing the robot in both environments

## Exercise 4: Sensor Integration

**Difficulty**: Intermediate  
**Estimated Time**: 2 hours

### Objective
Add sensor simulation to your robot in either Gazebo or Unity.

### Instructions
1. Add a LiDAR sensor to your robot model
2. Configure the sensor parameters to simulate a real LiDAR (e.g., Hokuyo URG-04LX-UG01)
3. Process the sensor data in ROS 2
4. Use the sensor data to implement a simple obstacle avoidance behavior
5. Test the behavior in simulation

### Expected Outcome
A robot that can detect obstacles using simulated LiDAR and navigate around them.

### Advanced Option
Implement multiple sensor types (camera, LiDAR, IMU) and fuse the data for improved perception.

## Exercise 5: Humanoid Robot Simulation

**Difficulty**: Advanced  
**Estimated Time**: 4 hours

### Objective
Create a humanoid robot model using the provided URDF template and simulate it in Gazebo.

### Instructions
1. Start with the simple humanoid URDF template provided in the assets
2. Enhance the model by adding:
   - More detailed links and joints
   - Correct inertial properties
   - Realistic joint limits
3. Create a controller to make the robot walk using inverse kinematics
4. Implement balance control to keep the robot stable
5. Test the robot in a Gazebo environment with obstacles

### Expected Outcome
A simulated humanoid robot capable of basic walking movements.

### Resources
- Review concepts of static and dynamic walking
- Research inverse kinematics approaches for humanoid robots
- Look into center of mass control for balance

## Additional Resources

- Gazebo tutorials: http://gazebosim.org/tutorials
- Unity robotics package: https://github.com/Unity-Technologies/Unity-Robotics-Hub
- URDF tutorials: http://wiki.ros.org/urdf/Tutorials
- Physics simulation best practices for robotics