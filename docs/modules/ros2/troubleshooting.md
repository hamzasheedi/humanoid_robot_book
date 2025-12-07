---
title: ROS 2 Troubleshooting Guide
---

# ROS 2 Troubleshooting Guide

This guide provides solutions to common issues encountered when working with ROS 2. Refer to this guide when you encounter problems during the tutorials or exercises.

## Common Setup Issues

### Installation Problems
**Problem**: "Unable to locate package ros-humble-desktop"
- **Solution**: Verify your Ubuntu version is 22.04. Check that you've properly added the ROS repository and updated your package list.

**Problem**: "Could not find a package configuration file"
- **Solution**: Ensure you've sourced your ROS 2 installation: `source /opt/ros/humble/setup.bash`

### Environment Issues
**Problem**: Commands like `ros2` are not found
- **Solution**: Add the ROS 2 sourcing command to your `.bashrc` file:
  ```bash
  echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
  source ~/.bashrc
  ```

## Communication Issues

### Nodes Not Communicating
**Problem**: Publisher and subscriber nodes are not communicating
- **Solution**: 
  1. Verify both nodes are on the same ROS domain ID
  2. Confirm both terminals have sourced the ROS environment
  3. Check that the topic names match exactly (including case sensitivity)
  4. Use `ros2 topic list` and `ros2 node list` to verify nodes are running

**Problem**: "Unable to load plugin" error when using ros2 command
- **Solution**: Install missing packages: `sudo apt install python3-ros-environment python3-ros-workspace`

## Package and Workspace Issues

### Building Packages
**Problem**: `colcon build` fails with compilation errors
- **Solution**:
  1. Verify your package.xml has correct dependencies
  2. Check that setup.py is properly configured
  3. Ensure all import statements in Python files are correct
  4. Verify CMakeLists.txt for C++ packages

**Problem**: Package not found when running `ros2 run`
- **Solution**:
  1. Make sure you built the package: `colcon build`
  2. Source the workspace: `source install/setup.bash`
  3. Check that you're in the workspace root directory

## Network Issues

### Multi-Machine Communication
**Problem**: Nodes on different machines cannot communicate
- **Solution**:
  1. Ensure ROS_DOMAIN_ID is the same on both machines
  2. Check firewall settings to allow ROS 2 traffic
  3. Verify network connectivity between machines using `ping`
  4. Set ROS_LOCALHOST_ONLY=0 if not using localhost

## Performance Issues

### Slow Performance
**Problem**: ROS 2 nodes running slowly or with high latency
- **Solution**:
  1. Check system resources (CPU, RAM usage)
  2. Reduce the frequency of high-bandwidth message publishing
  3. Consider using QoS settings for performance optimization

## Debugging Tools

### Useful Commands
- List all active topics: `ros2 topic list`
- Echo messages on a topic: `ros2 topic echo <topic_name> <msg_type>`
- List all active nodes: `ros2 node list`
- Get info about a node: `ros2 node info <node_name>`
- List available services: `ros2 service list`

### Using rqt
For visualization and debugging, install and use rqt tools:
```bash
sudo apt install ros-humble-rqt ros-humble-rqt-common-plugins
rqt
```

## Getting Help

If you encounter an issue not covered here:

1. Check the official ROS 2 documentation: https://docs.ros.org/en/humble/
2. Search the ROS Answers forum: https://answers.ros.org/questions/
3. Check GitHub repositories for known issues
4. Ask for help in robotics communities or forums specific to your setup

## Hardware-Specific Issues

### Jetson Platform
- Ensure sufficient power supply for Jetson board
- Monitor thermal throttling with `sudo tegrastats`
- Use appropriate compute mode for performance needs

### Cloud/Remote Development
- Be aware of latency when testing real-time systems
- Ensure adequate bandwidth for visualization tools
- Consider using headless mode when GUI is not needed