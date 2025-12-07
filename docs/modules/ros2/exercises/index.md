---
title: ROS 2 Exercises
---

# ROS 2 Exercises

These exercises are designed to reinforce the concepts learned in the ROS 2 fundamentals module. Complete them in order to build your understanding.

## Exercise 1: Basic Publisher/Subscriber

**Difficulty**: Beginner  
**Estimated Time**: 30 minutes

### Objective
Create a simple publisher that sends the current time and a subscriber that receives and displays it.

### Instructions
1. Create a new ROS 2 package called `time_talker`
2. Implement a publisher node that publishes the current time every 2 seconds as a string
3. Implement a subscriber node that listens to the time messages and prints them to the console
4. Test that both nodes work together using `ros2 run`

### Expected Outcome
The subscriber should print the time sent by the publisher with a timestamp of when it was received.

### Solution
A reference implementation is available in the solutions section.

## Exercise 2: Custom Message Type

**Difficulty**: Beginner  
**Estimated Time**: 45 minutes

### Objective
Create a custom message type for sending robot status information.

### Instructions
1. Define a custom message type called `RobotStatus.msg` with fields for:
   - robot_id (string)
   - battery_level (float32)
   - is_moving (bool)
2. Create a publisher that sends robot status messages
3. Create a subscriber that receives and displays these messages
4. Test the communication between nodes

### Expected Outcome
Successfully send and receive custom message types between ROS 2 nodes.

## Exercise 3: Services Introduction

**Difficulty**: Beginner  
**Estimated Time**: 45 minutes

### Objective
Implement a simple service that provides robot information on request.

### Instructions
1. Create a new package called `robot_service`
2. Define a service interface for requesting robot information
3. Implement a service server that responds to requests with robot status
4. Implement a service client that sends requests and displays responses
5. Test the service functionality

### Expected Outcome
The client should be able to request information from the server and receive a response.

### Hints
- Look up how to create custom service types in ROS 2
- Use `ros2 service` command-line tools to test your service

## Additional Resources

- ROS 2 tutorials: https://docs.ros.org/en/humble/Tutorials.html
- ROS 2 concepts: https://docs.ros.org/en/humble/Concepts/About-ROS-2-Client-Libraries.html
- Package management: https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html