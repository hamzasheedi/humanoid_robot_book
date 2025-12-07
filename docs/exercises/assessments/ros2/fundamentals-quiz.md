---
title: ROS 2 Fundamentals Assessment
---

# ROS 2 Fundamentals Assessment

This assessment tests understanding of ROS 2 (Robot Operating System 2) core concepts covered in Module 1.

## Quiz 1: ROS 2 Architecture

1. **What is a ROS 2 node?**
   a) A file containing robot code
   b) A process that performs computation and communication in ROS 2
   c) A type of sensor
   d) A hardware component

   **Answer:** b) A process that performs computation and communication in ROS 2

2. **Which of the following is NOT a valid ROS 2 communication pattern?**
   a) Publisher/Subscriber
   b) Service/Client
   c) Action Server/Action Client
   d) Function/Procedure

   **Answer:** d) Function/Procedure

3. **What is the purpose of a topic in ROS 2?**
   a) To store robot configuration files
   b) To provide a synchronous request/reply interface
   c) To enable asynchronous communication between nodes via messages
   d) To define robot geometry

   **Answer:** c) To enable asynchronous communication between nodes via messages

## Quiz 2: ROS 2 Messages and Services

4. **Which command lists all active ROS 2 topics?**
   a) `ros2 topics list`
   b) `rostopic list`
   c) `ros2 topic list`
   d) `ros2 list topics`

   **Answer:** c) `ros2 topic list`

5. **What is the difference between a ROS 2 service and a ROS 2 topic?**
   a) Topics are for hardware only, services are for software
   b) Services provide asynchronous communication, topics provide synchronous communication
   c) Topics provide asynchronous communication, services provide synchronous request/reply communication
   d) There is no difference

   **Answer:** c) Topics provide asynchronous communication, services provide synchronous request/reply communication

## Practical Assessment

6. **Create a simple publisher in Python that publishes "Hello World" to a topic called "chatter" with a frequency of 1 Hz.**

   **Sample Solution:**
   ```python
   import rclpy
   from rclpy.node import Node
   from std_msgs.msg import String


   class MinimalPublisher(Node):

       def __init__(self):
           super().__init__('minimal_publisher')
           self.publisher_ = self.create_publisher(String, 'chatter', 10)
           timer_period = 1.0  # seconds
           self.timer = self.create_timer(timer_period, self.timer_callback)

       def timer_callback(self):
           msg = String()
           msg.data = 'Hello World'
           self.publisher_.publish(msg)
           self.get_logger().info('Publishing: "%s"' % msg.data)


   def main(args=None):
       rclpy.init(args=args)

       minimal_publisher = MinimalPublisher()

       rclpy.spin(minimal_publisher)

       # Destroy the node explicitly
       minimal_publisher.destroy_node()
       rclpy.shutdown()


   if __name__ == '__main__':
       main()
   ```

7. **What would be the correct command to echo messages from the "chatter" topic?**
   a) `ros2 topic echo chatter std_msgs/msg/String`
   b) `ros2 echo chatter`
   c) `ros2 topic read chatter`
   d) `rostopic echo chatter`

   **Answer:** a) `ros2 topic echo chatter std_msgs/msg/String`

## Advanced Questions

8. **What is the purpose of rclpy?**
   a) A C++ client library for ROS 2
   b) A Python client library for ROS 2
   c) A hardware driver for ROS 2
   d) A visualization tool for ROS 2

   **Answer:** b) A Python client library for ROS 2

9. **What is a launch file in ROS 2?**
   a) A file to start your computer
   b) A file that defines parameters for robot hardware
   c) An XML or Python file that defines how to start multiple nodes together
   d) A file to store robot maps

   **Answer:** c) An XML or Python file that defines how to start multiple nodes together

10. **Explain the purpose of the DDS (Data Distribution Service) in ROS 2.**

    **Sample Answer:** DDS acts as the middleware layer in ROS 2, providing a communication protocol that enables discovery, data transmission, and quality-of-service features between nodes. It handles the underlying network communication, enabling nodes to find each other and exchange data reliably.

## Scoring
- Questions 1-5: 2 points each
- Question 6: 5 points (correct implementation with proper structure)
- Question 7: 2 points
- Questions 8-9: 2 points each
- Question 10: 5 points (for comprehensive explanation)

**Total Points: 25**

**Passing Score: 18/25 (72%)**