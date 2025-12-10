---
title: Publisher/Subscriber Tutorial
---

# Publisher/Subscriber Tutorial

In ROS 2, the publisher/subscriber pattern is fundamental for communication between nodes. Publishers send messages to topics, and subscribers receive messages from topics. This tutorial will guide you through creating a simple publisher and subscriber.

## Learning Objectives

After completing this tutorial, you will be able to:
- Create a ROS 2 publisher node
- Create a ROS 2 subscriber node
- Understand the pub/sub communication model
- Test your nodes with ROS 2 tools

## Publisher Node

Create a new file called `talker.py` in your package:

```python
import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Subscriber Node

Create a new file called `listener.py` in your package:

```python
import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Package Structure

To run these nodes, you'll need to create a proper package structure:

```
ros2_tutorials/
├── src/
│   └── py_pubsub/
│       ├── py_pubsub/
│       ├── package.xml
│       ├── setup.cfg
│       ├── setup.py
│       └── CMakeLists.txt
```

## Testing Your Nodes

1. Source your ROS 2 environment:
   ```bash
   source /opt/ros/humble/setup.bash
   ```

2. Navigate to your workspace and build:
   ```bash
   cd ros2_tutorials
   colcon build
   source install/setup.bash
   ```

3. Run the publisher in one terminal:
   ```bash
   ros2 run py_pubsub talker
   ```

4. In another terminal, run the subscriber:
   ```bash
   ros2 run py_pubsub listener
   ```

You should see the publisher sending messages and the subscriber receiving them.

## Expected Output

Publisher output:
```
[INFO] [1612051200.123456789] [minimal_publisher]: Publishing: "Hello World: 0"
[INFO] [1612051200.623456789] [minimal_publisher]: Publishing: "Hello World: 1"
```

Subscriber output:
```
[INFO] [1612051200.373456789] [minimal_subscriber]: I heard: "Hello World: 0"
[INFO] [1612051200.873456789] [minimal_subscriber]: I heard: "Hello World: 1"
```

## Exercise

Modify the publisher to send a different message every few counts (e.g., "Counter: 1", "Counter: 2", etc.) and verify that the subscriber receives the modified messages.

## Error Handling

Common issues you might encounter:
- Nodes don't connect: Check that both terminals have sourced the ROS 2 environment
- Package not found: Ensure you've built your workspace with `colcon build`
- Permission errors: Check that your ROS 2 installation has proper permissions