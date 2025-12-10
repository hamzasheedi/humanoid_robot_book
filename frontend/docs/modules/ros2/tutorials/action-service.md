---
title: Action Service Tutorial
---

# Action Service Tutorial

In ROS 2, actions are used for long-running tasks that provide feedback and can be canceled. This tutorial will walk you through creating and using an action server and client.

## Learning Objectives

After completing this tutorial, you will be able to:
- Define custom action messages
- Create an action server that performs long-running tasks
- Create an action client that communicates with the server
- Understand when to use actions vs services vs topics

## Action Definition

First, let's define a custom action for moving a robot to a goal position. Create a file called `MoveRobot.action`:

```
# Define the goal (input to the action)
float64 x
float64 y
float64 theta

---
# Define the result (output from the action)
bool success
string message

---
# Define the feedback (ongoing updates during execution)
float64 distance_to_goal
string status
```

## Action Server

Create a new file called `move_robot_server.py`:

```python
import time
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node

# You'll need to define your action message or use a standard one
# For this example, we'll use a fake action
from example_interfaces.action import Fibonacci


class MoveRobotActionServer(Node):

    def __init__(self):
        super().__init__('move_robot_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,  # In practice, you would use your custom action
            'move_robot',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback)

    def destroy(self):
        self._action_server.destroy()
        super().destroy_node()

    def goal_callback(self, goal_request):
        """Accept or reject a client request to begin an action."""
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Accept or reject a client request to cancel an action."""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        """Execute the goal."""
        self.get_logger().info('Executing goal...')

        # Simulate robot movement
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.sequence = [0, 1]

        for i in range(1, 10):
            # Check if there's a cancel request
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return Fibonacci.Result()

            # Update feedback
            feedback_msg.sequence.append(
                feedback_msg.sequence[i] + feedback_msg.sequence[i-1])
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(f'Feedback: {feedback_msg.sequence}')

            # Simulate work
            time.sleep(1)

        # Check if goal was canceled
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            self.get_logger().info('Goal canceled')
            return Fibonacci.Result()

        # Goal completed successfully
        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = feedback_msg.sequence
        return result


def main(args=None):
    rclpy.init(args=args)
    action_server = MoveRobotActionServer()

    try:
        rclpy.spin(action_server)
    finally:
        action_server.destroy()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Action Client

Create a new file called `move_robot_client.py`:

```python
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

# Using Fibonacci as an example - you'd define your custom action
from example_interfaces.action import Fibonacci


class MoveRobotActionClient(Node):

    def __init__(self):
        super().__init__('move_robot_action_client')
        self._action_client = ActionClient(
            self,
            Fibonacci,
            'move_robot')

    def send_goal(self):
        goal_msg = Fibonacci.Goal()
        goal_msg.order = 10  # Example order

        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        self.get_logger().info('Sending goal request...')

        # Send the goal and get a future
        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        # Set up a callback for when the future is complete (goal is accepted)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        # Get the result future
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result.sequence}')
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {feedback.sequence}')


def main(args=None):
    rclpy.init(args=args)
    action_client = MoveRobotActionClient()

    action_client.send_goal()

    rclpy.spin(action_client)


if __name__ == '__main__':
    main()
```

## Testing the Action

1. Source your ROS 2 environment:
   ```bash
   source /opt/ros/humble/setup.bash
   ```

2. Run the action server in one terminal:
   ```bash
   ros2 run py_pubsub move_robot_server
   ```

3. In another terminal, run the action client:
   ```bash
   ros2 run py_pubsub move_robot_client
   ```

## When to Use Actions

- **Topics**: For continuous data streams (sensors, joint states)
- **Services**: For request/reply interactions that complete quickly
- **Actions**: For long-running tasks that need feedback, can be canceled, and have clear success/failure states

## Exercise

Modify the action server to simulate a real robot movement where the feedback represents the actual distance to the goal, decreasing as the robot moves toward it.

## Advanced Topics

- Goal preemption (handling new goals while executing)
- Timeout handling
- Multiple concurrent goals