---
sidebar_position: 3
title: 'Python AI Agents with rclpy'
---

# Python AI Agents with rclpy

This chapter demonstrates how to implement Python-based AI agents that can create ROS 2 nodes, publish and subscribe to control signals, and bridge AI decision-making to robot controllers using the rclpy client library.

import RosConcept from '@site/src/components/RosConcept';
import CodeExample from '@site/src/components/CodeExample';

<RosConcept
  title="rclpy"
  type="node"
  description="rclpy is the Python client library for ROS 2, providing Python bindings for the ROS 2 client library (rcl)."
  usage="It enables Python developers to create ROS 2 nodes, publish and subscribe to topics, provide services, and interact with actions."
/>

## Introduction to rclpy

rclpy is the Python client library for ROS 2. It provides Python bindings for the ROS 2 client library (rcl). This enables Python developers to create ROS 2 nodes, publish and subscribe to topics, provide services, and interact with actions.

Python is an excellent choice for AI agents because:

- **Rapid Prototyping**: Python allows for quick development and testing of AI algorithms
- **Rich Ecosystem**: Access to powerful AI libraries like TensorFlow, PyTorch, and scikit-learn
- **Readability**: Python code is easy to understand and maintain
- **Community Support**: Large community of Python developers in the AI field

## Setting up a Python ROS 2 Environment

Before creating AI agents, ensure your Python environment is properly configured for ROS 2:

```bash
# Source ROS 2 installation
source /opt/ros/humble/setup.bash  # Replace 'humble' with your ROS 2 distribution

# Create a Python virtual environment
python3 -m venv ros2_env
source ros2_env/bin/activate
pip install rclpy
```

## Creating a Basic ROS 2 Node in Python

A ROS 2 node in Python is created by inheriting from the `Node` class:

<CodeExample
  title="Basic ROS 2 Node in Python"
  description="A simple ROS 2 node that initializes and runs until shut down."
  code={`import rclpy
from rclpy.node import Node

class BasicAIAgent(Node):
    def __init__(self):
        super().__init__('basic_ai_agent')
        self.get_logger().info('Basic AI Agent node initialized')

def main(args=None):
    rclpy.init(args=args)
    ai_agent = BasicAIAgent()

    try:
        rclpy.spin(ai_agent)
    except KeyboardInterrupt:
        pass
    finally:
        ai_agent.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()`}
  language="python"
/>

## Publishing Control Signals

AI agents often need to publish control signals to robot controllers:

<CodeExample
  title="Control Publisher Node"
  description="A node that publishes control commands to a robot at regular intervals."
  code={`from std_msgs.msg import String
import rclpy
from rclpy.node import Node

class ControlPublisher(Node):
    def __init__(self):
        super().__init__('control_publisher')
        self.publisher = self.create_publisher(String, 'robot_control', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = 'Control command at: %d' % self.get_clock().now().nanoseconds
        self.publisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    control_publisher = ControlPublisher()

    try:
        rclpy.spin(control_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        control_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()`}
  language="python"
/>

## Subscribing to Sensor Data

AI agents also need to subscribe to sensor data to make informed decisions:

<CodeExample
  title="Sensor Subscriber Node"
  description="A node that subscribes to sensor data and processes it."
  code={`from std_msgs.msg import String
import rclpy
from rclpy.node import Node

class SensorSubscriber(Node):
    def __init__(self):
        super().__init__('sensor_subscriber')
        self.subscription = self.create_subscription(
            String,
            'sensor_data',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('Received sensor data: "%s"' % msg.data)
        # Process sensor data and make AI decisions here

def main(args=None):
    rclpy.init(args=args)
    sensor_subscriber = SensorSubscriber()

    try:
        rclpy.spin(sensor_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        sensor_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()`}
  language="python"
/>

## Bridging AI Decisions to Robot Control

A more complete example that bridges AI decision-making to robot control:

<CodeExample
  title="AI Bridge Node"
  description="A node that processes sensor data and makes AI decisions to control a robot."
  code={`import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import random

class AIBridgeNode(Node):
    def __init__(self):
        super().__init__('ai_bridge_node')

        # Publisher for control commands
        self.control_publisher = self.create_publisher(String, 'robot_control', 10)

        # Subscriber for sensor data
        self.sensor_subscriber = self.create_subscription(
            String,
            'sensor_data',
            self.sensor_callback,
            10)

        # Timer for AI decision making
        self.timer = self.create_timer(1.0, self.ai_decision_loop)

        self.get_logger().info('AI Bridge Node initialized')

    def sensor_callback(self, msg):
        self.get_logger().info(f'Received sensor data: {msg.data}')
        # Store sensor data for AI processing
        self.last_sensor_data = msg.data

    def ai_decision_loop(self):
        # Simple AI decision making
        # In a real implementation, this would use ML models or complex logic
        if hasattr(self, 'last_sensor_data'):
            ai_decision = self.make_decision(self.last_sensor_data)
            self.publish_control_command(ai_decision)

    def make_decision(self, sensor_data):
        # Placeholder for AI decision making logic
        # This could involve neural networks, rule-based systems, etc.
        possible_decisions = [
            "MOVE_FORWARD",
            "TURN_LEFT",
            "TURN_RIGHT",
            "STOP",
            "EXPLORE"
        ]
        return random.choice(possible_decisions)

    def publish_control_command(self, decision):
        msg = String()
        msg.data = decision
        self.control_publisher.publish(msg)
        self.get_logger().info(f'Published control command: {decision}')

def main(args=None):
    rclpy.init(args=args)
    ai_bridge = AIBridgeNode()

    try:
        rclpy.spin(ai_bridge)
    except KeyboardInterrupt:
        pass
    finally:
        ai_bridge.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()`}
  language="python"
/>

## Common AI-to-Control Patterns

When building AI agents that control robots, several common patterns appear. Here are examples of how to implement these patterns:

### Pattern 1: State Machine Controller

<CodeExample
  title="State Machine Controller"
  description="A node that implements a simple state machine to control robot behavior based on sensor input."
  code={`import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class StateMachineController(Node):
    def __init__(self):
        super().__init__('state_machine_controller')

        # Publisher for robot velocity commands
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Subscriber for sensor data
        self.sensor_subscription = self.create_subscription(
            String, 'sensor_data', self.sensor_callback, 10)

        # Timer for state machine updates
        self.timer = self.create_timer(0.1, self.state_machine_update)

        # Initial state
        self.state = 'SEARCHING'  # SEARCHING, APPROACHING, AVOIDING
        self.last_sensor_data = ''

        self.get_logger().info('State Machine Controller initialized')

    def sensor_callback(self, msg):
        self.last_sensor_data = msg.data

    def state_machine_update(self):
        # Parse sensor data (simplified example)
        sensor_value = 0.0
        if self.last_sensor_data:
            try:
                sensor_value = float(self.last_sensor_data.split(':')[-1])
            except ValueError:
                pass

        # State transition logic
        if self.state == 'SEARCHING':
            if sensor_value < 1.0:  # Object detected nearby
                self.state = 'APPROACHING'
            else:
                self.rotate_in_place()
        elif self.state == 'APPROACHING':
            if sensor_value < 0.5:  # Too close
                self.state = 'AVOIDING'
            elif sensor_value > 1.5:  # Too far
                self.state = 'SEARCHING'
            else:
                self.move_forward()
        elif self.state == 'AVOIDING':
            if sensor_value > 1.0:  # Safe distance
                self.state = 'SEARCHING'
            else:
                self.move_backward()

    def rotate_in_place(self):
        cmd = Twist()
        cmd.angular.z = 0.5  # Rotate at 0.5 rad/s
        self.cmd_vel_publisher.publish(cmd)

    def move_forward(self):
        cmd = Twist()
        cmd.linear.x = 0.3  # Move forward at 0.3 m/s
        self.cmd_vel_publisher.publish(cmd)

    def move_backward(self):
        cmd = Twist()
        cmd.linear.x = -0.2  # Move backward at 0.2 m/s
        self.cmd_vel_publisher.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    controller = StateMachineController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()`}
  language="python"
/>

### Pattern 2: PID Controller with AI Override

<CodeExample
  title="PID Controller with AI Override"
  description="A node that implements a PID controller with AI-based parameter adjustment."
  code={`import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
import time

class PIDControllerWithAI(Node):
    def __init__(self):
        super().__init__('pid_controller_with_ai')

        # Publisher for velocity commands
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Subscribers for target and current values
        self.target_subscription = self.create_subscription(
            Float64, 'target_value', self.target_callback, 10)
        self.current_subscription = self.create_subscription(
            Float64, 'current_value', self.current_callback, 10)

        # Timer for control loop
        self.timer = self.create_timer(0.05, self.control_loop)  # 20 Hz

        # PID parameters
        self.kp = 1.0
        self.ki = 0.1
        self.kd = 0.05

        # PID internal variables
        self.previous_error = 0.0
        self.integral = 0.0
        self.target_value = 0.0
        self.current_value = 0.0
        self.last_time = self.get_clock().now()

        # AI adjustment parameters
        self.ai_adjustment_factor = 1.0

        self.get_logger().info('PID Controller with AI initialized')

    def target_callback(self, msg):
        self.target_value = msg.data

    def current_callback(self, msg):
        self.current_value = msg.data

    def control_loop(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9  # Convert to seconds
        self.last_time = current_time

        if dt <= 0:
            return

        # Calculate error
        error = self.target_value - self.current_value

        # Update integral
        self.integral += error * dt

        # Calculate derivative
        derivative = (error - self.previous_error) / dt

        # Calculate PID output
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)

        # Apply AI-based adjustment (simplified example)
        # In a real system, this could adjust PID parameters based on system behavior
        adjusted_output = output * self.ai_adjustment_factor

        # Create and publish command
        cmd = Twist()
        cmd.linear.x = max(-1.0, min(1.0, adjusted_output))  # Clamp to reasonable range
        self.cmd_vel_publisher.publish(cmd)

        # Update previous error
        self.previous_error = error

        # Simple AI adjustment based on error magnitude
        if abs(error) > 1.0:
            self.ai_adjustment_factor = 1.1  # Increase responsiveness for large errors
        elif abs(error) < 0.1:
            self.ai_adjustment_factor = 0.9  # Reduce for fine control
        else:
            self.ai_adjustment_factor = 1.0  # Normal operation

def main(args=None):
    rclpy.init(args=args)
    controller = PIDControllerWithAI()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()`}
  language="python"
/>

### Pattern 3: Behavior-Based Controller

<CodeExample
  title="Behavior-Based Controller"
  description="A node that implements multiple behaviors and arbitrates between them."
  code={`import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

class BehaviorBasedController(Node):
    def __init__(self):
        super().__init__('behavior_based_controller')

        # Publisher for velocity commands
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Subscriber for laser scan data
        self.scan_subscription = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10)

        # Timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)

        # Robot state
        self.scan_data = None
        self.min_distance = float('inf')
        self.min_angle = 0

        self.get_logger().info('Behavior-Based Controller initialized')

    def scan_callback(self, msg):
        self.scan_data = msg
        # Find closest obstacle
        if msg.ranges:
            min_idx = 0
            min_val = float('inf')
            for i, dist in enumerate(msg.ranges):
                if dist < min_val and not math.isnan(dist):
                    min_val = dist
                    min_idx = i
            self.min_distance = min_val
            self.min_angle = msg.angle_min + min_idx * msg.angle_increment

    def control_loop(self):
        if self.scan_data is None:
            return

        # Calculate behaviors
        avoid_behavior = self.avoid_obstacles_behavior()
        explore_behavior = self.explore_behavior()

        # Arbitrate between behaviors
        final_cmd = self.arbitrate_behaviors(avoid_behavior, explore_behavior)

        # Publish command
        self.cmd_vel_publisher.publish(final_cmd)

    def avoid_obstacles_behavior(self):
        cmd = Twist()

        # If obstacle is too close, turn away
        if self.min_distance < 0.8:
            cmd.angular.z = 0.8 if self.min_angle > 0 else -0.8
            cmd.linear.x = 0.0
        elif self.min_distance < 1.2:
            # Turn away but continue moving forward slowly
            cmd.angular.z = 0.4 if self.min_angle > 0 else -0.4
            cmd.linear.x = 0.2
        else:
            # No immediate obstacles, no turning
            cmd.angular.z = 0.0
            cmd.linear.x = 0.0

        return cmd

    def explore_behavior(self):
        cmd = Twist()

        # Default exploration behavior - move forward
        cmd.linear.x = 0.5
        cmd.angular.z = 0.0

        return cmd

    def arbitrate_behaviors(self, avoid_behavior, explore_behavior):
        # If avoiding obstacles is active, prioritize it
        if abs(avoid_behavior.angular.z) > 0.1 or avoid_behavior.linear.x < 0.1:
            return avoid_behavior
        else:
            # Otherwise, blend behaviors (simplified)
            cmd = Twist()
            cmd.linear.x = explore_behavior.linear.x
            cmd.angular.z = avoid_behavior.angular.z
            return cmd

def main(args=None):
    rclpy.init(args=args)
    controller = BehaviorBasedController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()`}
  language="python"
/>

One of the key advantages of using Python for AI agents is the ability to easily integrate with popular AI libraries. Here's an example of how to use a simple machine learning model within a ROS 2 node:

<CodeExample
  title="AI Node with ML Integration"
  description="A node that uses a simple machine learning model to make decisions based on sensor data."
  code={`import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sklearn.linear_model import LinearRegression
import numpy as np

class AINodeWithML(Node):
    def __init__(self):
        super().__init__('ai_node_with_ml')

        # Publisher and subscriber
        self.publisher = self.create_publisher(String, 'ai_decision', 10)
        self.subscription = self.create_subscription(
            String, 'sensor_data', self.sensor_callback, 10)

        # Simple ML model (in practice, you'd load a trained model)
        self.model = LinearRegression()

        # Initialize with some dummy training data
        X_train = np.array([[1], [2], [3], [4], [5]])
        y_train = np.array([2, 4, 6, 8, 10])
        self.model.fit(X_train, y_train)

        self.get_logger().info('AI Node with ML initialized')

    def sensor_callback(self, msg):
        # In a real scenario, you'd parse the sensor data appropriately
        # For this example, we'll extract a number from the message
        try:
            sensor_value = float(msg.data.split(':')[-1]) if ':' in msg.data else 1.0
            prediction = self.model.predict([[sensor_value]])[0]

            # Make a decision based on the prediction
            if prediction > 5:
                decision = "ACTION_A"
            else:
                decision = "ACTION_B"

            # Publish the AI decision
            decision_msg = String()
            decision_msg.data = f"Decision: {decision}, Prediction: {prediction:.2f}"
            self.publisher.publish(decision_msg)
            self.get_logger().info(f'AI Decision: {decision_msg.data}')

        except ValueError:
            self.get_logger().warn(f'Could not parse sensor data: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    ai_node = AINodeWithML()

    try:
        rclpy.spin(ai_node)
    except KeyboardInterrupt:
        pass
    finally:
        ai_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()`}
  language="python"
/>

## Best Practices for AI Agents in ROS 2

When implementing AI agents with ROS 2, consider these best practices:

- **Modularity**: Keep AI logic separate from ROS communication logic
- **Performance**: Be mindful of computational requirements for real-time applications
- **Safety**: Implement safety checks before sending commands to robots
- **Error Handling**: Robust error handling for sensor failures or communication issues
- **Testing**: Thoroughly test in simulation before deploying to real robots
- **Resource Management**: Properly manage memory and computational resources
- **Logging**: Implement comprehensive logging for debugging and monitoring
- **Configuration**: Use ROS 2 parameters for configurable AI behavior

## Quality of Service (QoS) for AI Agents

When designing AI agents, consider the appropriate QoS settings for your use case:

- **Reliability**: Use "reliable" for critical control commands, "best effort" for sensor data
- **Durability**: Use "transient local" for parameters or initial states that new subscribers need
- **History**: Choose based on how much past data your AI algorithm needs

## Exercises

To practice implementing Python AI agents:

### Exercise 1: Simple Decision Node
Create a node that subscribes to a sensor topic and makes simple decisions based on the sensor values. The node should publish different control commands based on thresholds.

### Exercise 2: AI Controller
Implement an AI agent that subscribes to multiple sensor topics and uses a simple rule-based system to control a robot's movement.

### Exercise 3: ML Integration
Create a node that loads a pre-trained machine learning model and uses it to make decisions based on sensor input.

### Exercise 4: Parameterized AI
Implement an AI agent that uses ROS 2 parameters to configure its behavior at runtime.

## Troubleshooting Common Issues

When working with Python AI agents and ROS 2, you may encounter common issues. Here are solutions to the most frequent problems:

### Import Errors

**Problem**: `ModuleNotFoundError: No module named 'rclpy'`
**Solution**: Make sure you have sourced your ROS 2 installation and activated your Python environment:
```bash
source /opt/ros/humble/setup.bash  # Replace 'humble' with your ROS 2 distribution
source your_python_venv/bin/activate  # If using a virtual environment
```

### Node Initialization Issues

**Problem**: Node fails to initialize with errors about ROS domain or master URI
**Solution**: Ensure that the ROS 2 daemon is running:
```bash
# Check if ROS 2 daemon is running
ros2 daemon status

# Start the daemon if it's not running
ros2 daemon start
```

### Publisher/Subscriber Connection Problems

**Problem**: Nodes don't see each other or can't communicate
**Solution**:
1. Check that topic names match exactly between publishers and subscribers
2. Verify that message types are compatible
3. Check QoS settings if experiencing connection issues
4. Use `ros2 topic list` to see available topics
5. Use `ros2 topic echo <topic_name>` to verify data is being published

### Threading Issues

**Problem**: Callbacks not executing or nodes hanging
**Solution**:
- Don't use `rclpy.spin()` in a callback function
- Use `rclpy.spin_once()` for non-blocking execution when needed
- Be careful with multithreading - ROS 2 nodes are not thread-safe by default

### Memory Management

**Problem**: Memory leaks in long-running nodes
**Solution**:
- Properly destroy nodes with `node.destroy_node()`
- Use context managers when possible
- Monitor memory usage during development

### Performance Issues

**Problem**: High latency or slow execution
**Solution**:
- Reduce the frequency of high-computation operations
- Use appropriate QoS settings for your use case
- Consider using multithreading for I/O-bound operations

## Summary

Python AI agents with rclpy provide a powerful way to bridge AI decision-making with robot control systems. The combination of Python's rich AI ecosystem with ROS 2's robust communication framework enables the development of sophisticated robotic systems. The next chapter will cover how to model humanoid robots using URDF.