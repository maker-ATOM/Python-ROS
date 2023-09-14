# Publisher Subscriber model

## Description

ROS 2 is heavily based is heavily based on OOP.

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
```

Previously we used to define a node using rospy (in ROS 1) within the python script. But in ROS 2 the features are abstracted to class Node within the rclpy.node module which is in turn present in rclpy package used for managing and creating ROS 2 nodes.

We also import the built-in string message type that the node uses to structure the data that it passes on the topic.

```python
class MinimalPublisher(Node):
```
then we create the MinimalPublisher class, which inherits from (or is a subclass of) Node.

```python
def __init__(self):
    super().__init__('minimal_publisher')
    self.publisher_ = self.create_publisher(String, 'topic', 10)
    timer_period = 0.5  # seconds
    self.timer = self.create_timer(timer_period, self.timer_callback)
    self.i = 0
```

When an instance of this class is created the __init__ method is executed. This method initially calls the __init__method from the Node class to create a node of name 'minimal_publisher'. Which will publish data of type String on topic 'topic' with queue size = 10.
<br>
 Queue size is a required QoS (quality of service) setting that limits the amount of queued messages if a subscriber is not receiving them fast enough.

It then calls the method create_publisher which setups the created node as a publisher. self.publisher_ is assigned the publisher object returned by self.create_publisher, which uses publish method to send messages on the desired topic.

Output of various print statements:

```python
print(self.publisher_.publish(msg))
None

print(self.publisher_)
<rclpy.publisher.Publisher object at 0x7f5f0b89fca0>

```

Next a timer is associated with the instance which calls timer_callback function every 0.5 seconds.

```python
def timer_callback(self):
    msg = String()
    msg.data = 'Hello World: %d' % self.i
    self.publisher_.publish(msg)
    self.get_logger().info('Publishing: "%s"' % msg.data)
    self.i += 1
```

timer_callback creates a message with the counter value appended, and publishes it to the console with get_logger().info.

```python
def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()
```

The main function first initializes rclpy lib, creates a node, and then spins the node so that any callback function associated to that node is executed, which is timer_callback in this case.

## ROSGraph
<p align="center">
	<img src="images/rosgraph.png" width="596" height="72"/>
</p>