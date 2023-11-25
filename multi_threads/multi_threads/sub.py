import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class MinimalSubscriber(Node):

    def __init__(self, node_name, topic_name, callback):
        super().__init__(node_name) # type: ignore
        self.subscription = self.create_subscription(String, topic_name, callback, 10)

def listener_a_callback(msg):
    minimal_subscriber_a.get_logger().info("Entered callback a")
    time.sleep(5)
    minimal_subscriber_a.get_logger().info("Exited callback a")

def listener_b_callback(msg):
    minimal_subscriber_b.get_logger().info("Entered callback b")
    time.sleep(5)
    minimal_subscriber_b.get_logger().info("Exited callback b")

def main(args=None):
    rclpy.init(args=args)

    global minimal_subscriber_a
    global minimal_subscriber_b

    minimal_subscriber_a = MinimalSubscriber('minimal_subscriber_a', 'topic_a', listener_a_callback)
    minimal_subscriber_b = MinimalSubscriber('minimal_subscriber_b', 'topic_b', listener_b_callback)

    """
    Multi-threading
    """
    # executor = rclpy.executors.MultiThreadedExecutor()

    # executor.add_node(minimal_subscriber_a)
    # executor.add_node(minimal_subscriber_b)

    # executor.spin()

    """
    Sequential processing
    """
    while rclpy.ok():
        rclpy.spin_once(minimal_subscriber_a)
        rclpy.spin_once(minimal_subscriber_b)

        

    minimal_subscriber_a.destroy_node()
    minimal_subscriber_b.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
