# Server Client Model

## Description

We wont bwe creating a custom service, but will be utilizing from example_interfaces - AddTwoInts whose contents are,

```python
int64 a
int64 b
---
int64 sum
```

**Server**

The first two lines are the parameters of the request, and below the dashes is the response.

```python
def __init__(self):
    super().__init__('minimal_service')
    self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)
```
The class constructor first initiates the node and then it creates a service and defines the type, service name and callback function.

```python
    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))
```
The callback associated with the service is executed whenever a request is raised. the function returns the operation data.

**Client**

```python
    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()
```

The constructor first create a client and attaches it to service name of the server. The client waits for the service to be available using a while loop, and once available an instance is created to store the request data.

```python
    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
```

This custom method stores the data into request instance and sends a asynchronous service request. This returns a object representing the ongoing service call. Node is then spun continuously until the service is complete. Async operation allows other ROS2 events to be processed while waiting for the service. the result is then returned.

```python
def main():
    rclpy.init()

    minimal_client = MinimalClientAsync()
    response = minimal_client.send_request(int(sys.argv[1]), int(sys.argv[2]))
    minimal_client.get_logger().info(
        'Result of add_two_ints: for %d + %d = %d' %
        (int(sys.argv[1]), int(sys.argv[2]), response.sum))

    minimal_client.destroy_node()
    rclpy.shutdown()
```
After the node is setup and ready to receive service, the `send_request` function is called which waits until response is received from the server. After reception of the service the received data is logged to console. And the node is destroyed with script being terminated.

## ROS Graph