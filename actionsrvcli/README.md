# Action Server Model

## Server

```python
    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback)
```
The init method setups a node name `fibonacci_action_server`. Then the we instantiate a new action server with the associated to the action client, Fibonacci as the action type the server will handle, 'fibonacci' as the action name and execute_callback method to be executed when action goal is received.

```python
    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        result = Fibonacci.Result()
        return result
```

If the callback had only this content the server will work, but we will gwt a warning stating goal not set. 

```python
    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        goal_handle.succeed()

        result = Fibonacci.Result()
        return result
```

We need to use succeed method() on goal handle to indicate that goal was successful. We will receive goal finished with the status SUCCEEDED. But in this case nothing is being computed specifically the fibonacci series.

```python
    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        sequence = [0, 1]
        for i in range(1, goal_handle.request.order):
            sequence.append(sequence[i] + sequence[i-1])

        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = sequence
        return result
```
Now the series is being calculated and stored within the sequence element of result.

```python
    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        feedback_msg = Fibonacci.Feedback()
        feedback_msg.partial_sequence = [0, 1]

        for i in range(1, goal_handle.request.order):
            feedback_msg.partial_sequence.append(
                feedback_msg.partial_sequence[i] + feedback_msg.partial_sequence[i-1])
            self.get_logger().info('Feedback: {0}'.format(feedback_msg.partial_sequence))
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1)

        goal_handle.succeed()

        result = Fibonacci.Result()
        result.sequence = feedback_msg.partial_sequence
        return result
```
We can also provide feedback to an action client during goal execution. We can make our action server publish feedback for action clients by calling the goal handle’s publish_feedback() method.

We’ll replace the sequence variable, and use a feedback message to store the sequence instead. After every update of the feedback message in the for-loop.

## Client

```python
    def __init__(self):
        super().__init__('fibonacci_action_client')
        self._action_client = ActionClient(self, Fibonacci, 'fibonacci')
```

In the init method we setup the node and instantiate a action client, arguments begin the ROS2 node utilizing the action service, type of action and action name.

```python
    def send_goal(self, order):
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        self._action_client.wait_for_server()

        return self._action_client.send_goal_async(goal_msg)
```
This method waits for the action server to be available, then sends a goal to the server. It returns a future that we can later wait on. This method only sends the goal to receive and does not takes result into account.

```python
    def send_goal(self, order):
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(goal_msg)

        self._send_goal_future.add_done_callback(self.goal_response_callback)
```

The modified send goal method sends the request and waits for the service to be available. The ActionClient.send_goal_async() method returns a future to a goal handle. First we register a callback for when the future is complete. Note that the future is completed when an action server accepts or rejects the goal request.

```python
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
```

After the action server rejects or accepts the goal request, goal_response_callback is executed. Within it is checked if rejected it returns early. If accepted a message is logged. Now that we’ve got a goal handle, we can use it to request the result with the method get_result_async(). Similar to sending the goal, we will get a future that will complete when the result is ready.In the callback, we log the result sequence and shutdown ROS 2 for a clean exit.

We can also receive feedbacks from teh server.

## ROS Graph

 <p align="center">
	<img src="images/rosgraph.png"/>
</p>