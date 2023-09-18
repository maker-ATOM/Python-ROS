# Transforms

## Static Broadcaster

```python
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
```

The tf2_ros package provides a StaticTransformBroadcaster to make the publishing of static transforms easy.

```python
    def __init__(self, transformation):
        super().__init__('static_turtle_tf2_broadcaster')

        self.tf_static_broadcaster = StaticTransformBroadcaster(self)

        # Publish static transforms once at startup
        self.make_transforms(transformation)
```

The StaticFramePublisher class constructor initializes the node with the name static_turtle_tf2_broadcaster. Then, StaticTransformBroadcaster is created, which will send one static transformation upon the startup.

```python
    def make_transforms(self, transformation):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = transformation[1]

        t.transform.translation.x = float(transformation[2])
        t.transform.translation.y = float(transformation[3])
        t.transform.translation.z = float(transformation[4])
        quat = quaternion_from_euler(
            float(transformation[5]), float(transformation[6]), float(transformation[7]))
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        self.tf_static_broadcaster.sendTransform(t)
```

Before using `self.tf_static_broadcaster.sendTransform(t)` to broadcast the transform we need to give it the appropriate metadata.

## tf2 Broadcaster

The scripts publishes turtle pose to tf2.
```python
    def __init__(self):
        super().__init__('turtle_tf2_frame_publisher')

        # Declare and acquire `turtlename` parameter
        self.turtlename = self.declare_parameter(
          'turtlename', 'turtle').get_parameter_value().string_value

        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscribe to a turtle{1}{2}/pose topic and call handle_turtle_pose
        # callback function on each message
        self.subscription = self.create_subscription(
            Pose,
            f'/{self.turtlename}/pose',
            self.handle_turtle_pose,
            1)
        self.subscription  # prevent unused variable warning
```

 Firstly, we define and acquire a single parameter turtlename, which specifies a turtle name, e.g. turtle1 or turtle2. `self.declare_parameter('turtlename', 'turtle')`: This method declares a parameter named `turtlename` with a default value of `turtle`. If this parameter already exists, it will retrieve its value. If it doesn't exist, it will use the provided default value. The `turtlename` parameter is passed by via launch file or terminal.

Afterward, the node subscribes to topic turtleX/pose and runs function handle_turtle_pose on every incoming message.

```python
    def handle_turtle_pose(self, msg):
        t = TransformStamped()

        # Read message content and assign it to
        # corresponding tf variables
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = self.turtlename

        # Turtle only exists in 2D, thus we get x and y translation
        # coordinates from the message and set the z coordinate to 0
        t.transform.translation.x = msg.x
        t.transform.translation.y = msg.y
        t.transform.translation.z = 0.0

        # For the same reason, turtle can only rotate around one axis
        # and this why we set rotation in x and y to 0 and obtain
        # rotation in z axis from the message
        q = quaternion_from_euler(0, 0, msg.theta)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        # Send the transformation
        self.tf_broadcaster.sendTransform(t)
```

We then create a TransformStamped object and give it the appropriate metadata. Then set time as current and utilize the pose to publish the transform. The transform is then passed to`sendTransform` method which takes care of publishment.

To run the node with parameter name,
```python
ros2 run tf2_tutorial tf2_broadcaster --ros-args -p turtlename:=turtle1
```

The execution can be verified using,

```python
ros2 run tf2_ros tf2_echo world turtle1
```

**Note**: This above node also requires turtlesim to be running before hand. Also both the nodes can be used to run at the same time using launch file while also setting the turtlename parameter in the launch file itself.

## tf2_listener

```python
    def __init__(self):
        super().__init__('turtle_tf2_frame_listener')

        # Declare and acquire `target_frame` parameter
        self.target_frame = self.declare_parameter(
          'target_frame', 'turtle1').get_parameter_value().string_value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Create a client to spawn a turtle
        self.spawner = self.create_client(Spawn, 'spawn')
        # Boolean values to store the information
        # if the service for spawning turtle is available
        self.turtle_spawning_service_ready = False
        # if the turtle was successfully spawned
        self.turtle_spawned = False

        # Create turtle2 velocity publisher
        self.publisher = self.create_publisher(Twist, 'turtle2/cmd_vel', 1)

        # Call on_timer function every second
        self.timer = self.create_timer(1.0, self.on_timer)
```

First we declare a parameter `target_frame` which stores the parameter it receives if nothing is received then  default `turtle1` is assigned. Then we create a transform listener so that the node can listen to the broadcasted transforms by `tf2_broadcaster`. When we initiate a client for the service `spawn` but do not request any. Then we create publisher to publish velocity values of turtle2 and finally a callback timer.

```python
    def on_timer(self):
        # Store frame names in variables that will be used to
        # compute transformations
        from_frame_rel = self.target_frame
        to_frame_rel = 'turtle2'

        if self.turtle_spawning_service_ready:
            if self.turtle_spawned:
                # Look up for the transformation between target_frame and turtle2 frames
                # and send velocity commands for turtle2 to reach target_frame
                try:
                    t = self.tf_buffer.lookup_transform(
                        to_frame_rel,
                        from_frame_rel,
                        rclpy.time.Time())
                except TransformException as ex:
                    self.get_logger().info(
                        f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
                    return

                msg = Twist()
                scale_rotation_rate = 1.0
                msg.angular.z = scale_rotation_rate * math.atan2(
                    t.transform.translation.y,
                    t.transform.translation.x)

                scale_forward_speed = 0.5
                msg.linear.x = scale_forward_speed * math.sqrt(
                    t.transform.translation.x ** 2 +
                    t.transform.translation.y ** 2)

                self.publisher.publish(msg)
            else:
                if self.result.done():
                    self.get_logger().info(
                        f'Successfully spawned {self.result.result().name}')
                    self.turtle_spawned = True
                else:
                    self.get_logger().info('Spawn is not finished')
        else:
            if self.spawner.service_is_ready():
                # Initialize request with turtle name and coordinates
                # Note that x, y and theta are defined as floats in turtlesim/srv/Spawn
                request = Spawn.Request()
                request.name = 'turtle2'
                request.x = float(4)
                request.y = float(2)
                request.theta = float(0)
                # Call request
                self.result = self.spawner.call_async(request)
                self.turtle_spawning_service_ready = True
            else:
                # Check if the service is ready
                self.get_logger().info('Service is not ready')
```

In init method we defined a attribute `turtle_spawning_service_ready`, initially it is false, so we first call a spawn service to spawn the turtle if the service is available. After requesting the spawn service we set the attribute `turtle_spawning_service_ready` to true so that to ensure the service is requested only once. After we check if the spawn took place successfully or not using custom attribute `turtle_spawned`, initially it will be false and we check if spawn happened only once.

Then we receive the transform from turtle1 to turtle2 and calculate the twist values based upon distance and angle to turtle1.