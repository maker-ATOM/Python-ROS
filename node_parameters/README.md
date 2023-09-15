# Node Parameters

## Description

```python
    def __init__(self):
        super().__init__('minimal_param_node')

        self.declare_parameter('my_parameter', 'world')

        self.timer = self.create_timer(1, self.timer_callback)
```

The line `self.declare_parameter('my_parameter', 'world')` of the constructor creates a parameter with the name my_parameter and a default value of world. The parameter type is inferred from the default value, so in this case it would be set to a string type.

```python
    def timer_callback(self):
        my_param = self.get_parameter('my_parameter').get_parameter_value().string_value

        self.get_logger().info('Hello %s!' % my_param)

        my_new_param = rclpy.parameter.Parameter(
            'my_parameter',
            rclpy.Parameter.Type.STRING,
            'world'
        )
        all_new_parameters = [my_new_param]
        self.set_parameters(all_new_parameters)
```

The first line of our timer_callback function gets the parameter my_parameter from the node, and stores it in my_param. Next the get_logger function ensures the event is logged. A new parameter is created with the same name 'my_parameter', a type of STRING, and an initial value of 'world'. This parameter is created using the rclpy.parameter.Parameter class.The set_parameters function then sets the parameter my_parameter back to the default string value world. In the case that the user changed the parameter externally, this ensures it is always reset back to the original.

The parameter value can be changed using terminal,

```python
ros2 param set <node_name> <parameter_name> <value>
```

and also using launch file.