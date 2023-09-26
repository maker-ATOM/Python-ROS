# ROS-Learnings
This repository contains self learning and understanding of Robot Operating System.

### Python for ROS

```python
Core python concepts with OOP in python.
```

### ROS1

```python
Learnings of ROS1 (superficial).
```

## ROS2

```python
Learnings of ROS2 (in depth).
- ROS Wiki

```

### pubsub

```python
Basic utilization of publisher subscriber model.
- rclpy
- Node class
```

### srvcli

```python
Basic utilization of server client model.
```

### node_parameters

```python
Implementation of node parameters. 
```

### custom_interface

```python
Package for custom interfaces.
```

### actionsrvcli

```python
Basic utilization of Action server client model.
```

### tf2_tutorial

```python
Usage of tf2 to broadcast and listen transforms.
```

### Repository Structure
```python
├── Python for ROS
│   ├── joint.py
│   └── README.md
│
├── ROS1
│   ├── Image
│   └── README.md
│
├── ROS2
│    ├── Images
│    └── README.md
│
├── pubsub
│   ├── package.xml
│   ├── pubsub
│   │   ├── publisher.py
│   │   └── subscriber.py
│   ├── README.md
│   ├── setup.cfg
│   └── setup.py
│
├── srvcli
│   ├── package.xml
│   ├── README.md
│   ├── setup.cfg
│   ├── setup.py
│   └── srvcli
│       ├── client.py
│       └── server.py
│
├── node_parameters
│   ├── launch
│   │   └── parametric_node_launch.launch.py
│   ├── node_parameters
│   │   └── parametric_node.py
│   ├── package.xml
│   ├── README.md
│   ├── setup.cfg
│   └── setup.py
│
├── actionsrvcli
│   ├── actionsrvcli
│   │   ├── actionclient.py
│   │   ├── actionserver.py
│   ├── package.xml
│   ├── README.md
│   ├── setup.cfg
│   └── setup.py
│
├── custom_interfaces
│   ├── action
│   │   └── Fibonacci.action
│   ├── CMakeLists.txt
│   ├── package.xml
│   └── README.md
│
├── tf2_tutorial
│    ├── package.xml
│    ├── README.md
│    ├── setup.cfg
│    ├── setup.py
│    └── tf2_tutorial
│        ├── static_broadcaster.py
│        ├── tf2_broadcaster.py
│        └── tf2_listener.py
│
│
└── README.md
```