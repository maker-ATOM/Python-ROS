# ROS-Learnings
This repository contains self learning and understanding of Robot Operating System.

**Python for ROS**

Core python concepts with OOP in python.

**ROS1**

Learnings of ROS1 (superficial).

**ROS2**

Learnings of ROS2 (in depth).

- ROS Wiki

**pubsub**

Basic utilization of publisher subscriber model.

- rclpy
- Node class

**srvcli**

Basic utilization of server client model.

**node_parameters**

Implementation of node parameters. 

## Repository Structure
```python
├── pubsub
│   ├── package.xml
│   ├── pubsub
│   │   ├── __init__.py
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
│       ├── __init__.py
│       └── server.py
│
│
├── node_parameters
│   ├── launch
│   │   └── parametric_node_launch.launch.py
│   ├── node_parameters
│   │   ├── __init__.py
│   │   └── parametric_node.py
│   ├── package.xml
│   ├── README.md
│   ├── setup.cfg
│   └── setup.py
│
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
└── README.md

```