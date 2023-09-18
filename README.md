# ROS-Learnings
This repository contains self learning and understanding of Robot Operating System.

## Python for ROS
Core python concepts with OOP in python.

## ROS1
Learnings of ROS1 (superficial).

## ROS2
Learnings of ROS2 (in depth).
<br>
- ROS Wiki

## pubsub
Basic utilization of publisher subscriber model.
<br>
- rclpy
- Node class

## srvcli
Basic utilization of server client model.

## node_parameters
Implementation of node parameters. 

## custom_interface
Package for custom interfaces.

## actionsrvcli
Basic utilization of Action server client model.

## tf2_tutorial
Usage of tf2 to broadcast and listen transforms.

## Repository Structure
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