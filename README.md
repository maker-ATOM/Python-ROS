# ROS-Learnings

<!-- <font size="5">This text has a larger font size.</font>

<span style="font-size: 20px;">This text has a font size of 20px.</span>

<span style="font-size: 20px;">This text has a font size of 20px.</span> -->

This repository contains self learning and understanding of Robot Operating System.

**Note**: pubsub, srvcli, node_parameters, custom_interface, actionsrvcli, tf2_tutorial are ROS2 packages and custom_interface contains custom rosmsg to represent a  Adjacency list used in project <a href="https://github.com/maker-ATOM/Path-Planning-Algorithms">Path-Planning-Algorithms</a>

<h3>Python for ROS</h3>
Core python concepts with OOP in python.

<h3>ROS1</h3>
Learnings of ROS1 (superficial).

<h3>ROS2</h3>
Learnings of ROS2 (in depth).
<br>
- ROS Wiki

<h3>pubsub</h3>
Basic utilization of publisher subscriber model.
<br>
- rclpy
- Node class

<h3>srvcli</h3>
Basic utilization of server client model.

<h3>node_parameters</h3>
Implementation of node parameters. 

<h3>custom_interface</h3>
Package for custom interfaces.

<h3>actionsrvcli</h3>
Basic utilization of Action server client model.

<h3>tf2_tutorial</h3>
Usage of tf2 to broadcast and listen transforms.

<h3>multi_threads</h3>
Association of threads to callback methods.

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
│   ├── msg
│   │   └── Point.msg
│   │   └── Neighbors.msg
│   │   └── AdjacencyList.msg
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