# ROS2

<details open="open">
  <summary>Table of Contents</summary>
  <ol>
    <li><a href="#CLI - tools">ROS Wiki</a></li>
    <ol>
        <li><a href="#Environment">Environment</a></li>
        <li><a href="#RQT-and-ROS2-tools">RQT and ROS2 tools</a></li>
        <li><a href="#Nodes">Nodes</a></li>
        <li><a href="#Topics">Topics</a></li>
        <li><a href="#Services">Services</a></li>
        <li><a href="#Parameters">Parameters</a></li>
        <li><a href="#Actions">Actions</a></li>
        <li><a href="#Launch">Launch</a></li>
        <li><a href="#ROSbag">ROSbag</a></li>
    </ol>
    <li><a href="#Projects">Projects</a></li>
    <li><a href="#Resources">Resources</a></li>
    <!-- <li><a href="#Description-of-src-files">Description of src files</a></li>
    <li><a href="#Project-Status">Project Status</a></li>
    <li><a href="#Project-Upgradation">Project Upgradation</a></li> -->
  </ol>
</details>



**Learning Status**


- [x] ROS - Mecharithm
- [ ] ROS wiki
    - [ ] Tutorials
        - [x] CLI Tools
        - [ ] Client Libraries
        - [ ] Intermediate
        - [ ] Advance
        - [ ] Demos
        - [ ] Miscellaneous
    - [ ] How to Guides
    - [ ] Concepts
- [ ] Robotics Back-End
- [ ] The Construct
- [ ] Raspberry Pi
- [ ] Linux

## CLI - tools

### Environment

The core ROS 2 workspace is called the underlay. Subsequent local workspaces are called overlays.


 Without sourcing the setup files, we won’t be able to access ROS 2 commands, or find or use ROS 2 packages.


 Add this command to `.bashrc` file.
 ```python
 source /opt/ros/humble/setup.bash
 ```
 So that every time a new terminal session is initiated this command is executed and ROS 2 commands can be accessed. 


 ROS 2 nodes on the same domain can freely discover and send messages to each other, while ROS 2 nodes on different domains cannot.
 To avoid interference between different groups of computers running ROS 2 on the same network, a different domain ID should be set for each group.

 The domain ID can be changed using
 ```python
 export ROS_DOMAIN_ID=<our_domain_id>
 ```

 By default, ROS 2 communication is not limited to localhost. ROS_LOCALHOST_ONLY environment variable allows us to limit ROS 2 communication to localhost only. This means our ROS 2 system, and its topics, services, and actions will not be visible to other computers on the local network.

 ```python
 export ROS_LOCALHOST_ONLY=1
 ```


IF any ROS2 package is not being located, the first thing we should do is check our environment variables and ensure they are set to the version and distro we intended.

 We won’t be able to use the `sudo apt install ros-<distro>-<package>` command if we install from source.



### RQT and ROS2 tools

rqt is a graphical user interface (GUI) tool for ROS 2. Everything done in rqt can be done on the command line, but rqt provides a more user-friendly way to manipulate ROS 2 elements.

Use `rqt_graph` to visualize the changing nodes and topics, as well as the connections between them.

Use `rqt` to have a graphical user interface to publish to topic without creating a publisher node.  
 

To find executables within a package run,
```python
ros2 pkg executables <package_name>
```

List subcommand can be used to view all the available elements of the corresponding command.
```python
ros2 node list
ros2 topic list
ros2 service list
ros2 action list
```


The ROS graph is a network of ROS 2 elements processing data together at the same time. It encompasses all executables and the connections between them if we were to map them all out and visualize them.

`rqt_console` is a GUI tool used to introspect log messages in ROS 2. Typically, log messages show up in our terminal.
```python
ros2 run rqt_console rqt_console
```
ROS 2’s logger levels are ordered by severity:

```python
Fatal
Error
Warn
Info
Debug
```
There is no exact standard for what each level indicates, but it’s safe to assume that:

- **Fatal** messages indicate the system is going to terminate to try to protect itself from detriment.

- **Error** messages indicate significant issues that won’t necessarily damage the system, but are preventing it from functioning properly.

- **Warn** messages indicate unexpected activity or non-ideal results that might represent a deeper issue, but don’t harm functionality outright.

- **Info** messages indicate event and status updates that serve as a visual verification that the system is running as expected.

- **Debug** messages detail the entire step-by-step process of the system execution.

The default level is Info. We will only see messages of the default severity level and more-severe levels.


### Nodes
Each node in ROS should be responsible for a single, modular purpose, e.g. controlling the wheel motors or publishing the sensor data from a laser range-finder. Each node can send and receive data from other nodes via topics, services, actions, or parameters.

 <p align="center">
	<img src="Images/Nodes-TopicandService.gif"/>
</p>


The ROS system architecture must be designed in a way so that each individual task of the robot can be executed by individual nodes.


To run a executable,
```python
ros2 run <package_name> <executable_name>
```

The `node_name` and `executable_name` need not necessarily be same always.

**Remapping** allows us to reassign default node properties, like node name, topic names, service names, etc., to custom values.

### Topics
Topics are a vital element of the ROS graph that act as a bus for nodes to exchange messages.
A node may publish data to any number of topics and simultaneously have subscriptions to any number of topics.
Topics don't have be a point-point communication they can one-many, many-one and many-many.

 <p align="center">
	<img src="Images/Topic-SinglePublisherandSingleSubscriber.gif"/>
</p>

```python
ros2 topic list -t
``` 
will return the same list of topics, this time with the topic type appended in brackets.

```python
ros2 topic ...
```
Double tab to view all available introspection tools of ROS2.


### Services
Services are based on a call-and-response model versus the publisher-subscriber model of topics. While topics allow nodes to subscribe to data streams and get continual updates, services only provide data when they are specifically called by a client.
There can be many clients requesting a service from a server. But only one server exists serving the request.

 <p align="center">
	<img src="Images/
Service-SingleServiceClient.gif"/>
</p>


Unlike a topic - a one way communication pattern where a node publishes information that can be consumed by one or more subscribers - a service is a request/response pattern where a client makes a request to a node providing the service and the service processes the request and generates a response.

We generally don’t want to use a service for continuous calls; topics or even actions would be better suited.

Running the `ros2 service list` command in a new terminal will return a list of all the services currently active in the system:

To find out the type of a service, use the command:
```python
ros2 service type <service_name>
```

To find all the services of a specific type, we can use the command: 
```python
ros2 service find <type_name>
```

If run,
```python
ros2 interface show turtlesim/srv/Spawn
```
a service within turtlesim package, we will receive,

```python
float32 x
float32 y
float32 theta
string name # Optional.  A unique name will be created and returned if this is empty
---
string name
```

The information above the --- line tells us the arguments needed to call /spawn. x, y and theta determine the 2D pose of the spawned turtle, and name is clearly optional. The information below the line indicates the information received after the server serves the requests. 

To call a service,
```python
ros2 service call <service_name> <service_type> <arguments>
```

### Parameters
A parameter is a configuration value of a node. We can think of parameters as node settings. A node can store parameters as integers, floats, booleans, strings, and lists. In ROS 2, each node maintains its own parameters.

To see the parameters belonging to our nodes, open a new terminal and enter the command:
```python
ros2 param list
```

To display the type and current value of a parameter, use the command:

```python
ros2 param get <node_name> <parameter_name>
```

To change a parameter’s value at runtime, use the command:
```python
ros2 param set <node_name> <parameter_name> <value>
```

We can view all of a node’s current parameter values by using the command:
```python
ros2 param dump <node_name>
```
The command prints to the standard output (stdout) by default but we can also redirect the parameter values into a file to save them for later.

```python
ros2 param dump /<node_name> > <file_name>
```

To start the same node using our saved parameter values, use:
```python
ros2 run <package_name> <executable_name> --ros-args --params-file <file_name>
```

### Actions
Actions are one of the communication types in ROS 2 and are intended for long running tasks. They consist of three parts: a goal, feedback, and a result.


Actions are built on topics and services. Their functionality is similar to services, except actions can be canceled. They also provide steady feedback, as opposed to services which return a single response.


Actions use a client-server model, similar to the publisher-subscriber model (described in the topics tutorial). An “action client” node sends a goal to an “action server” node that acknowledges the goal and returns a stream of feedback and a result.

 <p align="center">
	<img src="Images/Action-SingleActionClient.gif"/>
</p>

The action client send a request to the action server stating that a goal(task) needs to be achieved. The task can be something like to get a robot joint reach a particular position.

The server keeps the client updating over the feedback topic.
When the server-side chooses to stop processing a goal, it is said to “abort” the goal.
Don’t assume every action server will choose to abort the current goal when it gets a new one.

Clients and servers are two different nodes.

To identify all the actions in the ROS graph, run the command:

```python
ros2 action list
```

Each action is encompassed in a rosmsg.
If we check the action provided within turtlesim,
```python
ros2 interface show turtlesim/action/RotateAbsolute

# The desired heading in radians
float32 theta
---
# The angular displacement in radians to the starting position
float32 delta
---
# The remaining rotation in radians
float32 remaining
```
The section of this message above the first --- is the structure (data type and name) of the goal request. The next section is the structure of the result. The last section is the structure of the feedback.

An example of action could be A robot system would likely use actions for navigation. An action goal could tell a robot to travel to a position. While the robot navigates to the position, it can send updates along the way (i.e. feedback), and then a final result message once it’s reached its destination.

### Launch
Launch files allow us to start up and configure a number of executables containing ROS 2 nodes simultaneously.

Running a single launch file with the ros2 launch command will start up our entire system - all nodes and their configurations - at once.

Launch files can be written in Python, XML and YAML.

### ROSbag
ros2 bag is a command line tool for recording data published on topics in our system. It accumulates the data passed on any number of topics and saves it in a database. We can then replay the data to reproduce the results of our tests and experiments. Recording topics is also a great way to share our work and allow others to recreate it.

ros2 bag can only record data from published messages in topics. ROSbag file will save in the directory where we run it.

To record the data published to a topic use the command syntax:
```python
ros2 bag record <topic_name>
```
We can record multiple topics and also can rename the bag file.
```python
ros2 bag record -o <file_name> <topic_1> <topic_2>
```

We also republish the data stored in the bag for introspection.
```python
ros2 bag play <file_name>
```

## Projects

## Resources

- [ROS 2 Documentation: Humble documentation](https://docs.ros.org/en/humble/index.html)
- [ROS2 Tutorials - ROS2 Humble For Beginners](https://www.youtube.com/playlist?list=PLLSegLrePWgJudpPUof4-nVFHGkB62Izy) - YouTube
- [UNIX / Linux Tutorial for Beginners](http://www.ee.surrey.ac.uk/Teaching/Unix/)